#!/usr/bin/python3.10
"""
宇树 UNITREE_GO8010 电机零点校准工具

将电机当前位置设置为 0 度基准点 (offset = current_position)。

用法:
  python3 calibrate_zero_position.py              # 一键校准所有宇树电机
  python3 calibrate_zero_position.py --dry-run    # 仅查看当前位置，不修改配置
  python3 calibrate_zero_position.py --names Unitree8010_1,Unitree8010_2  # 指定电机

前提:
  - unitree_motor_node 正在运行
  - 电机已上电（可直接拖动到目标位置后执行校准）
"""

import rclpy
from rclpy.node import Node
from motor_control_ros2.msg import UnitreeGO8010State
import yaml
import re
import os
import subprocess
import sys
import math
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(SCRIPT_DIR)))
CONFIG_PATH = os.path.join(PROJECT_ROOT, 'src/motor_control_ros2/config/motors.yaml')


def load_motor_names():
    """从 motors.yaml 自动读取所有 UNITREE_GO8010 电机名称"""
    try:
        with open(CONFIG_PATH, 'r') as f:
            config = yaml.safe_load(f)
    except Exception as e:
        print(f"❌ 无法读取配置文件: {CONFIG_PATH}\n   {e}")
        sys.exit(1)

    names = []
    for iface in config.get('serial_interfaces', []):
        for motor in iface.get('motors', []):
            if motor.get('type') == 'UNITREE_GO8010':
                names.append(motor['name'])
    if not names:
        print("❌ 未在配置中找到 UNITREE_GO8010 电机")
        sys.exit(1)
    return names


def get_motor_names():
    """解析命令行参数或自动检测"""
    for arg in sys.argv:
        if arg.startswith('--names='):
            return arg.split('=', 1)[1].split(',')
    return load_motor_names()


MOTOR_NAMES = get_motor_names()


def read_positions(node, timeout=5.0):
    """订阅话题读取所有电机当前位置（弧度），复用外部传入的 node"""
    positions = {}

    def cb(msg):
        if msg.joint_name in MOTOR_NAMES and msg.online:
            positions[msg.joint_name] = msg.position

    sub = node.create_subscription(UnitreeGO8010State, '/unitree_go8010_states', cb, 10)

    start = time.time()
    while time.time() - start < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        if len(positions) >= len(MOTOR_NAMES):
            break

    node.destroy_subscription(sub)
    return positions


def patch_offset(motor_name, new_offset):
    """在 motors.yaml 中精确替换或添加指定电机的 offset 值"""
    # 先读取并备份原文件
    with open(CONFIG_PATH, 'r', encoding='utf-8') as f:
        content = f.read()

    backup_path = CONFIG_PATH + '.bak'
    with open(backup_path, 'w', encoding='utf-8') as bf:
        bf.write(content)

    # 尝试匹配已有的 offset 字段（更宽松的缩进匹配，兼容 2/4 空格）
    pattern = (
        r'(- name: ' + re.escape(motor_name) + r'\s*\n'
        r'(?:[ \t]{2,6}\w+:.*\n)*?'
        r'[ \t]{2,6}offset:[ \t]*)'
        r'[-+]?\d*\.?\d+'
    )
    if re.search(pattern, content):
        new_content = re.sub(pattern, r'\g<1>' + f'{new_offset:.6f}', content)
        # 原子写入
        tmp_path = CONFIG_PATH + '.tmp'
        with open(tmp_path, 'w', encoding='utf-8') as tf:
            tf.write(new_content)
        os.replace(tmp_path, CONFIG_PATH)
        return True

    # offset 字段不存在，在 motor 块末尾插入（兼容不同缩进）
    pattern2 = (
        r'(- name: ' + re.escape(motor_name) + r'\s*\n'
        r'((?:[ \t]{2,6}\w+:.*\n)*))'
    )
    match = re.search(pattern2, content)
    if match:
        insert_pos = match.end()
        # 使用与当前块相同缩进（取第一行缩进）
        block = match.group(2)
        indent_match = re.search(r'^[ \t]+', block, re.M)
        indent = indent_match.group(0) if indent_match else '    '
        insert_text = f'{indent}offset: {new_offset:.6f}\n'
        new_content = content[:insert_pos] + insert_text + content[insert_pos:]
        tmp_path = CONFIG_PATH + '.tmp'
        with open(tmp_path, 'w', encoding='utf-8') as tf:
            tf.write(new_content)
        os.replace(tmp_path, CONFIG_PATH)
        return True

    print(f"  ❌ 未在配置中找到电机: {motor_name}")
    return False


def restart_node():
    """重启 unitree_motor_node"""
    print("\n🔄 重启 unitree_motor_node ...")
    subprocess.run(['pkill', '-9', '-f', 'unitree_motor_node'],
                   stderr=subprocess.DEVNULL)
    time.sleep(1.5)

    # 使用源码路径作为参数启动节点，确保读取到刚写入的 motors.yaml
    setup_cmd = (f'source {PROJECT_ROOT}/install/setup.bash && '
                 f'ros2 run motor_control_ros2 unitree_motor_node '
                 f'--ros-args -p config_file:="{CONFIG_PATH}"')
    proc = subprocess.Popen(['bash', '-c', setup_cmd],
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL,
                            preexec_fn=os.setpgrp)
    print(f"  ✓ 节点已启动 (PID={proc.pid})，等待初始化...")
    time.sleep(3)
    return proc


def verify_zero(node, timeout=5.0):
    """验证所有电机位置是否接近 0"""
    positions = read_positions(node, timeout)
    print("\n=== 零位验证 ===")
    all_ok = True
    for name in MOTOR_NAMES:
        if name not in positions:
            print(f"  {name}: ❌ 未收到数据")
            all_ok = False
            continue
        deg = math.degrees(positions[name])
        mark = "✅" if abs(deg) < 2.0 else "⚠️"
        print(f"  {name}: {deg:+.4f}° {mark}")
        if abs(deg) >= 2.0:
            all_ok = False
    return all_ok


def main():
    dry_run = '--dry-run' in sys.argv

    # 全局仅初始化一次 rclpy
    rclpy.init()
    node = rclpy.create_node('zero_calibrator')

    print("\n" + "=" * 55)
    print("  宇树 UNITREE_GO8010 电机零点校准")
    if dry_run:
        print("  [预览模式 — 不修改配置]")
    print("=" * 55)
    print(f"📋 目标电机 ({len(MOTOR_NAMES)}): {', '.join(MOTOR_NAMES)}")

    # 1. 读取当前位置
    print("\n📡 读取电机当前位置...")
    positions = read_positions(node, timeout=5.0)

    if len(positions) < len(MOTOR_NAMES):
        missing = set(MOTOR_NAMES) - set(positions.keys())
        print(f"\n❌ 仅读取到 {len(positions)}/{len(MOTOR_NAMES)} 个电机")
        if missing:
            print(f"   缺失/离线: {missing}")
        print("   请确认 unitree_motor_node 正在运行且电机已上电")
        sys.exit(1)

    # 2. 显示当前角度
    print("\n📐 当前各电机角度:")
    print("-" * 40)
    for name in MOTOR_NAMES:
        if name in positions:
            deg = math.degrees(positions[name])
            print(f"  {name:20s}: {deg:+8.4f}°")
        else:
            print(f"  {name:20s}: 无数据")
    print("-" * 40)

    if dry_run:
        print("\n[预览模式] 跳过 offset 写入和节点重启。")
        print("如需校准，请执行: python3 calibrate_zero_position.py")
        node.destroy_node()
        rclpy.shutdown()
        return

    # 3. 确认操作
    print("\n⚠️  即将把所有电机当前位置设置为 0 度基准点，并重启节点。")
    confirm = input("确认继续? [y/N]: ").strip().lower()
    if confirm not in ('y', 'yes'):
        print("已取消。")
        node.destroy_node()
        rclpy.shutdown()
        return

    # 4. 写入新 offset（令当前位置读为 0：output = position - offset → offset = position）
    print("\n📝 更新 offset:")
    for name in MOTOR_NAMES:
        if name not in positions:
            continue
        cur_rad = positions[name]
        ok = patch_offset(name, cur_rad)
        if ok:
            print(f"  {name}: {math.degrees(cur_rad):+.2f}° → offset = {cur_rad:.6f} rad")
    print("✅ 配置已更新")

    # 5. 销毁旧节点，重启 motor_node 使新 offset 生效
    node.destroy_node()
    rclpy.shutdown()
    proc = restart_node()

    # 6. 重新初始化 rclpy 并验证
    time.sleep(2)
    rclpy.init()
    node2 = rclpy.create_node('zero_calibrator_v')
    ok = verify_zero(node2)
    if ok:
        print("\n✅ 零点校准完成！所有电机位置 < ±2°")
    else:
        print("\n⚠️  部分电机偏差较大，可再次执行校准")

    # 7. 清理（用户自行重启）
    node2.destroy_node()
    rclpy.shutdown()
    print("\n🧹 清理...")
    subprocess.run(['pkill', '-9', '-f', 'unitree_motor_node'],
                   stderr=subprocess.DEVNULL)
    print("  ✓ unitree_motor_node 已终止（请手动重新启动）\n")


if __name__ == '__main__':
    main()
