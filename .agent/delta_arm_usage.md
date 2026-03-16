# Delta 机械臂使用指南

## 概述

本项目的 Delta 机械臂为**一自由度德尔塔并联机构**，由三个宇树 GO-M8010-6 电机同角度驱动，实现末端平台的上下升降运动。

核心特点：
1. **三电机同步**：三个电机共享同一目标角度，通过并联连杆将旋转运动转换为直线升降
2. **多串口同 ID**：每个电机独占一个串口（USB2Serial），ID 均为 0，通过 `device` 字段区分
3. **逆运动学控制**：输入目标高度（米），自动计算关节角度

---

## 系统架构

```
遥控器(Joy)
    │
    ▼
RemoteControlNode          (左扳机 LT → 高度映射)
    │ /delta_arm_command
    ▼
DeltaArmNode               (逆运动学 → FOC 命令)
    │ /unitree_go8010_command
    ▼
motor_control_node          (协议编码 → 串口收发)
    │ 三个串口: /dev/ttyUSB0, USB1, USB2
    ▼
三个 GO-M8010-6 电机
```

---

## 启动步骤

### 步骤 1：确认硬件连接

确保三个 USB 转串口连接正确，查看设备号：

```bash
ls /dev/ttyUSB*
```

预期输出类似：`/dev/ttyUSB0  /dev/ttyUSB1  /dev/ttyUSB2`

> 设备号可能因插拔顺序变化，需与 `delta_arm.yaml` 的 `motor_devices` 一一对应。

### 步骤 2：编译

```bash
cd ~/111/USB2CAN_motor
source /opt/ros/humble/setup.bash
colcon build --packages-select motor_control_ros2
source install/setup.bash
```

### 步骤 3：启动电机控制节点（底层硬件通信）

```bash
ros2 run motor_control_ros2 motor_control_node
```

此节点负责：串口打开、协议编解码、收发电机帧。

### 步骤 4：启动 Delta 机械臂节点

```bash
ros2 run motor_control_ros2 delta_arm_node
```

此节点负责：接收高度命令 → 逆运动学 → 发送 FOC 指令。

### 步骤 5：启动遥控器节点（可选）

```bash
ros2 run motor_control_ros2 remote_control_node
```

通过 Xbox 手柄的**左扳机 (LT)** 控制机械臂高度。

---

## 手动测试（不需要遥控器）

### 高度控制模式

```bash
# 设置目标高度为 0.25 米
ros2 topic pub --once /delta_arm_command motor_control_ros2/msg/DeltaArmCommand \
  "{header: {stamp: {sec: 0}}, mode: 1, target_height: 0.25, velocity_scale: 0.5}"
```

### 回零模式

```bash
ros2 topic pub --once /delta_arm_command motor_control_ros2/msg/DeltaArmCommand \
  "{header: {stamp: {sec: 0}}, mode: 2, target_height: 0.0, velocity_scale: 0.0}"
```

### 空闲模式（电机刹车）

```bash
ros2 topic pub --once /delta_arm_command motor_control_ros2/msg/DeltaArmCommand \
  "{header: {stamp: {sec: 0}}, mode: 0, target_height: 0.0, velocity_scale: 0.0}"
```

### 查看机械臂状态

```bash
ros2 topic echo /delta_arm_state
```

输出字段说明：

| 字段 | 含义 |
|------|------|
| `online` | 电机是否在线 |
| `mode` | 当前控制模式 (0=空闲, 1=高度控制, 2=回零) |
| `current_height` | 当前末端高度（米） |
| `target_height` | 目标末端高度（米） |
| `joint_angle` | 当前关节角度（度） |
| `motor_position` | 电机原始位置（弧度） |
| `motor_temperature` | 电机温度（C） |
| `motor_error` | 电机错误码（0=正常） |

---

## 配置文件说明

配置文件路径：`src/motor_control_ros2/config/delta_arm.yaml`

### 参数单位速查表

| 参数 | 单位 | 说明 |
|------|------|------|
| `geometry.a/b/c/r` | 米 (m) | 几何尺寸 |
| `limits.height_min/max` | 米 (m) | 高度范围 |
| `limits.joint_angle_min/max` | **度 (°)** | 关节角范围（运动学层，人类直觉） |
| `motion.max_velocity` | 米/秒 (m/s) | 末端升降速度 |
| `motion.acceleration` | 米/秒² (m/s²) | 加速度 |
| `initial.height` | 米 (m) | 上电初始高度 |
| `initial.homing_velocity` | 米/秒 (m/s) | 回零速度 |
| `motor.kp / kd` | 无量纲 | FOC 位置/速度增益 |
| `motor.direction` | 无量纲 | 1 或 -1 |
| `motor_offsets_rad` | **弧度 (rad)** | 安装补偿（直接叠加到电机 FOC 命令） |

> **为什么度和弧度混用？** 关节角限位服务于运动学层，用度更直觉；offset 直接叠加到宇树电机的 `position_target`（原生单位弧度），所以用弧度。

### 几何参数

```yaml
geometry:
  a: 0.26     # 臂杆长度（米）——连接电机到末端平台的连杆
  b: 0.250    # 曲柄长度（米）——电机输出轴到臂杆铰接点
  c: 0.370    # 水平偏移（米）——电机轴到中心轴水平距离
  r: 0.380    # 末端半径（米）——末端平台铰接点到中心距离
```

**运动学公式**：

```
正运动学 (角度→高度):
  cos(alpha) = (b*cos(theta) + c - r) / a
  h = b*sin(theta) + a*sin(alpha)

逆运动学 (高度→角度):
  A = -2*h*b,  B = 2*b*(c-r),  C = a^2 - b^2 - (c-r)^2 - h^2
  theta = asin(C / sqrt(A^2+B^2)) - atan2(B, A)
```

### 运动范围

```yaml
limits:
  height_min: 0.171       # 最低高度（米）
  height_max: 0.472       # 最高高度（米）
  joint_angle_min: -360.0 # 最小关节角（度）
  joint_angle_max: 390.0  # 最大关节角（度）
```

> 节点启动时会自动采样关节角范围，计算**几何可达高度包络**并裁剪 `height_min/max`，日志会打印实际可达范围。

### FOC 电机参数

```yaml
motor:
  kp: 0.50       # 关节刚度（位置增益）
  kd: 0.04       # 关节阻尼（速度增益）
  direction: 1   # 角度符号翻转（1 或 -1）
```

### 电机映射（多串口同 ID）

```yaml
# 三个电机都是 ID=0，通过设备路径区分
motor_ids:
  - 0
  - 0
  - 0

motor_names:
  - arm_motor_1
  - arm_motor_2
  - arm_motor_3

motor_devices:
  - /dev/ttyUSB0
  - /dev/ttyUSB1
  - /dev/ttyUSB2
```

### 安装补偿

```yaml
# 单位：弧度
# 语义：物理角 = 运动学角 + offset（乘以 direction 之前）
motor_offsets_rad:
  - -0.2688   # arm_motor_1 (-15.4 度)
  - -0.0297   # arm_motor_2 (-1.7 度)
  - -0.0367   # arm_motor_3 (-2.1 度)
```

---

## 安装补偿标定步骤

由于三个电机的机械安装角度不完全一致，需要单独标定 offset。

### 步骤 1：将所有 offset 置零

编辑 `delta_arm.yaml`：

```yaml
motor_offsets_rad:
  - 0.0
  - 0.0
  - 0.0
```

### 步骤 2：启动节点并记录各电机角度

1. 启动 `motor_control_node` 和 `motor_monitor_node`
2. **手动将末端平台移到一个已知参考位置**（如最低点或水平位置）
3. 记录三个电机的反馈位置（弧度）：

```bash
ros2 topic echo /unitree_go8010_states
```

记录每个电机的 `position` 字段值。

### 步骤 3：计算 offset

以 arm_motor_1 为基准（或理论值），其他电机的 offset 为：

```
offset_i = 实测物理角_i - 基准角
```

例如：三个电机在同一高度时反馈分别为 `0.500, 0.770, 0.536` 弧度，以 motor_1 为基准：

```yaml
motor_offsets_rad:
  - 0.0       # 基准
  - 0.270     # 0.770 - 0.500
  - 0.036     # 0.536 - 0.500
```

### 步骤 4：验证

重新启动节点，发送高度命令，观察三个电机是否同步运动，末端平台是否水平。

---

## 遥控器操作说明（Xbox 手柄）

| 控件 | 功能 |
|------|------|
| 左扳机 (LT) | 控制机械臂高度：未按=最低，全按=最高 |
| 左摇杆 | 底盘前后/左右运动 |
| 右摇杆 X | 底盘旋转 |
| RB | 加速（每按一下 +50%） |
| LB | 减速（每按一下 -50%） |
| Back | 紧急停止 |

LT 扳机映射逻辑：

```
未按下: raw = 1.0  → trigger_norm = 0.0 → 最低高度
完全按下: raw = -1.0 → trigger_norm = 1.0 → 最高高度

目标高度 = arm_height_min + trigger_norm × (arm_height_max - arm_height_min)
```

遥控器的高度范围在 `remote_control_node` 的参数中配置：

```yaml
arm_height_min: 0.10   # LT 未按时的高度
arm_height_max: 0.30   # LT 全按时的高度
arm_enabled: true       # 是否启用机械臂控制
```

---

## ROS Topic 列表

| Topic | 类型 | 方向 | 说明 |
|-------|------|------|------|
| `/delta_arm_command` | DeltaArmCommand | 遥控器 → 机械臂 | 高度/模式命令 |
| `/delta_arm_state` | DeltaArmState | 机械臂 → 外部 | 实时状态反馈 |
| `/unitree_go8010_command` | UnitreeGO8010Command | 机械臂 → 底层 | FOC 电机命令 |
| `/unitree_go8010_states` | UnitreeGO8010State | 底层 → 机械臂 | 电机状态反馈 |

---

## 故障排查

### 问题 1：节点启动报 "初始高度无逆运动学解"

**原因**：`initial.height` 超出几何可达范围

**解决**：
1. 查看日志中打印的 `几何可达高度范围: [x, y]`
2. 将 `initial.height` 设在该范围内

### 问题 2：电机不响应命令

**排查步骤**：
1. 确认 `motor_control_node` 已启动且串口打开成功
2. 确认 `motors.yaml` 中三个宇树电机的 `device` 与 `delta_arm.yaml` 的 `motor_devices` 一致
3. 用 `ros2 topic echo /unitree_go8010_command` 检查是否有命令发出
4. 检查电机 24V 供电是否正常

### 问题 3：三个电机不同步（末端平台倾斜）

**原因**：安装补偿不正确

**解决**：按照上方"安装补偿标定步骤"重新标定 `motor_offsets_rad`

### 问题 4：设备号变化（ttyUSB 编号乱序）

**原因**：USB 插拔顺序不同导致内核分配不同编号

**解决方案**：
1. 使用 udev 规则固定设备号（推荐）
2. 或每次启动前确认对应关系：
   ```bash
   # 查看各串口设备信息
   udevadm info -a /dev/ttyUSB0 | grep serial
   ```

### 问题 5：目标高度不可达，日志报 "自动投影到最近可达高度"

**原因**：请求高度超出几何约束

**说明**：这是正常保护行为，节点会自动将目标投影到最近可达高度点。若频繁出现，考虑调整遥控器的 `arm_height_min/max` 使其落在可达范围内。

---

## 配置文件模板

完整的 `delta_arm.yaml`：

```yaml
# Delta 机械臂配置文件
geometry:
  a: 0.26
  b: 0.250
  c: 0.370
  r: 0.380

limits:
  height_min: 0.171
  height_max: 0.472
  joint_angle_min: -360.0
  joint_angle_max: 390.0

motion:
  max_velocity: 0.15
  acceleration: 0.5
  control_frequency: 200.0

initial:
  height: 0.20
  homing_velocity: 0.05

motor:
  kp: 0.50
  kd: 0.04
  direction: 1

motor_ids:
  - 0
  - 0
  - 0

motor_names:
  - arm_motor_1
  - arm_motor_2
  - arm_motor_3

motor_devices:
  - /dev/ttyUSB0    # ← 根据实际接线调整
  - /dev/ttyUSB1
  - /dev/ttyUSB2

motor_offsets_rad:
  - -0.2688         # ← 需要标定
  - -0.0297
  - -0.0367
```

---

## 检查清单

- [ ] 三个串口设备已连接且设备号正确
- [ ] `motors.yaml` 中三个宇树电机配置正确
- [ ] `delta_arm.yaml` 的 `motor_devices` 与实际设备对应
- [ ] `motor_offsets_rad` 已标定
- [ ] `motor_control_node` 启动成功，串口打开无报错
- [ ] `delta_arm_node` 启动成功，几何可达范围日志正常
- [ ] 手动发送高度命令测试通过
- [ ] 遥控器 LT 扳机控制机械臂高度正常
- [ ] 三个电机同步运动，末端平台水平

---

**文档版本**: v1.0
**更新时间**: 2026-03-14
**适用系统**: motor_control_ros2 (Delta 并联机械臂)
