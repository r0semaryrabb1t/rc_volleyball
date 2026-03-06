# 宇树 GO-8010 电机

## 📦 电机概述

宇树 GO-8010 电机力位混合控制，支持 RS485 通信。

## 🔧 配置

### 电机配置

编辑 `src/motor_control_ros2/config/motors.yaml`：

```yaml
serial_interfaces:
  - device: /dev/ttyUSB0
    baudrate: 4000000
    motors:
      - name: fl_hip_motor
        type: GO8010
        id: 0
        direction: 1
        offset: 0.0
```

### SDK 路径配置（树梅派）

编辑 `src/motor_control_ros2/CMakeLists.txt`：

```cmake
set(UNITREE_SDK_PATH "/home/sunrise/unitree_actuator_sdk")
set(UNITREE_SDK_LIB "${UNITREE_SDK_LIB_DIR}/libUnitreeMotorSDK_Arm64.so")
```

## 📡 控制话题

### 订阅

```bash
ros2 topic pub --once /unitree_go8010_command motor_control_ros2/msg/UnitreeGO8010Command \
  '{joint_name: "fl_hip_motor", mode: 0, kp: 0.0, kd: 0.01, pos_des: 0.0, vel_des: 100.0, torque_ff: 0.0}'
```

### 参数说明

| 参数 | 说明 |
|------|------|
| mode | 控制模式 |
| kp | 位置增益 |
| kd | 速度增益 |
| pos_des | 目标位置 (弧度) |
| vel_des | 目标速度 (弧度/秒) |
| torque_ff | 前馈力矩 |

## 📊 状态话题

```bash
ros2 topic echo /unitree_go8010_states
```

---

**更新时间**: 2026-01-15
