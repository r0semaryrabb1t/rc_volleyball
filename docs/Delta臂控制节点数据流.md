# Delta 机械臂控制节点数据流分析

## 1. 系统拓扑

```
                        /delta_arm_command (DeltaArmCommand)
                            │  mode, target_height, velocity_scale
                            ▼
                 ┌─────────────────────────┐
                 │    delta_arm_node        │  配置: delta_arm.yaml
                 │                          │  ├─ 几何参数 (a, b, c, r)
                 │  ① 逆运动学 (h → θ)     │  ├─ 运动范围限制
                 │  ② 可达高度投影          │  ├─ kp, kd, direction, offsets_rad
                 │  ③ offset+direction 变换 │  └─ motor_ids / motor_devices
                 │  ④ 三电机同角度下发      │
                 └──────┬──────────────────┘
                        │                          ▲
       publish (×3 msg) │                          │ subscribe
                        ▼                          │
            /unitree_go8010_command        /unitree_go8010_states
            (UnitreeGO8010Command)         (UnitreeGO8010State)
                        │                          ▲
                        ▼                          │
                 ┌─────────────────────────┐       │
                 │   motor_control_node    │  配置: motors.yaml
                 │                         │  └─ gear_ratio (仅此一项)
                 │   setFOCCommand(        │
                 │     pos, vel, kp, kd, τ)│  纯透传，不做坐标变换
                 │   原生串口 SendRecv     │
                 └─────────────────────────┘
                        │
                   RS-485 Serial (4Mbps)
              ┌─────────┼─────────┐
         /dev/ttyUSB0  USB1     USB2
          arm_motor_1  _2       _3
          (GO8010 ID0) (ID0)   (ID0)
```

**注意**: 三个电机都是 ID=0，靠独立串口区分（多串口同 ID 场景）。

## 2. 参数归属（干净分离，无交叉）

| 参数 | 唯一来源 | 归属节点 |
|------|---------|---------|
| kp / kd | delta_arm.yaml | delta_arm_node |
| direction | delta_arm.yaml | delta_arm_node |
| offset (rad) | delta_arm.yaml | delta_arm_node |
| gear_ratio | motors.yaml | motor_control_node |

**与底盘模式一致**: 应用层参数（offset/direction/kp/kd）在上层节点的 YAML 中，硬件参数（gear_ratio）在 motors.yaml 中，完全不交叉。

## 3. 详细数据流

### 3.1 输入: /delta_arm_command → delta_arm_node

```
DeltaArmCommand:
  mode:
    MODE_HEIGHT (0) — 高度控制
    MODE_HOME   (1) — 回零
    MODE_IDLE   (2) — 空闲 (刹车)
  target_height:      目标高度 (米)
  velocity_scale:     速度缩放因子 [0.0, 1.0]
```

### 3.2 控制循环 (200 Hz)

```
每个周期:
  Step 1: 读取电机反馈，反向变换
      对所有在线电机:
        theoretical_rad = fb.position × direction - offset[i]
        angle_deg = theoretical_rad × 180/π
      current_height = forwardKinematics(avg_angle)

  Step 2: 根据状态机执行控制
      RUNNING / HOMING → sendMotorCommands(target_angle, velocity_scale)
      ERROR            → sendBrakeCommands()
      IDLE             → 不发命令 (保持刹车)

  Step 3: 发布 /delta_arm_state
```

### 3.3 坐标变换

**命令路径** (运动学 → 物理):
```
kinematic_rad = target_angle_deg × π/180
physical_cmd  = (kinematic_rad + offset[i]) × direction
```
代码: `delta_arm_node.cpp:sendMotorCommands()`

**反馈路径** (物理 → 运动学):
```
theoretical_rad = fb.position × direction - offset[i]
angle_deg       = theoretical_rad × 180/π
```
代码: `delta_arm_node.cpp:controlLoop()`

**等价性验证** (direction=1 时):
- 命令: `(kinematic + offset) × 1 = kinematic + offset`
- 反馈: `raw × 1 - offset = raw - offset`
- 与重构前行为完全一致

### 3.4 输出: delta_arm_node → motor_control_node

每个控制周期发送 **3 条** `UnitreeGO8010Command` (三电机各自 offset):

```
UnitreeGO8010Command:
  id              = 0                          (三个电机都是 ID 0)
  device          = "/dev/ttyUSB0" / "1" / "2" (靠 device 区分电机)
  mode            = 1 (MODE_FOC)
  position_target = (kinematic_rad + offset[i]) × direction   [rad]
  velocity_target = velocity_scale × max_velocity × direction  [m/s]
  kp              = config_.kp (0.50)           ← 来自 delta_arm.yaml
  kd              = config_.kd (0.04)           ← 来自 delta_arm.yaml
  torque_ff       = 0.0
```

### 3.5 motor_control_node 处理

`unitreeGOCommandCallback()`:
```
按 msg->id + msg->device 匹配电机
  → 多串口同 ID: device 非空时精确匹配，空时广播

MODE_FOC:
  motor->setFOCCommand(pos, vel, kp, kd, torque_ff)
      直接存储，无任何变换

MODE_BRAKE:
  motor->setBrakeCommand()
      所有参数归零
```

### 3.6 UnitreeMotorNative 命令构建

`buildCommandPacket()`:
```
纯透传（无 offset/direction 变换）:
  pos_cmd = cmd_pos_des_        ← 上层已处理好的物理值
  vel_cmd = cmd_vel_des_
  tau_cmd = cmd_torque_ff_

定点数编码:
  tor_des_q8  = tau_cmd × 256
  spd_des_q7  = vel_cmd × gear_ratio × 256 / (2π)
  pos_des_q15 = pos_cmd × gear_ratio × 32768 / (2π)
  k_pos_q15   = cmd_kp × 1280
  k_spd_q15   = cmd_kd × 1280

串口发送: 17 字节 → 等待 10ms → 接收 16 字节反馈
```

### 3.7 反馈: motor_control_node → delta_arm_node

```
motor_control_node publishStates():
    UnitreeGO8010State {
        joint_name: "arm_motor_1"
        position:  原始物理值 [rad, 输出轴]   ← 无 offset/direction 处理
        velocity:  原始物理值 [rad/s]
        torque:    原始物理值 [N.m]
        temperature: 温度 (°C)
        online:    心跳检测
    }

delta_arm_node controlLoop():
    反向变换: theoretical = fb.position × direction - offset[i]
    取所有在线电机平均角度 → forwardKinematics → 当前高度
```

## 4. 电机映射表

| 电机 | 名称 | 串口设备 | ID | Offset (rad) | 等效度数 |
|------|------|---------|:--:|:----------:|:------:|
| arm_motor_1 | Delta 臂电机 1 | /dev/ttyUSB0 | 0 | -0.2688 | -15.4° |
| arm_motor_2 | Delta 臂电机 2 | /dev/ttyUSB1 | 0 | -0.0297 | -1.7° |
| arm_motor_3 | Delta 臂电机 3 | /dev/ttyUSB2 | 0 | -0.0367 | -2.1° |

共享参数: gear_ratio=6.33, direction=1, kp=0.50, kd=0.04, baudrate=4000000

## 5. 状态机

```
         armCommandCallback
              │
    ┌─────────┼─────────────┐
    ▼         ▼             ▼
  IDLE     RUNNING       HOMING
  (刹车)   (高度控制)    (回零)
    │         │             │
    │   sendMotorCommands   │
    │   (FOC: pos+vel+kp+kd)│
    │         │             │
    ▼         ▼             ▼
  sendBrake  每周期发送    velocity_scale =
  Commands   3条FOC命令    homing_vel / max_vel
```

- **IDLE → RUNNING**: 收到 MODE_HEIGHT 命令
- **IDLE → HOMING**: 收到 MODE_HOME 命令
- **任意 → IDLE**: 收到 MODE_IDLE 命令 (立即发刹车)
- **ERROR**: 持续发刹车

## 6. Topic 一览

| Topic | 方向 | 消息类型 | 频率 | 内容 |
|-------|------|---------|------|------|
| `/delta_arm_command` | 遥控器 → delta_arm | DeltaArmCommand | 不定 | mode, height, vel_scale |
| `unitree_go8010_command` | delta_arm → motor | UnitreeGO8010Command | 200Hz × 3 | FOC 全参数（物理空间） |
| `unitree_go8010_states` | motor → delta_arm | UnitreeGO8010State | 200Hz × 3 | pos/vel/torque/temp（原始值） |
| `/delta_arm_state` | delta_arm → 外部 | DeltaArmState | 200Hz | height/angle + 代表电机状态 |

## 7. 与底盘控制模式对比

| 方面 | Delta 臂 (Unitree GO8010) | 底盘 (DJI GM6020+GM3508) |
|------|--------------------------|--------------------------|
| 通信接口 | RS-485 串口 (4Mbps) | CAN 总线 (921600) |
| 电机数 | 3 (同角度) | 8 (4 转向 + 4 驱动) |
| 控制模式 | FOC 直通 — 上层提供全部参数 | PID 闭环 — 上层只给目标值 |
| kp/kd 归属 | delta_arm.yaml 独占 | pid_params.yaml 独占 |
| offset 归属 | delta_arm.yaml 独占 | chassis_params.yaml 独占 |
| direction 归属 | delta_arm.yaml 独占 | chassis_params.yaml 独占 |
| 参数交叉 | **无** — 完全分离 | **无** — 完全分离 |

## 8. 调参指南

### 改 kp / kd

**只改 `delta_arm.yaml → motor.kp / motor.kd`**。

```yaml
# delta_arm.yaml
motor:
  kp: 0.50    # 关节刚度 — 增大: 更硬更精确, 减小: 更软更柔顺
  kd: 0.04    # 关节阻尼 — 增大: 抑制振荡, 减小: 响应更快
```

推荐调参步骤:
1. kp 从 0.1 起步，逐步增大到位置跟踪满意
2. 出现振荡时增大 kd
3. kp 和 kd 每次调整幅度不超过 ×2

### 改安装补偿 (offset)

**只改 `delta_arm.yaml → motor_offsets_rad`** (弧度)。

```yaml
motor_offsets_rad:
  - -0.2688   # arm_motor_1 (-15.4°)
  - -0.0297   # arm_motor_2 (-1.7°)
  - -0.0367   # arm_motor_3 (-2.1°)
```

### 改运动范围

改 `delta_arm.yaml → limits` 的 height_min/max 和 joint_angle_min/max。
节点启动时会自动计算几何可达范围并裁剪。
