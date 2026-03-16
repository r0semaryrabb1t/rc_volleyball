# delta_arm_manager 节点数据流分析

## 1. 系统拓扑

```
                    /delta_arm/target (ArmTarget)
                        │  target_angles[3], execute
                        ▼
             ┌──────────────────────────────┐
             │       delta_arm_manager      │  配置: arm_config.yaml
             │                              │  ├─ control_frequency (200 Hz)
             │  状态机:                     │  ├─ initialization.downward_torque
             │    INIT → SOFT_LANDING       │  ├─ initialization.landing_timeout
             │    SOFT_LANDING → READY      │  ├─ initialization.landing_angle_threshold
             │    READY → EXECUTE → READY   │  ├─ initialization.landing_stable_duration
             │                              │  ├─ pd.kp / pd.kd
             │  梯形速度规划 (200 Hz)       │  ├─ motion_profile.max_velocity
             │    pos_next = pos_now        │  ├─ position_tolerance
             │      + clamp(err, ±v*dt)     │  └─ motors[].name / motors[].id
             └──────┬───────────────────────┘
                    │                              ▲
   publish (×3 msg) │                              │ subscribe
                    ▼                              │
        unitree_go8010_command           unitree_go8010_states
        (UnitreeGO8010Command)           (UnitreeGO8010State)
                    │                              ▲
                    ▼                              │
             ┌──────────────────────────────┐      │
             │      motor_control_node      │  配置: motors.yaml
             │                              │  └─ gear_ratio (纯硬件参数)
             │   setFOCCommand(             │
             │     pos, vel, kp, kd, τ)    │  纯透传，不做坐标变换
             │   原生串口 SendRecv          │
             └──────────────────────────────┘
                    │
              RS-485 Serial (4 Mbps)
         ┌──────────┼──────────┐
    /dev/ttyUSB0  USB1      USB2
     arm_motor_1  _2        _3
     (ID 0)      (ID 1)    (ID 2)
```

**注意**: 三个电机 ID 分别为 0、1、2，靠 `motors[].id` 字段在 `arm_config.yaml` 中配置，
`motor_control_node` 按 `id` 字段路由到对应串口。

---

## 2. Topic 一览

| Topic | 方向 | 消息类型 | 频率 | 内容 |
|-------|------|---------|:----:|------|
| `/delta_arm/target` | 外部 → manager | `ArmTarget` | 不定 | `target_angles[3]`(rad), `execute` |
| `unitree_go8010_command` | manager → motor | `UnitreeGO8010Command` | 200 Hz × 3 | FOC 全参数（物理空间） |
| `unitree_go8010_states` | motor → manager | `UnitreeGO8010State` | 200 Hz × 3 | pos/vel/torque/temp/online |
| `/delta_arm/ready` | manager → 外部 | `std_msgs/String` | 状态变化时 | `"READY"` |

---

## 3. 消息定义

### 3.1 ArmTarget.msg（输入）

```
std_msgs/Header header

float64[3] target_angles   # 三路电机目标角度（弧度，输出轴）
bool execute               # true: 进入 EXECUTE 状态; false: 忽略本条消息
```

### 3.2 UnitreeGO8010Command（输出，每路电机各发一条）

```
std_msgs/Header header
uint8  id               # 电机 ID (来自 arm_config.yaml motors[].id)
uint8  mode             # 1 = MODE_FOC
float64 position_target  # 期望位置 (rad，输出轴)
float64 velocity_target  # 期望速度 (rad/s，输出轴，EXECUTE 阶段为 0)
float32 torque_ff        # 前馈力矩 (Nm)
float32 kp              # 位置刚度
float32 kd              # 速度阻尼
```

- **SOFT_LANDING 阶段**：`kp=kd=0`，`torque_ff = initialization.downward_torque`（纯力矩控制）
- **READY/EXECUTE 阶段**：`torque_ff=0`，`kp/kd` 来自 `arm_config.yaml → pd`

---

## 4. 状态机

```
             节点启动
                │
                ▼
              INIT
           (首个控制周期)
                │ 立即
                ▼
          SOFT_LANDING ────────────────────────────────────────────────────┐
          施加向下力矩                                                     │ 超时
          (torque_ff = downward_torque, kp=kd=0)                          │ (> landing_timeout)
                │                                                          │
                │ 三电机绝对角度均 < landing_angle_threshold               │
                │ 持续 landing_stable_duration                             │
                │                                                          │
                ▼                                                          │
             READY ◄───────────────────────────────────────────────────────┘
          发布 /delta_arm/ready: "READY"
          保持当前位置（kp/kd PD 控制）
                │
                │ 收到 ArmTarget (execute=true)
                ▼
            EXECUTE
          梯形速度规划跟踪三路目标角度
          (pos_next = pos_now + clamp(err, ±max_velocity*dt))
                │
                │ 三路电机误差均 < position_tolerance
                ▼
             READY
          再次发布 /delta_arm/ready: "READY"
```

**状态转移规则**

| 当前状态 | 触发条件 | 目标状态 |
|---------|---------|---------|
| INIT | 第一个控制周期 | SOFT_LANDING |
| SOFT_LANDING | 三电机 `\|pos\| < threshold` 持续 `stable_duration` | READY |
| SOFT_LANDING | 经过 `landing_timeout` 秒 | READY（超时警告） |
| READY | 收到 `ArmTarget (execute=true)` | EXECUTE |
| EXECUTE | 三电机误差均 `< position_tolerance` | READY |

---

## 5. 控制循环（200 Hz）

```
每个控制周期 (dt = 1/200 s = 5 ms):

  case INIT:
    → state = SOFT_LANDING
    → 记录 init_start_time

  case SOFT_LANDING:
    ① 检查超时: elapsed = now - init_start_time
       elapsed > landing_timeout → 强制进入 READY
    ② 向三电机各发:
         UnitreeGO8010Command(id=motor_ids[i], mode=FOC,
           pos=0, vel=0, kp=0, kd=0, torque_ff=downward_torque)
    ③ 检查着陆条件: ∀i: |current_positions[i]| < landing_angle_threshold
       若满足且持续 landing_stable_duration → 进入 READY，发布 /delta_arm/ready

  case READY:
    ① 若尚未发布 ready → publishReady()
    ② 向三电机各发:
         UnitreeGO8010Command(id=motor_ids[i], mode=FOC,
           pos=current_positions[i], vel=0, torque_ff=0, kp=kp_, kd=kd_)

  case EXECUTE:
    ① max_step = max_velocity * dt
    ② for i in [0,1,2]:
         err = target_positions[i] - planned_positions[i]
         step = clamp(err, -max_step, +max_step)
         planned_positions[i] += step
         发送 UnitreeGO8010Command(pos=planned_positions[i], kp=kp_, kd=kd_)
    ③ 若 allMotorsReached() → state = READY, ready_published = false
```

---

## 6. 软着陆详解

软着陆目的：在节点启动时安全地将三路电机驱动到零位参考点（绝对角 ≈ 0 rad），
避免直接下发位置命令时因初始位置未知而产生冲击力矩。

```
施加力矩: torque_ff = downward_torque (默认 -1.0 Nm)
          kp = 0, kd = 0 → 纯力矩驱动

判定着陆:
  当三路电机绝对角度同时满足 |pos[i]| < landing_angle_threshold (默认 0.02 rad ≈ 1.1°)
  并且上述条件持续 landing_stable_duration (默认 0.5 s)

超时保护:
  若 landing_timeout (默认 5.0 s) 内未完成软着陆，强制进入 READY
  → RCLCPP_WARN 提示，不阻塞后续操作
```

---

## 7. 梯形速度规划详解

梯形规划本质是对目标角度做一阶速度限幅积分，防止阶跃目标导致电机瞬间高速运动。

```
每帧:
  max_step = max_velocity * dt       // dt = 1/control_frequency

  for i = 0, 1, 2:
    err  = target[i] - planned[i]   // 规划位置与目标的差
    step = clamp(err, -max_step, +max_step)
    planned[i] += step              // 规划位置向目标靠近，每帧最多走 max_step

    → 下发 planned[i] 作为电机位置目标

到达判定:
  |target[i] - current[i]| < position_tolerance (默认 0.05 rad)
  三路同时满足 → 状态切回 READY
```

**示例**（`max_velocity=2.0 rad/s`，`dt=0.005 s`）：

| 帧号 | max_step | 误差 1.0 rad 时 planned 增量 | 运动时间估算 |
|:----:|:-------:|:----:|:----:|
| 1 | 0.01 rad | 0.01 rad | 1.0 / 2.0 = 0.5 s（匀速段） |

---

## 8. 配置文件参数参考（`arm_config.yaml`）

| 路径 | 类型 | 默认值 | 说明 |
|------|------|:------:|------|
| `control_frequency` | double | 200.0 | 控制定时器频率（Hz） |
| `initialization.downward_torque` | double | -1.0 | 软着陆施加力矩（Nm，负值向下） |
| `initialization.landing_timeout` | double | 5.0 | 软着陆最长允许时间（s） |
| `initialization.landing_angle_threshold` | double | 0.02 | 着陆判定角度阈值（rad） |
| `initialization.landing_stable_duration` | double | 0.5 | 着陆稳定持续时间（s） |
| `pd.kp` | double | 0.50 | READY/EXECUTE 阶段位置刚度 |
| `pd.kd` | double | 0.10 | READY/EXECUTE 阶段速度阻尼 |
| `motion_profile.max_velocity` | double | 2.0 | 梯形规划最大速度（rad/s） |
| `position_tolerance` | double | 0.05 | 到达判定容差（rad） |
| `safety.max_height` | double | 0.3 | 参考最大高度（m，仅记录，不限位） |
| `motors[i].name` | string | `"arm_motor_i"` | 对应 `UnitreeGO8010State.joint_name` |
| `motors[i].id` | uint8 | 0/1/2 | 发往 `UnitreeGO8010Command.id` |

完整示例：

```yaml
control_frequency: 200.0

initialization:
  downward_torque: -1.0
  landing_timeout: 5.0
  landing_angle_threshold: 0.02
  landing_stable_duration: 0.5

pd:
  kp: 0.50
  kd: 0.10

motion_profile:
  max_velocity: 2.0

position_tolerance: 0.05

safety:
  max_height: 0.3

motors:
  - name: "arm_motor_1"
    id: 0
  - name: "arm_motor_2"
    id: 1
  - name: "arm_motor_3"
    id: 2
```

---

## 9. 参数归属（与底盘模式一致，无交叉）

| 参数 | 唯一来源 | 归属节点 |
|------|---------|---------|
| kp / kd | `arm_config.yaml` | `delta_arm_manager` |
| max_velocity | `arm_config.yaml` | `delta_arm_manager` |
| downward_torque | `arm_config.yaml` | `delta_arm_manager` |
| motor name / id 映射 | `arm_config.yaml` | `delta_arm_manager` |
| gear_ratio | `motors.yaml` | `motor_control_node` |
| 串口设备路径 | `motors.yaml` | `motor_control_node` |

---

## 10. 调参指南

### 软着陆调参

```yaml
# 机械臂较重 / 摩擦较大 → 增大向下力矩幅值
initialization:
  downward_torque: -2.0   # 原 -1.0，酌情调大

# 着陆后仍有抖动 → 延长稳定时间
  landing_stable_duration: 1.0   # 原 0.5 s

# 到底角度非常接近 0 → 收紧阈值
  landing_angle_threshold: 0.01  # 原 0.02 rad
```

### EXECUTE 位置跟踪调参

```yaml
pd:
  kp: 0.50    # 增大 → 跟踪更硬更精确；过大 → 振荡
  kd: 0.10    # 增大 → 抑制振荡；过大 → 响应变慢

motion_profile:
  max_velocity: 1.0   # 减小 → 运动更平稳；增大 → 速度更快
```

推荐调参步骤：
1. 先令 `max_velocity` 较小（0.5 rad/s），确认运动方向和范围正确。
2. `kp` 从 0.1 起步，逐步增大直到位置跟踪满意。
3. 出现振荡时增大 `kd`；`kp` 和 `kd` 每次调整幅度不超过 × 2。
4. 调好刚度后再逐步提高 `max_velocity`。

### 到达判定容差

```yaml
position_tolerance: 0.05   # rad ≈ 2.9°
# 过大 → 未到位就切回 READY
# 过小 → 电机轻微抖动时无法切回 READY
```

---

## 11. 常见问题

| 现象 | 可能原因 | 排查方法 |
|------|---------|---------|
| 节点卡在 SOFT_LANDING | 力矩太小，电机未能回到 0 位 | 加大 `\|downward_torque\|` 或延长 `landing_timeout` |
| EXECUTE 后未切回 READY | `position_tolerance` 太小或 kp 太弱 | 增大 tolerance 或 kp |
| 电机命令发出但不响应 | `motors[i].id` 与硬件 ID 不匹配 | 检查 `arm_config.yaml → motors[i].id` 与实际电机拨码 |
| 收到 ArmTarget 但不执行 | 当前状态非 READY | 等待 `/delta_arm/ready` 话题出现 `"READY"` 消息后再发送目标 |
| 运动时振荡 | kd 偏小或 max_velocity 偏大 | 增大 kd，减小 max_velocity |

---

## 12. 与旧版 delta_arm_node 对比

| 方面 | 旧版 `delta_arm_node` | 新版 `delta_arm_manager` |
|------|----------------------|--------------------------|
| 控制接口 | `DeltaArmCommand (mode, height, vel_scale)` | `ArmTarget (target_angles[3], execute)` |
| 初始化 | 无自动零位流程 | 自动软着陆 (SOFT_LANDING) |
| 速度规划 | 由外部 `velocity_scale` 决定 | 内建梯形规划 (max_velocity) |
| 就绪通知 | 无 | 发布 `/delta_arm/ready: "READY"` |
| 逆运动学 | 有（高度 → 角度） | 无（直接输入三路目标角度） |
| 配置文件 | `delta_arm.yaml` | `arm_config.yaml` |
