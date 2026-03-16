# 最终方案 Plan

## 问题一：串口热插拔修复

### 状态：✅ 已完成

### 修改文件
- `src/motor_control_ros2/src/hardware/serial_interface.cpp`

### 修改内容
在 `sendRecvAccumulate()` 和 `sendRecv()` 的 `send()` 前增加：
```cpp
tcflush(fd_, TCIFLUSH);  // 清空内核 RX 缓冲区
```

### 原理
RS485 半双工请求-响应协议下，发送前不可能有有效的待接收数据。
`tcflush` 清除的只可能是：断电期间的噪声/残留/不完整帧。

### 验证
1. `colcon build --packages-select motor_control_ros2`
2. 启动 motor_control_node
3. 确认 3 台电机在线
4. **不关节点**，断电所有电机
5. 重新上电
6. 观察 3 台电机是否全部恢复在线

---

## 问题二：Delta 机械臂无法到达目标位置

### 状态：⚠️ 需要用户操作（校准 + 调参）

### 根因
1. **kp=0.02 极度偏小** — 产生的力矩（~0.01 Nm）连静摩擦都无法克服
2. **arm_motor_2 offset=0.063 可疑** — 与其他电机（0.639, 0.872）差异过大

### 建议修改：delta_arm.yaml

```yaml
motor:
  kp: 0.50    # 原 0.02，提高 25 倍使电机能到达目标
  kd: 0.10    # 原 0.08，适当提高防止振荡
```

### Offset 重新校准步骤
1. 手动将机械臂放到**已知高度** h（建议用最低位 h_min=0.171m）
2. 启动 motor_control_node，从 monitor 读取 3 个电机的 position (rad)
3. 计算理论运动学角：`θ_rad = inverseKinematics(h) × π / 180`
4. 计算每个电机的 offset：`offset = position × direction − θ_rad`
5. 更新 delta_arm.yaml 的 `motor_offsets_rad`

### 注释修正
```yaml
motor_offsets_rad:
  - 0.639  # arm_motor_1 (36.6°) — 原注释 -15.4° 有误
  - ???    # arm_motor_2 — 需重新校准
  - 0.872  # arm_motor_3 (50.0°) — 原注释 -2.1° 有误
```

### 安全注意事项
- 调参前确保机械臂周围无障碍物
- kp 不要一次性跳到很大值，从 0.5 开始观察
- 如果振荡，先增大 kd 再增大 kp

---

## 会议纪要索引
- [问题一会议纪要](meeting_serial_hotplug.md)
- [问题二会议纪要](meeting_delta_arm_position.md)
- [高度 Bug 原始分析](delta_arm_height_bug_analysis.md)
不
---

## 问题三：单圈绝对编码器（内圈）与机械臂解算（外圈）冲突

### 状态：✅ 已完成（代码与配置均已落地）

### 会议摘要（主Agent + 3个Sub-Agent）
1. **开发子代理（激进方案）**：直接在现有逆解上继续调 `offset`，让高度看起来匹配。
2. **测试子代理（否决）**：否决。原因是上电初值随机落在单圈不同相位，单调性不稳定，单靠 `offset` 无法保证每次一致。
3. **架构子代理（折中方案）**：增加“上电两阶段引导流程”，先走绝对位置定姿，再启用机械臂外圈解算。
4. **最终共识**：三方一致同意采用“**start 位 → ready 位 → 解算控制**”流程，并强制每次上电执行。

### 已实现方案
- 在 `delta_arm.yaml` 增加 `startup_sequence`：
  - `start_positions_rad`: `[0.669, 0.846, 0.886]`
  - `ready_positions_rad`: `[1.261, 1.690, 1.689]`
  - `enabled: true`
- `delta_arm_node` 新增引导状态机：
  - `MOVE_TO_START` -> `MOVE_TO_READY` -> `DONE`
- 引导未完成前，拒绝高度/回零命令（仅允许空闲）。
- 引导完成后，才进入正常逆解与高度控制。

### 修改文件
- `src/motor_control_ros2/include/motor_control_ros2/delta_arm_node.hpp`
- `src/motor_control_ros2/src/delta_arm_node.cpp`
- `src/motor_control_ros2/config/delta_arm.yaml`

### 验证步骤
1. 上电后启动节点，观察日志：
   - 先到 start 位
   - 再到 ready 位
   - 最后提示“进入机械臂解算控制”
2. 引导阶段发送高度命令应被拒绝（日志告警）。
3. 引导完成后发送高度命令，应正常动作并稳定收敛。

---

## 问题四：`motor_offsets_rad` 是否应替换成 `ready_positions_rad`

### 状态：✅ 已定稿（不替换）

### 会议摘要（主Agent + 3个Sub-Agent）
1. **开发子代理（激进）**：建议直接把 `motor_offsets_rad` 改成 ready 三个值，简化配置。
2. **测试子代理（否决）**：否决。ready 位是“启动过程的目标点”，不是“坐标系映射常量”；混用会导致解算漂移。
3. **架构子代理（调和）**：保留“双参数体系”并明确职责边界：
  - `startup_sequence.ready_positions_rad`：一次性引导位
  - `motor_offsets_rad`：长期映射参数（物理角到运动学角）
4. **最终共识**：三方一致同意“不替换”，并建议仅在标定流程后更新 `motor_offsets_rad`。

### 最终方案
- `motor_offsets_rad` 保持“映射参数”语义，不改成 ready 值。
- `ready_positions_rad` 仅用于上电引导状态机，不参与长期映射。
- 若需重标定 offset：
  1. 完成上电引导（到 ready）
  2. 在已知高度/角度姿态采样物理角
  3. 用 `offset = physical_rad * direction - kinematic_rad` 逐电机计算
  4. 仅更新 `motor_offsets_rad`

---

## 问题五：机械臂模块重写为 `delta_arm_manager`

### 状态：✅ 已完成

### 会议摘要（主Agent + 3个Sub-Agent）
1. **开发子代理（激进）**：直接删除旧 Delta 与手柄逻辑，一次性切到新状态机。
2. **测试子代理（否决）**：否决“一步到位”上线。要求保留最小可回滚路径，并先完成消息与配置兼容检查。
3. **架构子代理（折中）**：同意“删除旧逻辑”，但分 4 步实施：
  - 步骤A：先落地新 msg 与新配置
  - 步骤B：实现 `delta_arm_manager` 状态机（Init/SoftLanding/Ready/Execute）
  - 步骤C：移除旧 Delta 节点与手柄到机械臂发布链路
  - 步骤D：编译+联调+发布 ready 信号
4. **最终共识**：三方一致同意按折中方案执行。

### 新节点目标
- 节点名：`delta_arm_manager`
- 控制对象：3 路 GO8010
- 核心状态机：
  - `INIT`
  - `SOFT_LANDING`
  - `READY`
  - `EXECUTE`

### 新配置文件（`arm_config.yaml`）
- `initialization.downward_torque`
- `initialization.landing_timeout`
- `pd.kp`
- `pd.kd`
- `motion_profile.max_velocity`
- `safety.max_height`
- （建议补充）`control_frequency`
- （建议补充）`landing_angle_threshold`（默认 0.02 rad）
- （建议补充）`landing_stable_duration`（默认 0.5 s）

### 新消息
- `ArmTarget.msg`
  - `float64[3] target_angles`
  - `bool execute`

### 关键技术决策
1. **Soft Landing 判定**
  - 修订为“连续 0.5s 三电机角度变化量都小于 0.02rad”作为完成条件。
  - 即判定静止/落稳，不再使用绝对角度阈值。
2. **Ready 通知**
  - 发布 `std_msgs/String` 到 `/delta_arm/ready`，内容：`READY`。
3. **梯形速度规划**
  - 采用离散帧积分：`pos_next = pos_now + clamp(err, ±max_velocity*dt)`。
  - 每帧同步下发 3 路命令。
4. **同步控制频率**
  - 控制定时器固定频率执行（默认 200Hz，可配置）。
5. **解耦坐标设计**
  - 初始化解耦后，三电机控制坐标起点强制置为 `0 rad`（不读取真实绝对角作为控制零点）。
  - 高度变化由相对增量驱动：外圈只计算 `Δrad`。
  - 三个电机每一控制周期的目标增量必须相同：`Δrad_1 = Δrad_2 = Δrad_3`。
  - 三电机目标统一表达为：`target_i = target_i_prev + Δrad`。

### 删除/替换范围（实施时执行）
- 删除旧 Delta 机械臂节点及其旧消息依赖。
- 删除手柄到机械臂的发布逻辑（保留底盘/其它控制链）。
- 新增 `delta_arm_manager` 可执行目标并替换 CMake 注册。

### 口径已确认
- 解耦初始化弧度固定 0。
- 控制只看变化量，不看绝对角。
- 三电机变化量始终一致。
