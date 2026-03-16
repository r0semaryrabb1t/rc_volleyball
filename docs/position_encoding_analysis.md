# GO-M8010-6 位置编码分析报告

## 现象描述

用户观察到电机监控显示的位置数据行为如下：

| 状态 | arm_motor_1 | arm_motor_2 | arm_motor_3 |
|------|-------------|-------------|-------------|
| 上电（接近最低点） | +0.585 rad | +0.942 rad | +0.941 rad |
| 最低点 | -1.400 rad | -1.044 rad | -1.044 rad |
| 最高点 | +2.474 rad | +2.765 rad | +2.581 rad |

用户疑问：**从最高点开始下降时位置值变小，到最低点变为负数，是否是角度转换错误？**

---

## 数据流溯源

```
电机硬件
  └─ 串口反馈包（16字节）
       └─ parseFeedback()  [unitree_motor_native.cpp:128]
            ├─ pos_q15（int32_t，原始定点数）
            ├─ pos_deg = pos_q15 * 360.0 / gear_ratio_ / 32768.0
            └─ position_ = pos_deg * π / 180.0
  └─ publishStates()  [motor_control_node.cpp:584]
       └─ msg.position = motor->getOutputPosition()
            └─ encoder_on_output_=true → 直接返回 position_
  └─ 监控节点显示
```

---

## 根因分析

### 问题一：这不是 Bug，而是设计上的误解

位置值呈现「开机为零点、向下为负、向上为正」的行为，是 **GO-M8010-6 的正常工作方式**。

该电机的位置编码器是**上电归零的相对编码器**（incremental/relative encoder），不是真正的「单圈绝对值编码器」：

- 上电时，电机将当前位置记为 0
- 向一个方向转动 → 位置增大（正值）
- 向另一个方向转动 → 位置减小（负值）
- 跨越零点后继续转动 → 值持续增减，不会跳变

因此，监控显示的「从最高点下降时值变小、到最低点变负」是**完全正确的物理行为**。

### 问题二（潜在 Bug）：gear_ratio 可能被多除一次

当前代码（`unitree_motor_native.cpp:167`）：
```cpp
double pos_deg = pos_q15 * 360.0 / gear_ratio_ / 32768.0;
```

这里对 `gear_ratio_（6.33）` 进行了除法。**需要确认 GO-M8010-6 协议返回的 `pos_q15` 是电机轴还是输出轴的位置。**

**实测验证方法**：
1. 手动将输出轴转动整整一圈（360°）
2. 观察 position 的变化量
   - 如果变化 ≈ 2π rad（6.28）→ 当前公式正确（pos_q15 是电机轴，除以 gear_ratio 后得到输出轴）
   - 如果变化 ≈ 2π × 6.33 ≈ 39.7 rad → gear_ratio 被多除了，应删除该除法

根据宇树 GO-M8010-6 的官方协议文档和社区资料，`pos_q15` **是输出轴角度**，协议层已内部处理减速比，因此当前代码可能多除了一次 gear_ratio。

---

## 结论

| 问题 | 结论 |
|------|------|
| 位置值变负是否异常？ | **否**，是相对编码器上电归零的正常行为 |
| 角度转换公式是否正确？ | **待验证**，gear_ratio 可能被多除一次 |
| 是否需要 homing？ | **是**，如需绝对位置参考，需增加回零逻辑 |

---

## 建议修复方案

### 方案 A：如果 pos_q15 是输出轴（需实测确认）

修改 `unitree_motor_native.cpp:167`：
```cpp
// 旧（错误）
double pos_deg = pos_q15 * 360.0 / gear_ratio_ / 32768.0;

// 新（正确）
double pos_deg = pos_q15 * 360.0 / 32768.0;
```

同时将 `MotorBase` 构造参数 `encoder_on_output` 保持为 `true`。

### 方案 B：如果 pos_q15 是电机轴（当前假设）

当前公式正确，无需修改。

### Homing 建议（长期）

为 Delta 机械臂增加上电 homing 流程：
1. 上电后发送零力矩命令，记录当前位置为 `position_offset_`
2. 后续所有位置 = `position_ - position_offset_`
3. 或者增加限位开关，触发后重置零点

---

## 实测验证步骤

```bash
# 1. 启动控制节点
ros2 run motor_control_ros2 motor_control_node

# 2. 另一终端监控位置
ros2 topic echo /unitree_go8010_states

# 3. 手动将输出轴从某位置转动整整一圈
# 4. 记录 position 变化量，与预期值对比：
#    - 正确（输出轴弧度）：变化 ≈ 6.28 rad
#    - gear_ratio 多除：变化 ≈ 39.7 rad
```
