# motor_offsets_rad 详解

> 基于 GO-M8010-6 电机使用手册 v1.2 及项目代码

## 1. GO-M8010-6 编码器特性（手册摘要）

| 特性 | 说明 |
|---|---|
| 编码器类型 | 单圈绝对位置编码器 |
| 单圈范围 | `[0, 2π)` rad（转子侧） |
| 多圈累加 | 运行时可累加，**掉电丢失圈数**（只保留单圈余数） |
| 减速比 | **6.33**（转子侧 → 输出端） |
| 通信侧 | 所有协议参数均为**转子侧**值，非输出端 |

关键结论：每次上电后编码器从当前位置开始计数，物理零位取决于安装和上电时刻的机械位置。

## 2. 混合控制公式（手册第4节）

电机内部 PD 控制器输出力矩：

```
τ = τ_ff + k_p × (p_des − p) + k_d × (ω_des − ω)
```

- `p_des` 和 `p` 都是**转子侧**弧度（多圈累加）
- 输出端位置 = 转子侧位置 / 6.33

## 3. motor_offsets_rad 的作用

### 问题
每个电机安装后，其编码器零位（物理0°）不一定对齐运动学要求的零位。需要一个 offset 来补偿。

### 公式定义
```
物理角 = 运动学角 + offset    （乘以 direction 之前）
```

### 代码对应

**发送命令**（`delta_arm_node.cpp:425`）：
```cpp
cmd.position_target = (kinematic_rad + offset) * config_.direction;
```

**读取反馈**（`delta_arm_node.cpp:286`）：
```cpp
double theoretical_rad = fb.position * config_.direction - offset;
```

### 配置位置
唯一来源：`src/motor_control_ros2/config/delta_arm.yaml`

```yaml
motor_offsets_rad:
  - -0.2688   # arm_motor_1 (-15.4°)
  - -0.0297   # arm_motor_2 (-1.7°)
  - -0.0367   # arm_motor_3 (-2.1°)
```

## 4. 负数的含义

### 举例：offset = -0.2688 rad (-15.4°)

| 场景 | 计算 | 结果 |
|---|---|---|
| 运动学目标 = 0° | 物理目标 = 0 + (-0.2688) | **-0.2688 rad** |
| 物理反馈 = 0 rad | 运动学角 = 0 - (-0.2688) | **+0.2688 rad (+15.4°)** |

#### 物理含义

```
offset < 0（负数）
  → 电机物理零位在运动学零位的【正方向】上
  → 电机装偏了，零位偏正
  → 需要负 offset 把目标往回拉

offset > 0（正数）
  → 电机物理零位在运动学零位的【负方向】上
  → 电机装偏了，零位偏负
  → 需要正 offset 把目标往正推

offset = 0
  → 物理零位与运动学零位完全对齐
  → 无需补偿
```

### 简单记忆

> **offset 的符号 = 运动学零位看物理零位的方向取反**
>
> 物理零位偏正 → offset 为负；物理零位偏负 → offset 为正。

## 5. 标定流程

1. 将 offset 全部设为 0
2. 上电，让电机进入 FOC 模式
3. 手动转动电机输出轴到运动学零位（delta臂全展开水平位）
4. 读取此时电机反馈的物理角度值 `p_read`
5. `offset = p_read × direction`（因为 `物理角 = 运动学角(0) + offset`）
6. 填入 `delta_arm.yaml` 的 `motor_offsets_rad`

## 6. 注意事项

- offset 单位是**弧度**（rad），不是角度（°）
- offset 是**输出端**弧度，不是转子侧（代码内部会处理减速比换算）
- 掉电重启后，若电机在断电期间被挪动了位置，offset 可能需要重新标定
- `direction` 参数与 offset 独立，direction 解决正反转方向，offset 解决零位对齐
