# motor_monitor_node 宇树电机相关漏洞分析

## 日期: 2026-03-14
## 分析范围: motor_monitor_node.cpp + motor_control_node.cpp 发布端

---

## 漏洞清单

### BUG-1（严重）: `MotorStats::was_online` 未初始化 — 未定义行为

**位置**: `motor_monitor_node.cpp:96`

```cpp
struct MotorStats {
    // ...
    bool was_online;  // ← 未初始化！
};
```

**问题**: 新电机首次上报时，`was_online` 值不确定（可能是 `true` 或 `false`）。
`checkOnlineStatus()` 中的初始化守卫（第151行）永远不会触发，因为 `updateMotorStats()` 先执行，已经将 `last_update` 设为非零值。

**后果**:
- 若 `was_online` 恰好为 `true`，且电机在线 → 丢失"上线"日志
- 若 `was_online` 恰好为 `true`，且电机离线 → 虚假"离线"日志

**修复**: 在 struct 中初始化 `bool was_online = false;`

---

### BUG-2（中等）: motor_control_node 发布 GO8010State 时遗漏 3 个字段

**位置**: `motor_control_node.cpp:582-594`

| 字段 | msg 定义 | 是否填充 | 默认值 |
|------|----------|----------|--------|
| `mode` | `uint8` | **否** | 0 (BRAKE) |
| `error` | `int8` | **否** | 0 |
| `acceleration` | `int16` | **否** | 0 |

**后果**:
- 监控端无法看到电机当前控制模式（FOC/BRAKE/CALIBRATE）
- 电机报错时 error code 丢失，无法诊断
- 加速度数据永远为 0

**修复**: 在发布端补充这三个字段。

---

### BUG-3（低）: monitor_node 不显示宇树电机关键诊断信息

**位置**: `motor_monitor_node.cpp:305-319`

宇树电机表格缺少以下列：
- `motor_id` — 在多串口同ID场景（arm_motor_1/2/3 均为 ID=0）下必须显示
- `error` — 电机错误码
- `mode` — 当前控制模式

**对比**: DJI 电机表格有 `model` 列，达妙有 `力矩` 列，但宇树缺少关键的 `ID`/`Error` 列。

---

### BUG-4（低）: unitree A1 订阅是死代码

**位置**: `motor_monitor_node.cpp:60-63`

```cpp
unitree_sub_ = this->create_subscription<...UnitreeMotorState>(
    "unitree_motor_states", 10, ...);
```

motor_control_node 中 `unitree_state_pub_` 从未发布过数据（SDK 版已禁用），
这个订阅永远不会触发。`unitree_states_` map 永远为空。

**后果**: 无功能影响，但增加代码阅读负担。建议保留但加注释，以备将来启用 A1 支持。

---

### BUG-5（低）: 空行和格式问题

**位置**: `motor_monitor_node.cpp:309-311`

```cpp
        std::string status_text = is_online ? "在线" : "离线";



        oss << "│ " << ...
```

多余空行，风格不一致。

---

## 修复方案

### 修复 BUG-1: MotorStats 初始化

```cpp
struct MotorStats {
    rclcpp::Time last_update;
    double actual_hz = 0.0;
    int msg_count = 0;
    rclcpp::Time last_stat_time;
    bool was_online = false;  // ← 添加默认值
};
```

### 修复 BUG-2: 补充 motor_control_node 发布字段

在 `publishStates()` 的宇树电机循环中增加：
```cpp
msg.mode = 0;  // TODO: 从 motor 获取当前模式
msg.error = motor->getErrorCode();
// acceleration 暂不可用（协议中未直接提供）
```

### 修复 BUG-3: monitor_node 增加诊断列

在宇树电机表格中增加 `ID` 和 `错误码` 列。

### 修复 BUG-5: 清理空行
