# 串口电机分层重构计划

## 日期: 2026-03-14
## 状态: 已通过专家评审

---

## 问题分析

### 当前分层对比

| 层次 | CAN侧（良好） | 串口侧（混乱） |
|------|---------------|----------------|
| 硬件层 | `CANInterface` — 纯I/O | `SerialInterface` — 纯I/O ✓ |
| 协议层 | `DJIMotor` — 纯编解码，不持有硬件引用 | `UnitreeMotorNative` — **混合I/O和协议** ✗ |
| 编排层 | Node: 取控制帧 → CAN发送 → 回调解析 | Node: 直接调 `motor->sendRecv()` |

### 隐藏 Bug（name hiding）

`UnitreeMotorNative` 声明了与 `MotorBase` 同名的 `online_` 和 `last_feedback_time_ns_`，
形成 **name hiding** 而非 **override**。通过 `MotorBase*`（如 `motors_` map 遍历）调用
`checkHeartbeat()` 时，读取的是基类从未更新的字段 → **宇树电机在 motors_ 遍历中永远判定为离线**。

---

## 重构方案

### 1. SerialInterface — 新增请求-响应方法

```cpp
// serial_interface.hpp
bool sendRecvAccumulate(const uint8_t* send_data, size_t send_len,
                        uint8_t* recv_buffer, size_t expected_len,
                        int wait_ms = 10, int timeout_ms = 20);
```

将 `UnitreeMotorNative::sendRecv()` 中的 send→sleep→accumulate-receive 时序逻辑移入此方法。

### 2. UnitreeMotorNative — 纯协议层

**删除：**
- `serial_` 成员（`shared_ptr<SerialInterface>`）
- `setSerialInterface()` 方法
- `sendRecv()` 方法
- `rx_buffer_[16]` 成员
- `send_debug_count_` 成员
- `online_` 成员（消除 name hiding，使用基类）
- `last_feedback_time_ns_` 成员（消除 name hiding，使用基类）
- `isOnline()`, `checkHeartbeat()`, `getOutputPosition()`, `getOutputVelocity()`,
  `getOutputTorque()`, `getTemperature()` 方法（消除 name hiding，使用基类）

**新增：**
- `size_t getCommandPacket(uint8_t* buffer)` — 构建17字节命令包
- `bool parseFeedback(const uint8_t* data, size_t len)` — 解析反馈，同步更新基类字段

**保留：**
- `updateFeedback()` 空实现（CAN回调兼容）
- `tx_buffer_[17]` 内部工作区
- `buildCommandPacket()` 内部方法
- 所有控制命令方法（setFOCCommand 等）

### 3. motor_control_node — 编排层

```cpp
void writeUnitreeNativeMotors() {
  for (auto& motor : unitree_native_motors_) {
    auto serial = serial_network_->getInterface(motor->getInterfaceName());
    if (!serial || !serial->isOpen()) continue;

    // 协议层：构建命令
    uint8_t cmd[17];
    motor->getCommandPacket(cmd);

    // 硬件层：发送接收
    uint8_t resp[16];
    if (serial->sendRecvAccumulate(cmd, 17, resp, 16, 10, 20)) {
      // 协议层：解析反馈
      motor->parseFeedback(resp, 16);
    }
  }
}
```

`addUnitreeNativeMotorFromConfig()` 不再传递 `serial` 参数。

### 4. 删除 serial_port_manager.hpp

依赖旧 SDK `serialPort/SerialPort.h`，全项目无引用。

---

## 线程安全评估

- 单线程 executor 下天然安全
- `getCommandPacket()` 和 `parseFeedback()` 内部各自持有 `mutex_`
- `SerialNetwork::getInterface()` 有 `mutex_` 保护
- 无新增线程安全风险

## 兼容性评估

- `OPS9EncoderNode` — 独立 `SerialInterface`，仅用 `receive()`，无影响
- `DeltaArmNode` — 通过 ROS2 topic 通信，无影响
- CAN 侧电机 — 完全不涉及

---

## 修改文件清单

| 文件 | 操作 | 说明 |
|------|------|------|
| `hardware/serial_interface.hpp` | 修改 | 新增 `sendRecvAccumulate()` 声明 |
| `hardware/serial_interface.cpp` | 修改 | 实现 `sendRecvAccumulate()` |
| `unitree_motor_native.hpp` | 修改 | 删除I/O成员，新增协议接口，消除name hiding |
| `unitree_motor_native.cpp` | 修改 | 删除sendRecv()，实现新接口，同步基类字段 |
| `motor_control_node.cpp` | 修改 | 重写编排层 |
| `hardware/serial_port_manager.hpp` | 删除 | 无引用的旧SDK遗留 |
