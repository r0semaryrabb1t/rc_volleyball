# CRC-CCITT 放置位置决策

## 日期: 2026-03-14
## 问题: CRC 计算应放在 `unitree_motor_native.cpp`（协议层）还是 `serial_interface.cpp`（硬件层）？

---

## 结论: 保留在协议层 (`unitree_motor_native.cpp`)

### 核心论据

#### 1. CRC 是设备协议的一部分，不是传输协议

| 层次 | 职责 | CRC 属于？ |
|------|------|-----------|
| **设备协议层** (`UnitreeMotorNative`) | GO-M8010-6 定点数编解码、帧头 0xFE/0xEE、CRC-CCITT | **是** |
| **硬件传输层** (`SerialInterface`) | 串口打开/关闭、字节收发、波特率设置 | 否 |

CRC-CCITT 是 GO-M8010-6 电机协议规范的一部分（17字节命令包的最后2字节），与 RS485 串口传输层无关。

#### 2. 与 CAN 侧分层的对比验证

```
CAN 侧:
  DJIMotor (协议层)     → 编解码 CAN 数据帧内容
  CANInterface (硬件层)  → 收发 CAN 帧，不关心帧内容

串口侧 (当前):
  UnitreeMotorNative (协议层)  → 编解码 17字节命令包 + CRC
  SerialInterface (硬件层)     → 收发字节流，不关心内容
```

两侧完全对称。CAN 侧的 `CANInterface` 不会计算 DJI 电机的校验码，同理 `SerialInterface` 也不应该计算 GO-M8010-6 的 CRC。

#### 3. RS485 是透明字节传输

USB-CAN 适配器有自己的 0x55/0xAA 帧封装协议（适配器级别），但 RS485 串口是透明的字节通道，没有额外的传输协议层。因此 CRC 只可能属于设备协议层。

#### 4. 可复用性

如果未来有其他串口设备使用不同的校验算法（如 Modbus CRC-16），`SerialInterface` 不应绑定任何特定的校验逻辑。CRC 实现跟随设备协议类走。

### 替代方案（已否决）

**方案 B**: 将 CRC 移到 `serial_interface.cpp`
- 缺点: 硬件层承担了协议层职责，破坏分层
- 缺点: 不同设备可能使用不同校验算法，硬件层不应区分设备类型
- 缺点: 与 CAN 侧设计不对称

### 未来优化（可选）

`calcCrcCcitt()` + `CRC_TABLE` 可以提取到独立的 `crc_utils.hpp` 工具头文件，供多个协议类复用。但当前只有一个使用者，暂无必要。

---

## 相关文件

- `src/motor_control_ros2/src/unitree_motor_native.cpp` — CRC 计算所在位置
- `src/motor_control_ros2/include/motor_control_ros2/unitree_motor_native.hpp` — `calcCrcCcitt()` 声明
- `src/motor_control_ros2/src/hardware/serial_interface.cpp` — 纯硬件 I/O，不涉及 CRC
