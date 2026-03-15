# 热插拔修复会议摘要

**日期**: 2026-03-14
**议题**: 宇树电机断电重启后只有 1/3 台能恢复在线
**结论**: 三位 Sub-Agent 全票通过分层修复方案

---

## 根因

`tcflush(TCIFLUSH)` 只清空内核软件缓冲区，无法清空 USB-Serial 芯片（CH340/FTDI）的硬件 FIFO。电机上电时产生的噪声字节会在 `send()` 之后延迟搬运到内核缓冲区，导致 `receive()` 读到噪声前缀 + 有效帧混合数据，帧错位，`parseFeedback()` 失败，心跳超时，持续离线。

---

## 思想碰撞

### Sub-Agent A（嵌入式工程师）— 激进方案
主张在 `parseFeedback` 前加帧扫描：扩大缓冲区到 64 字节，搜索 `0xFD 0xEE` 帧头，跳过噪声前缀。这是工业串口标准做法。

### Sub-Agent B（测试/运维）— 质疑
反对直接在底层加帧扫描：`0xFD 0xEE` 可能出现在噪声中（概率 1/65536 误判）；`sendRecvAccumulate` 职责不应承担帧对齐；主张保守的失败重试 + 二次 flush 方案。

### Sub-Agent C（架构师）— 折中
分层处理：
- **硬件层**：`sendRecvAccumulate` 改为返回 `ssize_t`（实际收到字节数），缓冲区扩大留余量
- **编排层**：`motor_control_node` 做帧扫描 + CRC 兜底误判 + 最多 1 次重试

**A 和 B 均同意 C 的方案。**

---

## 最终实施

| 文件 | 修改内容 |
|------|----------|
| `serial_interface.hpp` | `sendRecvAccumulate` 返回 `ssize_t`，参数 `expected_len` 改为 `max_len` |
| `serial_interface.cpp` | 实现返回实际收到字节数，发送失败返回 -1，超时返回 0 |
| `motor_control_node.cpp` | `writeUnitreeNativeMotors` 使用 48 字节缓冲区，帧扫描搜索 `0xFD 0xEE`，失败时重试 1 次，记录 `[HOT_PLUG]` 日志 |

**编译状态**: ✅ 通过（`colcon build` 无警告无错误）
