# 问题一：宇树电机串口热插拔 — 根因分析与修复方案

## 现象

不关闭 `motor_control_node`，直接断电重启全部 3 台宇树 GO-M8010-6 电机后，**只有 1 台电机能恢复在线**，另外 2 台持续离线。必须重启整个节点才能恢复全部 3 台。

### 电机拓扑

```
motor_control_node
  ├── /dev/ttyUSB0 → arm_motor_1 (ID=0)  ← 碰巧恢复
  ├── /dev/ttyUSB1 → arm_motor_2 (ID=0)  ← 持续离线
  └── /dev/ttyUSB2 → arm_motor_3 (ID=0)  ← 持续离线
```

每个串口独立、同 ID=0、波特率 4000000 bps。

---

## 根因深度分析

### 1. 数据流回顾

`motor_control_node::controlLoop()` (200Hz) 对每台宇树电机执行：

```
sendRecvAccumulate(cmd[17], response[16], wait_ms=10, timeout_ms=20)
  ├── tcflush(TCIFLUSH)      // 清空 RX 内核缓冲区
  ├── send(cmd, 17)          // 发送 17 字节命令
  ├── tcdrain()              // 等待 TX 完成
  ├── sleep(10ms)            // 等待电机处理
  └── 累积 receive() 直到收满 16 字节或超时 20ms
```

### 2. 断电重启时序分析

```
时间轴:
─────────────────────────────────────────────────────────────
  t0          t1            t2          t3          t4
  断电        重新通电      控制循环N    控制循环N+1  ...
─────────────────────────────────────────────────────────────

t0~t1: 电机断电期间
  - send() 正常（数据到了 RS485 线但无应答）
  - receive() 超时返回 0 → parseFeedback 不执行 → online 超时变 false
  - ✅ 正常行为

t1~t2: 电机刚通电瞬间（关键！）
  - 电机上电 Boot (~5-50ms)，可能发送初始化字节/噪声脉冲
  - 这些字节被 USB-Serial 芯片（CH340/FTDI/CP2102）缓存在硬件 FIFO
  - 控制循环仍在 200Hz 运行
```

### 3. 病灶：tcflush 与乱序字节的竞态

当前代码 `sendRecvAccumulate()` **已有 `tcflush(TCIFLUSH)`**，但这不够：

```
tcflush(TCIFLUSH)           ← 清空此刻的内核 RX 缓冲区
send(cmd, 17) + tcdrain()   ← 花费 ~40μs（17×10bit / 4Mbps）
sleep(10ms)                 ← 等待电机处理
                            ← ⚠️ 在这 10ms 内，上电噪声/旧帧残片
                            ←   可能从硬件 FIFO 到达内核缓冲区！
receive() ...               ← 读到了 [噪声字节 + 部分有效响应]
```

**结果**：
- `receive()` 读到的前几个字节不是 `0xFD 0xEE`（有效帧头）
- 整个 16 字节响应发生**帧错位**
- `parseFeedback()` 检查帧头失败 → 返回 `false`
- 连续失败 → 心跳超时 → 持续离线

### 4. 为什么碰巧有 1 台能恢复？

3 台电机上电时间有微小差异（±数毫秒），恰好有 1 台：
- 上电时序刚好在 `tcflush` 之后、`sleep(10ms)` 之前没有产生噪声
- 或者噪声字节恰好被 `tcflush` 清掉了

### 5. 深层问题：USB-Serial 芯片硬件 FIFO

`tcflush(TCIFLUSH)` 只清空**内核软件缓冲区**，不清空 USB-Serial 转换芯片的硬件 FIFO。CH340 有 32 字节硬件 FIFO，FTDI 有 256 字节。这些字节会在之后的 USB 轮询中被搬运到内核缓冲区。

---

## 专家研讨会议

### Sub-Agent A（嵌入式工程师 / 串口协议专家）

> **激进方案：帧扫描 + 对齐恢复**
>
> tcflush 本身没错，但它无法解决"硬件 FIFO 延迟搬运"的问题。我主张在 `parseFeedback` 调用前加一层**帧扫描**：
>
> 1. 接收缓冲区从 16 字节扩大到 64 字节
> 2. 在收到的数据中搜索 `0xFD 0xEE` 帧头
> 3. 如果帧头不在 offset 0，跳过前面的垃圾字节，从帧头处开始解析
> 4. 这样即使有噪声前缀，也能正确解析
>
> 这是工业串口通信的标准做法（如 Modbus RTU 也有类似帧对齐机制）。

### Sub-Agent B（测试/运维工程师）

> **质疑：帧扫描可能引入新 Bug**
>
> 我反对直接在底层加帧扫描。理由：
> 1. 如果 `0xFD 0xEE` 恰好出现在噪声数据中（概率 1/65536），会导致误解析
> 2. 扩大接收缓冲区可能收到上一帧的残留 + 当前帧 → 帧边界混乱
> 3. 当前 `sendRecvAccumulate` 的职责是"收满 N 字节"，不应该承担帧对齐的语义
>
> 我主张更保守的方案：**失败重试 + 二次 flush**
> - 如果 `parseFeedback` 失败，立即 `tcflush` + 再发一次命令
> - 最多重试 2 次（30ms 内重试 2 次，不影响 200Hz 循环的整体延迟）
> - 这样第二次大概率能对齐

### Sub-Agent C（架构师 / 折中方案）

> **折中：分层修复，帧扫描放在协议层，重试放在编排层**
>
> 两位的担忧都有道理。我建议分层处理：
>
> **硬件层（serial_interface.cpp）**：
> - `sendRecvAccumulate` 保持职责不变：收满 N 字节或超时
> - 但接收缓冲区扩大到 expected_len + 16 字节（留余量吸收噪声前缀）
> - 返回**实际收到的字节数**，让上层判断
>
> **编排层（motor_control_node.cpp → writeUnitreeNativeMotors）**：
> - 接收到数据后，在数据中搜索 `0xFD 0xEE` 帧头
> - 如果帧头在 offset > 0，说明有噪声前缀，从帧头处取 16 字节解析
> - 如果找不到帧头或 CRC 失败，执行一次重试（flush + 重发）
> - 重试仍失败则标记本轮通信失败，等下一个 200Hz 周期
>
> 这样：
> - ✅ 帧扫描逻辑不污染通用的 `SerialInterface`
> - ✅ 误判 `0xFD 0xEE` 的风险被 CRC 校验兜底
> - ✅ 单次重试的延迟（~30ms）在 200Hz (5ms) 周期内虽然超时，但仅在热插拔恢复瞬间触发，不影响稳态性能

### 讨论结果

**A**：我同意 C 的分层设计。帧扫描放在编排层确实更干净。CRC 校验兜底 0xFDEE 误判的问题。

**B**：这个方案我接受。帧扫描 + CRC 双保险比纯重试更快恢复。不过我要求：重试次数硬编码为 1 次（不能无限重试），并在日志中记录"热插拔恢复"事件。

**A**：同意限制重试 1 次。补充一点：`sendRecvAccumulate` 应该返回实际收到的字节数而不只是 bool，这样编排层才能做帧扫描。

**C**：同意。我更新方案：`sendRecvAccumulate` 返回 `ssize_t`（收到字节数），-1 表示发送失败，0 表示超时，>0 表示收到的字节数。

**三人一致赞同。**

---

## 最终方案

### 修改 1: `serial_interface.hpp / .cpp` — sendRecvAccumulate 返回实际字节数

```cpp
// 修改前：
bool sendRecvAccumulate(..., size_t expected_len, ...);

// 修改后：
ssize_t sendRecvAccumulate(..., size_t max_len, ...);
// 返回值：-1=发送失败，0=超时无数据，>0=实际收到字节数
```

### 修改 2: `motor_control_node.cpp` — writeUnitreeNativeMotors 增加帧扫描 + 重试

```cpp
void writeUnitreeNativeMotors() {
  for (auto& motor : unitree_native_motors_) {
    auto serial = serial_network_->getInterface(motor->getInterfaceName());
    if (!serial || !serial->isOpen()) continue;

    uint8_t cmd[17];
    motor->getCommandPacket(cmd);

    // 接收缓冲区留余量（16字节帧 + 最多16字节噪声前缀）
    uint8_t response[48];
    constexpr size_t FRAME_LEN = 16;

    auto try_recv_parse = [&]() -> bool {
      ssize_t n = serial->sendRecvAccumulate(cmd, 17, response, sizeof(response), 10, 20);
      if (n <= 0) return false;

      // 帧扫描：在收到的数据中搜索 0xFD 0xEE
      for (ssize_t offset = 0; offset + FRAME_LEN <= n; ++offset) {
        if (response[offset] == 0xFD && response[offset + 1] == 0xEE) {
          if (motor->parseFeedback(&response[offset], FRAME_LEN)) {
            return true;
          }
        }
      }
      return false;
    };

    // 第一次尝试
    if (!try_recv_parse()) {
      // 重试 1 次（热插拔恢复）
      if (try_recv_parse()) {
        RCLCPP_INFO_THROTTLE(..., "[UnitreeNative] %s: 热插拔恢复成功（重试1次）", ...);
      } else {
        RCLCPP_WARN_THROTTLE(..., "[UnitreeNative] %s: 通信失败", ...);
      }
    }
  }
}
```

### 修改 3: 日志增强

- 新增 `[HOT_PLUG]` 日志标签，记录恢复事件
- 连续失败计数，超过 100 次时降低日志频率（避免日志洪泛）

### 不修改

- `parseFeedback()` 逻辑不变（帧头 + ID + CRC 三重校验足够）
- `tcflush` 保留（虽然不能解决全部问题，但仍有收益）
- 控制频率 200Hz 不变

### 风险评估

| 风险 | 概率 | 影响 | 缓解 |
|------|------|------|------|
| 0xFDEE 出现在噪声中 | 1/65536 | CRC 二次拦截 | CRC 误判概率再降 1/65536 → 总概率 ~4×10⁻¹⁰ |
| 重试导致控制周期抖动 | 低（仅热插拔恢复瞬间） | 1-2 个周期延迟 | 单次重试限制 + 200Hz 裕量 |
| sendRecvAccumulate 接口变更 | - | 编译错误 | 同步更新所有调用点 |

---

*编写日期: 2026-03-14*
*状态: 待实施*
