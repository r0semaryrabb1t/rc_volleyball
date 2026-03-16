# 宇树电机串口通信超时 BUG 修复记录

**日期：** 2026-03-15  
**影响版本：** 修复前所有版本  
**相关文件：**
- `src/motor_control_ros2/src/hardware/serial_interface.cpp`
- `src/motor_control_ros2/src/motor_control_node.cpp`

---

## 故障现象

- 控制节点启动日志显示：`控制频率: 200.0 Hz, 宇树电机(原生): 3`（配置正常）
- 监控界面实际显示频率仅 **1~2 Hz**（远低于目标 200 Hz）
- 3 台电机状态在 **在线 / 离线 之间反复切换**，无法稳定工作

---

## 根因分析

### BUG 1：`sendRecvAccumulate` 请求读取 48 字节，但电机只回 16 字节

```cpp
// 错误调用（motor_control_node.cpp）
ssize_t n = serial->sendRecvAccumulate(cmd, 17, buf, BUF_SIZE, 10, 20);
//                                              ^^^^^^^^
//                                    max_len=48，电机实际只发 16 字节
//                                    → 收满16字节后仍继续等待，直到超时
```

### BUG 2：超时计时只计 `sleep(1ms)` 次数，漏算 `receive()` 自身的阻塞时间

```cpp
// 错误实现（serial_interface.cpp）
while (total_received < max_len && elapsed_ms < timeout_ms) {
    ssize_t r = receive(...);   // 内部 select() 阻塞 20ms
    if (r > 0) {
        total_received += r;
    } else {
        sleep(1ms);
        elapsed_ms++;           // 只加 1ms，实际每轮消耗 21ms
    }
}
// timeout_ms=20 → 实际循环 20 轮 × 21ms = 420ms！
```

### 两个 BUG 叠加后的实际耗时

| 阶段 | 修复前 | 修复后 |
|------|--------|--------|
| `wait_ms` 等待设备处理 | 10 ms | 2 ms |
| 接收 16 字节帧 | 即时 | 即时 |
| 等待凑满 48 字节（BUG 1+2） | **420 ms** | 0 ms（16字节立即退出） |
| **单台电机单次通信** | **~430 ms** | **~3 ms** |
| **3 台电机串行合计** | **~1290 ms → 0.77 Hz** | **~9 ms → ~110 Hz** |

> 控制循环周期 ~1.3 s，远超心跳超时阈值 500 ms，每次循环间隙电机均被判为离线，
> 下次循环收到数据又变为在线，导致反复切换。

---

## 修复内容

### 1. `serial_interface.cpp` — `receive()` 内部 `select` 超时

```cpp
// 修复前
tv.tv_usec = 20000;  // 20ms

// 修复后
tv.tv_usec = 2000;   // 2ms（4Mbps 下 16 字节传输约 0.032ms，2ms 已有充足余量）
```

### 2. `serial_interface.cpp` — `sendRecvAccumulate` 改用墙钟 deadline

```cpp
// 修复前（计数 sleep 次数，漏算 receive() 阻塞时间）
int elapsed_ms = 0;
while (total_received < max_len && elapsed_ms < timeout_ms) {
    ssize_t r = receive(...);
    if (r > 0) { total_received += r; }
    else { sleep(1ms); elapsed_ms++; }
}

// 修复后（墙钟 deadline，计时精确）
auto deadline = steady_clock::now() + milliseconds(timeout_ms);
while (total_received < max_len) {
    if (steady_clock::now() >= deadline) break;
    ssize_t r = receive(...);
    if (r > 0) { total_received += r; }
}
```

### 3. `motor_control_node.cpp` — `max_len` 改为 `FRAME_LEN`，`wait_ms` 降低

```cpp
// 修复前
ssize_t n = serial->sendRecvAccumulate(cmd, 17, buf, BUF_SIZE, 10, 20);
//                                              ^^^^^^^^  ^^^  ^^  ^^
//                                              max=48   wait=10ms timeout=20ms

// 修复后
ssize_t n = serial->sendRecvAccumulate(cmd, 17, buf, FRAME_LEN, 2, 8);
//                                              ^^^^^^^^^  ^^^  ^  ^
//                                              max=16    wait=2ms timeout=8ms
```

---

## 修复效果

- 宇树电机实际通信频率恢复至 **100+ Hz**
- 3 台电机状态均稳定**在线**，不再抖动
- 控制节点整体控制频率恢复至配置值 **200 Hz**

---

## 经验总结

1. **`select` 超时是每次调用的最小等待时间**，在累积接收循环中会被反复叠加，需设置为远小于单帧传输时间的值。
2. **超时计时必须使用墙钟**（`steady_clock`），不能依赖"计 sleep 次数"估算，因为其他阻塞调用的时间会被漏计。
3. **`max_len` 应精确等于预期帧长**，多余的缓冲应在帧扫描逻辑中处理，而不是让接收循环无意义地等待填满。
