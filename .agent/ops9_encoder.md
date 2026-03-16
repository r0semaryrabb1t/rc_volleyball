# OPS9 码盘定位节点使用指南

## 概述

`ops9_encoder_node` 是一个**独立读取节点**，通过串口（115200 baud）读取 OPS9 码盘定位装置的 X/Y 位移、偏航角和加速度数据，并发布到 ROS2 话题。

作为从定位方法（secondary positioning），可为底盘或机械臂提供位置反馈。

---

## 系统架构

```
OPS9 码盘装置 ──► /dev/ttyUSB3 (115200 baud, ASCII)
                        │
                        ▼
               ops9_encoder_node
                        │
                        ▼
              /ops9/encoder_state
                        │
                        ▼
              上层应用（底盘定位/路径规划等）
```

---

## 启动方式

### 使用默认配置

```bash
ros2 run motor_control_ros2 ops9_encoder_node
```

默认加载 `install/.../share/motor_control_ros2/config/encoder.yaml`

### 指定配置文件

```bash
ros2 run motor_control_ros2 ops9_encoder_node \
  --ros-args -p config_file:=/home/toe/111/USB2CAN_motor/src/motor_control_ros2/config/encoder.yaml
```

---

## 配置文件（encoder.yaml）

```yaml
serial:
  device: /dev/ttyUSB3        # 编码器串口设备
  baudrate: 115200            # 波特率

read_frequency: 100.0         # 读取频率（Hz）
heartbeat_timeout_ms: 500.0   # 离线超时（毫秒）
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `serial.device` | `/dev/ttyUSB3` | OPS9 串口设备路径 |
| `serial.baudrate` | `115200` | 波特率（固定） |
| `read_frequency` | `100.0` | 定时器轮询频率（Hz） |
| `heartbeat_timeout_ms` | `500.0` | 超过此时间未收到数据标记离线 |

---

## 串口协议

OPS9 输出 ASCII 文本，格式为 `Key:Value\r\n`，四个字段循环发送：

```
yaw:123.45\r\n
weiyi_x:0.123\r\n
weiyi_y:-0.456\r\n
az:9.81\r\n
```

节点使用**位标志**收集完整帧（4 字段全收齐后发布一次）：

| 标志位 | Key | 含义 |
|--------|-----|------|
| `0x01` | `yaw` | 偏航角（度） |
| `0x02` | `weiyi_x` | X 轴位移 |
| `0x04` | `weiyi_y` | Y 轴位移 |
| `0x08` | `az` | Z 轴加速度 |

---

## Topic 定义

### OPS9EncoderState（发布）

话题名称：`/ops9/encoder_state`

```
std_msgs/Header header

bool online                     # 编码器在线状态
float64 position_x              # X 轴位移
float64 position_y              # Y 轴位移
float64 yaw                     # 偏航角（度）
float64 acceleration_z          # Z 轴加速度
```

---

## 显示信息说明

节点以 `RCLCPP_INFO_THROTTLE`（2 秒节流）输出：

```
OPS9 数据: pos(0.12, -0.45) yaw=123.5° az=9.81
```

离线时输出警告：

```
OPS9 编码器离线（超时 600 ms）
```

---

## 验证步骤

```bash
# 1. 检查串口设备
ls -l /dev/ttyUSB3

# 2. 启动节点
ros2 run motor_control_ros2 ops9_encoder_node

# 3. 查看话题数据
ros2 topic echo /ops9/encoder_state

# 4. 查看发布频率
ros2 topic hz /ops9/encoder_state
```

---

## 故障排查

### 问题 1：无法打开串口

**日志**：`无法打开串口: /dev/ttyUSB3`

**解决方法**：
1. 检查设备是否存在：`ls /dev/ttyUSB*`
2. 检查权限：`sudo chmod 666 /dev/ttyUSB3`
3. 确认设备号未因插拔改变（可用 `udev` 规则固定）

### 问题 2：online=false — 编码器离线

**可能原因**：OPS9 未上电、接线松动、波特率不匹配

**解决方法**：
1. 确认 OPS9 已通电
2. 用 `minicom` 或 `picocom` 测试串口：
   ```bash
   picocom -b 115200 /dev/ttyUSB3
   ```
   应看到 `yaw:...` 等文本输出
3. 检查 `encoder.yaml` 中 baudrate 是否为 115200

### 问题 3：数据解析失败

**日志**：`OPS9 数据解析失败: 'xxx'`

**可能原因**：串口噪声或协议格式变化

**解决方法**：
1. 用 `picocom` 检查原始数据格式
2. 确认格式为 `Key:Value\r\n`

### 问题 4：发布频率过低

**可能原因**：串口数据不完整，需要多个周期才能收齐 4 字段

**解决方法**：
1. 检查 OPS9 自身输出频率
2. 适当提高 `read_frequency`

---

## 配置检查清单

- [ ] 串口设备 `/dev/ttyUSB3` 存在且有读写权限
- [ ] OPS9 已通电，`picocom` 能看到数据
- [ ] `encoder.yaml` 参数正确（device / baudrate）
- [ ] 节点启动无报错
- [ ] `ros2 topic echo /ops9/encoder_state` 有数据
- [ ] `online=true` 且数据持续更新

---

**文档版本**: v1.0
**更新时间**: 2026-03-14
**适用系统**: motor_control_ros2 (OPS9 码盘定位)
