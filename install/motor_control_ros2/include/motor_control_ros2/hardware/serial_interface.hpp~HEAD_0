#ifndef MOTOR_CONTROL_ROS2__HARDWARE__SERIAL_INTERFACE_HPP_
#define MOTOR_CONTROL_ROS2__HARDWARE__SERIAL_INTERFACE_HPP_

#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include <map>
#include <cstdint>
#include <termios.h>

namespace motor_control {
namespace hardware {

/**
 * @brief 串口接收回调函数类型
 * @param data 数据指针
 * @param len 数据长度
 */
using SerialRxCallback = std::function<void(const uint8_t* data, size_t len)>;

/**
 * @brief 串口接口类
 * 
 * 支持高速串口通信（最高 4Mbps）。
 * 用于宇树 GO-8010 电机的 RS485 通信。
 * 
 * 特性：
 * - 支持高波特率（4000000 bps）
 * - 非阻塞读写
 * - 接收线程
 * - 统计信息
 */
class SerialInterface {
public:
  /**
   * @brief 构造函数
   * @param port 串口设备路径（如 /dev/ttyUSB0）
   * @param baudrate 波特率（默认 4000000）
   */
  SerialInterface(const std::string& port, int baudrate = 4000000);
  
  /**
   * @brief 析构函数
   */
  ~SerialInterface();
  
  /**
   * @brief 打开串口
   * @return 成功返回 true
   */
  bool open();
  
  /**
   * @brief 关闭串口
   */
  void close();
  
  /**
   * @brief 是否已打开
   */
  bool isOpen() const { return fd_ >= 0; }
  
  /**
   * @brief 获取端口名称
   */
  std::string getPortName() const { return port_name_; }
  
  /**
   * @brief 发送数据
   * @param data 数据指针
   * @param len 数据长度
   * @return 实际发送的字节数
   */
  ssize_t send(const uint8_t* data, size_t len);
  
  /**
   * @brief 接收数据（非阻塞）
   * @param buffer 接收缓冲区
   * @param max_len 最大接收长度
   * @return 实际接收的字节数，-1 表示错误
   */
   ssize_t receive(uint8_t* buffer, size_t max_len);
   
   /**
    * @brief 发送并接收（原子操作，匹配官方 SDK）
    * @param send_data 发送数据
    * @param send_len 发送数据长度
    * @param recv_buffer 接收缓冲区
    * @param recv_len 接收数据长度
    * @return 成功返回 true
    */
   bool sendRecv(const uint8_t* send_data, size_t send_len, uint8_t* recv_buffer, size_t recv_len);

   /**
    * @brief 请求-响应模式的发送接收（适用于宇树电机等半双工协议）
    *
    * 流程: 发送数据 → 等待 wait_ms → 累积接收数据直到 max_len 或超时
    *
    * @param send_data 发送数据
    * @param send_len 发送数据长度
    * @param recv_buffer 接收缓冲区（大小应 >= max_len）
    * @param max_len 最大接收长度（缓冲区上限）
    * @param wait_ms 发送后等待时间（毫秒）
    * @param timeout_ms 接收超时时间（毫秒）
    * @return 实际收到的字节数；-1 表示发送失败；0 表示超时无数据
    */
   ssize_t sendRecvAccumulate(const uint8_t* send_data, size_t send_len,
                              uint8_t* recv_buffer, size_t max_len,
                              int wait_ms = 10, int timeout_ms = 20);
   
   /**
    * @brief 设置 RS485 方向控制
    * @param tx_mode true=发送模式, false=接收模式
    */
   void setRs485Direction(bool tx_mode);
   
   /**
    * @brief 设置接收回调
   * @param callback 回调函数
   */
  void setRxCallback(SerialRxCallback callback);
  
  /**
   * @brief 启动接收线程
   */
  void startRxThread();
  
  /**
   * @brief 停止接收线程
   */
  void stopRxThread();
  
   /**
    * @brief 获取统计信息
    */
   struct Statistics {
     uint64_t tx_bytes;       // 发送字节数
     uint64_t rx_bytes;       // 接收字节数
     uint64_t tx_errors;      // 发送错误
     uint64_t rx_errors;      // 接收错误
   };
   
   Statistics getStatistics() const { return stats_; }
   void resetStatistics();
   
   /**
    * @brief 获取文件描述符
    */
   int getFd() const { return fd_; }
   
private:
  // 串口配置
  std::string port_;
  std::string port_name_;  // 保存端口名用于官方库
  int baudrate_;
  int fd_;  // 文件描述符
  
  // 接收缓冲区
  static constexpr size_t RX_BUFFER_SIZE = 1024;
  uint8_t rx_buffer_[RX_BUFFER_SIZE];
  
  // 接收回调
  SerialRxCallback rx_callback_;
  
  // 接收线程
  std::thread rx_thread_;
  std::atomic<bool> rx_running_;
  
  // 统计信息
  Statistics stats_;
  mutable std::mutex stats_mutex_;
  
  // 内部方法
  void receiveLoop();
  speed_t getBaudrateConstant(int baudrate);
};

/**
 * @brief 串口网络管理类
 * 
 * 管理多个串口设备（未来扩展用）。
 */
class SerialNetwork {
public:
  SerialNetwork();
  ~SerialNetwork();
  
  /**
   * @brief 添加串口接口
   * @param name 接口名称（如 "unitree_port"）
   * @param port 串口设备路径
   * @param baudrate 波特率
   * @return 成功返回 true
   */
  bool addInterface(const std::string& name, const std::string& port, int baudrate = 4000000);
  
  /**
   * @brief 获取接口
   * @param name 接口名称
   * @return 接口指针，不存在返回 nullptr
   */
  std::shared_ptr<SerialInterface> getInterface(const std::string& name);
  
  /**
   * @brief 发送数据
   * @param interface_name 接口名称
   * @param data 数据指针
   * @param len 数据长度
   * @return 实际发送的字节数
   */
  ssize_t send(const std::string& interface_name, const uint8_t* data, size_t len);
  
  /**
   * @brief 启动所有接收线程
   */
  void startAll();
  
  /**
   * @brief 停止所有接收线程
   */
  void stopAll();
  
  /**
   * @brief 关闭所有接口
   */
  void closeAll();

private:
  std::map<std::string, std::shared_ptr<SerialInterface>> interfaces_;
  std::mutex mutex_;
};

} // namespace hardware
} // namespace motor_control

#endif // MOTOR_CONTROL_ROS2__HARDWARE__SERIAL_INTERFACE_HPP_
