#pragma once

#include "motor_control_ros2/motor_base.hpp"
#include <cstdint>
#include <cmath>
#include <mutex>

namespace motor_control {

/**
 * @brief CRC-CCITT 查表（与Python实现一致）
 */
extern const uint16_t CRC_TABLE[256];

/**
 * @brief 计算CRC-CCITT (LSB-first)
 */
uint16_t calcCrcCcitt(const uint8_t* data, size_t len);

/**
 * @brief GO-M8010-6 电机命令结构（17字节）
 */
#pragma pack(push, 1)
struct UnitreeNativeCommand {
  uint8_t head[2];      // 0xFE, 0xEE
  uint8_t mode;         // id(4bit) | status(3bit) | none(1bit)
  int16_t tor_des;      // 力矩 q8 format
  int16_t spd_des;      // 速度 q7 format
  int32_t pos_des;      // 位置 q15 format
  uint16_t k_pos;       // 刚度 q15 format
  uint16_t k_spd;       // 阻尼 q15 format
  uint16_t crc;         // CRC16
};

/**
 * @brief GO-M8010-6 电机反馈结构（16字节）
 */
struct UnitreeNativeFeedback {
  uint8_t head[2];      // 0xFE, 0xEE
  uint8_t mode;         // id(4bit) | status(3bit) | none(1bit)
  int16_t torque;       // 力矩 q8 format
  int16_t speed;        // 速度 q7 format
  int32_t pos;          // 位置 q15 format
  int8_t temp;          // 温度
  uint8_t error_force;  // MError(3bit) | force(12bit) | none(1bit) 的高8位
  uint8_t force_low;    // force 低8位
  uint16_t crc;         // CRC16
};
#pragma pack(pop)

/**
 * @brief 宇树电机原生协议控制类（纯协议层，不依赖SDK和硬件I/O）
 *
 * 支持 GO-M8010-6 电机，使用定点数协议。
 * 仅负责命令包构建和反馈包解析，串口I/O由编排层完成。
 * 对标 DJIMotor 的分层设计：电机类 = 纯协议编解码。
 */
class UnitreeMotorNative : public MotorBase {
public:
  // 控制模式
  enum class Mode : uint8_t {
    BRAKE = 0,      // 刹车模式
    FOC = 1,        // FOC闭环控制
    CALIBRATE = 2   // 校准模式
  };

  /**
   * @brief 构造函数
   * @param name 电机名称
   * @param motor_id 电机ID (0-15)
   * @param gear_ratio 齿轮减速比
   */
  UnitreeMotorNative(const std::string& name,
                     uint8_t motor_id,
                     double gear_ratio = 6.33);

  ~UnitreeMotorNative() override = default;

  // ========== MotorBase 纯虚函数实现 ==========
  void updateFeedback(const std::string& interface_name,
                      uint32_t can_id,
                      const uint8_t* data,
                      size_t len) override {
    // 串口电机不使用CAN回调
    (void)interface_name;
    (void)can_id;
    (void)data;
    (void)len;
  }

  void getControlFrame(uint32_t& can_id, uint8_t* data, size_t& len) override {
    // 串口电机不使用CAN发送
    can_id = 0;
    len = 0;
    (void)data;
  }

  void enable() override {
    mode_ = Mode::FOC;
  }

  void disable() override {
    mode_ = Mode::BRAKE;
  }

  // ========== 协议层接口（类比 DJIMotor 的 getControlBytes / updateFeedback） ==========

  /**
   * @brief 构建命令包（17字节）
   * @param buffer 输出缓冲区，至少17字节
   * @return 命令包长度（固定17）
   */
  size_t getCommandPacket(uint8_t* buffer);

  /**
   * @brief 解析反馈包并更新电机状态
   * @param data 接收到的数据
   * @param len 数据长度（至少16字节）
   * @return 解析成功返回 true
   */
  bool parseFeedback(const uint8_t* data, size_t len);

  // ========== 控制命令 ==========

  /**
   * @brief 设置FOC命令
   * @param pos_des 目标位置（弧度）
   * @param vel_des 目标速度（弧度/秒）
   * @param kp 刚度系数
   * @param kd 阻尼系数
   * @param torque_ff 前馈力矩（Nm）
   */
  void setFOCCommand(double pos_des, double vel_des,
                     double kp, double kd, double torque_ff);

  /**
   * @brief 设置刹车命令
   */
  void setBrakeCommand();

  /**
   * @brief 设置校准命令
   */
  void setCalibrateCommand();

  // ========== 获取器 ==========
  uint8_t getMotorId() const { return motor_id_; }
  uint8_t getErrorCode() const { return error_code_; }

private:
  /**
   * @brief 构建命令包到内部缓冲区
   */
  void buildCommandPacket();

  // 电机参数
  uint8_t motor_id_;
  double gear_ratio_;

  // 控制命令
  Mode mode_ = Mode::FOC;
  double cmd_pos_des_ = 0.0;    // 目标位置（弧度）
  double cmd_vel_des_ = 0.0;    // 目标速度（弧度/秒）
  double cmd_torque_ff_ = 0.0;  // 前馈力矩（Nm）
  double cmd_kp_ = 0.0;         // 刚度（启动时零力矩，安全）
  double cmd_kd_ = 0.0;         // 阻尼（启动时零力矩，安全）

  // 错误码
  uint8_t error_code_ = 0;

  // 内部工作缓冲区
  uint8_t tx_buffer_[17];

  // 命令字段保护（ROS回调与控制循环可能并发写入）
  mutable std::mutex mutex_;
};

} // namespace motor_control
