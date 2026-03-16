#ifndef MOTOR_CONTROL_ROS2__UNITREE_MOTOR_HPP_
#define MOTOR_CONTROL_ROS2__UNITREE_MOTOR_HPP_

#include "motor_control_ros2/motor_base.hpp"
#include "motor_control_ros2/unitree_8010_protocol.hpp"
#include <memory>
#include <string>

namespace motor_control {

/**
 * @brief 宇树电机类（SDK版本占位符）
 * 
 * 此类保持向后兼容，但建议使用 UnitreeMotorNative 类
 * UnitreeMotorNative 实现了完整的原生协议支持，不依赖SDK
 */
class UnitreeMotor : public MotorBase {
public:
  UnitreeMotor(const std::string& joint_name, uint8_t motor_id, 
               int direction = 1, float offset = 0.0f,
               float gear_ratio = 6.33f, float k_pos = 0.20f, float k_spd = 0.04f)
    : MotorBase(joint_name, MotorType::UNITREE_GO8010, gear_ratio, false)
    , motor_id_(motor_id)
    , gear_ratio_(gear_ratio)
    , k_pos_(k_pos)
    , k_spd_(k_spd)
    , cmd_pos_(0.0f)
    , cmd_vel_(0.0f)
    , cmd_kp_(k_pos)
    , cmd_kd_(k_spd)
    , cmd_torque_(0.0f)
    , cmd_mode_(static_cast<uint8_t>(GO8010Mode::FOC))
  {
    (void)direction;
    (void)offset;
  }

  void updateFeedback(const std::string& interface_name, uint32_t can_id,
                      const uint8_t* data, size_t len) override {
    // SDK版本通过SDK获取反馈，此处为占位符
    (void)interface_name;
    (void)can_id;
    (void)data;
    (void)len;
  }

  void getControlFrame(uint32_t& can_id, uint8_t* data, size_t& len) override {
    // SDK版本不使用CAN发送
    can_id = 0;
    len = 0;
    (void)data;
  }

  void enable() override {
    cmd_mode_ = static_cast<uint8_t>(GO8010Mode::FOC);
  }
  
  void disable() override {
    cmd_mode_ = static_cast<uint8_t>(GO8010Mode::BRAKE);
  }

  void setHybridCommand(float pos, float vel, float kp, float kd, float torque, uint8_t mode) {
    cmd_pos_ = pos;
    cmd_vel_ = vel;
    cmd_kp_ = kp;
    cmd_kd_ = kd;
    cmd_torque_ = torque;
    cmd_mode_ = mode;
  }

  void setBrakeCommand() {
    cmd_mode_ = static_cast<uint8_t>(GO8010Mode::BRAKE);
    cmd_kp_ = 0.0f;
    cmd_kd_ = 0.0f;
    cmd_torque_ = 0.0f;
  }
  
  void setFOCCommand(float position_target, float velocity_target,
                     float kp, float kd, float torque_ff) {
    cmd_mode_ = static_cast<uint8_t>(GO8010Mode::FOC);
    cmd_pos_ = position_target;
    cmd_vel_ = velocity_target;
    cmd_kp_ = kp;
    cmd_kd_ = kd;
    cmd_torque_ = torque_ff;
  }
  
  void setCalibrateCommand() {
    cmd_mode_ = static_cast<uint8_t>(GO8010Mode::CALIBRATE);
  }

  b
  uint8_t getMotorId() const { return motor_id_; }

private:
  uint8_t motor_id_;
  float gear_ratio_;
  float k_pos_;
  float k_spd_;
  
  // 缓存当前命令参数
  float cmd_pos_;
  float cmd_vel_;
  float cmd_kp_;
  float cmd_kd_;
  float cmd_torque_;
  uint8_t cmd_mode_;
};

}  // namespace motor_control

#endif  // MOTOR_CONTROL_ROS2__UNITREE_MOTOR_HPP_
