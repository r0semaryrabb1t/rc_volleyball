#include "motor_control_ros2/unitree_motor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>

namespace motor_control {

UnitreeMotor::UnitreeMotor(const std::string& joint_name, MotorType type,
                           uint8_t motor_id, int direction, float offset)
  : MotorBase(joint_name, type,
              (type == MotorType::UNITREE_A1) ? 9.1f : 6.33f,
              true)
  , motor_id_(motor_id)
  , gear_ratio_(type == MotorType::UNITREE_A1 ? 9.1f : 6.33f)
{
  if (type == MotorType::UNITREE_A1) {
    sdk_cmd_.motorType = ::MotorType::A1;
    sdk_data_.motorType = ::MotorType::A1;
  } else {
    sdk_cmd_.motorType = ::MotorType::GO_M8010_6;
    sdk_data_.motorType = ::MotorType::GO_M8010_6;
  }
  sdk_cmd_.id = motor_id;
  sdk_cmd_.mode = queryMotorMode(sdk_cmd_.motorType, MotorMode::BRAKE);
  sdk_cmd_.kp = 0.0f;
  sdk_cmd_.kd = 0.0f;
  sdk_cmd_.q = 0.0f;
  sdk_cmd_.dq = 0.0f;
  sdk_cmd_.tau = 0.0f;
}

void UnitreeMotor::updateFeedback(const std::string& /*interface_name*/, uint32_t /*can_id*/,
                                  const uint8_t* /*data*/, size_t /*len*/) {
}

void UnitreeMotor::getControlFrame(uint32_t& /*can_id*/, uint8_t* /*data*/, size_t& len) {
  len = 0;
}

void UnitreeMotor::enable() {
  sdk_cmd_.mode = queryMotorMode(sdk_cmd_.motorType, MotorMode::FOC);
}

void UnitreeMotor::disable() {
  sdk_cmd_.mode = queryMotorMode(sdk_cmd_.motorType, MotorMode::BRAKE);
  sdk_cmd_.kp = 0.0f;
  sdk_cmd_.kd = 0.0f;
  sdk_cmd_.q = 0.0f;
  sdk_cmd_.dq = 0.0f;
  sdk_cmd_.tau = 0.0f;
}

void UnitreeMotor::setHybridCommand(float pos, float vel, float kp, float kd, float torque, uint8_t mode) {
  sdk_cmd_.kp = kp;
  sdk_cmd_.kd = kd;
  sdk_cmd_.tau = torque;
  sdk_cmd_.mode = mode;
  sdk_cmd_.q = pos / gear_ratio_;
  sdk_cmd_.dq = vel / gear_ratio_;
}

bool UnitreeMotor::sendRecv() {
  if (!sdk_serial_) {
    return false;
  }

  bool success = sdk_serial_->sendRecv(&sdk_cmd_, &sdk_data_);

  if (success) {
     updateLastFeedbackTime(std::chrono::steady_clock::now().time_since_epoch().count());
    position_ = sdk_data_.q * gear_ratio_;
    velocity_ = sdk_data_.dq * gear_ratio_;
    torque_ = sdk_data_.tau;
    temperature_ = static_cast<float>(sdk_data_.temp);
   
  } else {
    online_ = 0;
  }

  return success;
}

}  // namespace motor_control
