#ifndef MOTOR_CONTROL_ROS2__UNITREE_MOTOR_HPP_
#define MOTOR_CONTROL_ROS2__UNITREE_MOTOR_HPP_

#include "motor_control_ros2/motor_base.hpp"
#include "unitreeMotor/unitreeMotor.h"
#include "serialPort/SerialPort.h"
#include <memory>
#include <string>

namespace motor_control {

class UnitreeMotor : public MotorBase {
public:
  UnitreeMotor(const std::string& joint_name, MotorType type,
               uint8_t motor_id, int direction = 1, float offset = 0.0f);

  void updateFeedback(const std::string& interface_name, uint32_t can_id,
                      const uint8_t* data, size_t len) override;

  void getControlFrame(uint32_t& can_id, uint8_t* data, size_t& len) override;

  void enable() override;
  void disable() override;

  void setHybridCommand(float pos, float vel, float kp, float kd, float torque, uint8_t mode);

  bool sendRecv();

  void setSerialPort(std::shared_ptr<SerialPort> port) {
    sdk_serial_ = port;
  }

  std::shared_ptr<SerialPort> getSerialPort() const {
    return sdk_serial_;
  }

  uint8_t getMotorId() const { return motor_id_; }

private:
  uint8_t motor_id_;
  std::shared_ptr<SerialPort> sdk_serial_;
  MotorCmd sdk_cmd_;
  MotorData sdk_data_;
  float gear_ratio_;
};

}  // namespace motor_control

#endif  // MOTOR_CONTROL_ROS2__UNITREE_MOTOR_HPP_
