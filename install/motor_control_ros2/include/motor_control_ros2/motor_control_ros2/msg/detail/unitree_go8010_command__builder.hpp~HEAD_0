// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motor_control_ros2:msg/UnitreeGO8010Command.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_GO8010_COMMAND__BUILDER_HPP_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_GO8010_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motor_control_ros2/msg/detail/unitree_go8010_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motor_control_ros2
{

namespace msg
{

namespace builder
{

class Init_UnitreeGO8010Command_kd
{
public:
  explicit Init_UnitreeGO8010Command_kd(::motor_control_ros2::msg::UnitreeGO8010Command & msg)
  : msg_(msg)
  {}
  ::motor_control_ros2::msg::UnitreeGO8010Command kd(::motor_control_ros2::msg::UnitreeGO8010Command::_kd_type arg)
  {
    msg_.kd = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeGO8010Command msg_;
};

class Init_UnitreeGO8010Command_kp
{
public:
  explicit Init_UnitreeGO8010Command_kp(::motor_control_ros2::msg::UnitreeGO8010Command & msg)
  : msg_(msg)
  {}
  Init_UnitreeGO8010Command_kd kp(::motor_control_ros2::msg::UnitreeGO8010Command::_kp_type arg)
  {
    msg_.kp = std::move(arg);
    return Init_UnitreeGO8010Command_kd(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeGO8010Command msg_;
};

class Init_UnitreeGO8010Command_torque_ff
{
public:
  explicit Init_UnitreeGO8010Command_torque_ff(::motor_control_ros2::msg::UnitreeGO8010Command & msg)
  : msg_(msg)
  {}
  Init_UnitreeGO8010Command_kp torque_ff(::motor_control_ros2::msg::UnitreeGO8010Command::_torque_ff_type arg)
  {
    msg_.torque_ff = std::move(arg);
    return Init_UnitreeGO8010Command_kp(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeGO8010Command msg_;
};

class Init_UnitreeGO8010Command_velocity_target
{
public:
  explicit Init_UnitreeGO8010Command_velocity_target(::motor_control_ros2::msg::UnitreeGO8010Command & msg)
  : msg_(msg)
  {}
  Init_UnitreeGO8010Command_torque_ff velocity_target(::motor_control_ros2::msg::UnitreeGO8010Command::_velocity_target_type arg)
  {
    msg_.velocity_target = std::move(arg);
    return Init_UnitreeGO8010Command_torque_ff(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeGO8010Command msg_;
};

class Init_UnitreeGO8010Command_position_target
{
public:
  explicit Init_UnitreeGO8010Command_position_target(::motor_control_ros2::msg::UnitreeGO8010Command & msg)
  : msg_(msg)
  {}
  Init_UnitreeGO8010Command_velocity_target position_target(::motor_control_ros2::msg::UnitreeGO8010Command::_position_target_type arg)
  {
    msg_.position_target = std::move(arg);
    return Init_UnitreeGO8010Command_velocity_target(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeGO8010Command msg_;
};

class Init_UnitreeGO8010Command_mode
{
public:
  explicit Init_UnitreeGO8010Command_mode(::motor_control_ros2::msg::UnitreeGO8010Command & msg)
  : msg_(msg)
  {}
  Init_UnitreeGO8010Command_position_target mode(::motor_control_ros2::msg::UnitreeGO8010Command::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_UnitreeGO8010Command_position_target(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeGO8010Command msg_;
};

class Init_UnitreeGO8010Command_device
{
public:
  explicit Init_UnitreeGO8010Command_device(::motor_control_ros2::msg::UnitreeGO8010Command & msg)
  : msg_(msg)
  {}
  Init_UnitreeGO8010Command_mode device(::motor_control_ros2::msg::UnitreeGO8010Command::_device_type arg)
  {
    msg_.device = std::move(arg);
    return Init_UnitreeGO8010Command_mode(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeGO8010Command msg_;
};

class Init_UnitreeGO8010Command_id
{
public:
  explicit Init_UnitreeGO8010Command_id(::motor_control_ros2::msg::UnitreeGO8010Command & msg)
  : msg_(msg)
  {}
  Init_UnitreeGO8010Command_device id(::motor_control_ros2::msg::UnitreeGO8010Command::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_UnitreeGO8010Command_device(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeGO8010Command msg_;
};

class Init_UnitreeGO8010Command_header
{
public:
  Init_UnitreeGO8010Command_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UnitreeGO8010Command_id header(::motor_control_ros2::msg::UnitreeGO8010Command::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UnitreeGO8010Command_id(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeGO8010Command msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_control_ros2::msg::UnitreeGO8010Command>()
{
  return motor_control_ros2::msg::builder::Init_UnitreeGO8010Command_header();
}

}  // namespace motor_control_ros2

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_GO8010_COMMAND__BUILDER_HPP_
