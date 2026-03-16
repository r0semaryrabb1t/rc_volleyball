// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from motor_control_ros2:msg/UnitreeGO8010Command.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_GO8010_COMMAND__TRAITS_HPP_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_GO8010_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "motor_control_ros2/msg/detail/unitree_go8010_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace motor_control_ros2
{

namespace msg
{

inline void to_flow_style_yaml(
  const UnitreeGO8010Command & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: device
  {
    out << "device: ";
    rosidl_generator_traits::value_to_yaml(msg.device, out);
    out << ", ";
  }

  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: position_target
  {
    out << "position_target: ";
    rosidl_generator_traits::value_to_yaml(msg.position_target, out);
    out << ", ";
  }

  // member: velocity_target
  {
    out << "velocity_target: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_target, out);
    out << ", ";
  }

  // member: torque_ff
  {
    out << "torque_ff: ";
    rosidl_generator_traits::value_to_yaml(msg.torque_ff, out);
    out << ", ";
  }

  // member: kp
  {
    out << "kp: ";
    rosidl_generator_traits::value_to_yaml(msg.kp, out);
    out << ", ";
  }

  // member: kd
  {
    out << "kd: ";
    rosidl_generator_traits::value_to_yaml(msg.kd, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UnitreeGO8010Command & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: device
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "device: ";
    rosidl_generator_traits::value_to_yaml(msg.device, out);
    out << "\n";
  }

  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }

  // member: position_target
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position_target: ";
    rosidl_generator_traits::value_to_yaml(msg.position_target, out);
    out << "\n";
  }

  // member: velocity_target
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity_target: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_target, out);
    out << "\n";
  }

  // member: torque_ff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "torque_ff: ";
    rosidl_generator_traits::value_to_yaml(msg.torque_ff, out);
    out << "\n";
  }

  // member: kp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "kp: ";
    rosidl_generator_traits::value_to_yaml(msg.kp, out);
    out << "\n";
  }

  // member: kd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "kd: ";
    rosidl_generator_traits::value_to_yaml(msg.kd, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UnitreeGO8010Command & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace motor_control_ros2

namespace rosidl_generator_traits
{

[[deprecated("use motor_control_ros2::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const motor_control_ros2::msg::UnitreeGO8010Command & msg,
  std::ostream & out, size_t indentation = 0)
{
  motor_control_ros2::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use motor_control_ros2::msg::to_yaml() instead")]]
inline std::string to_yaml(const motor_control_ros2::msg::UnitreeGO8010Command & msg)
{
  return motor_control_ros2::msg::to_yaml(msg);
}

template<>
inline const char * data_type<motor_control_ros2::msg::UnitreeGO8010Command>()
{
  return "motor_control_ros2::msg::UnitreeGO8010Command";
}

template<>
inline const char * name<motor_control_ros2::msg::UnitreeGO8010Command>()
{
  return "motor_control_ros2/msg/UnitreeGO8010Command";
}

template<>
struct has_fixed_size<motor_control_ros2::msg::UnitreeGO8010Command>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<motor_control_ros2::msg::UnitreeGO8010Command>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<motor_control_ros2::msg::UnitreeGO8010Command>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_GO8010_COMMAND__TRAITS_HPP_
