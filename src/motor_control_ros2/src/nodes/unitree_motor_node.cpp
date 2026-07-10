/**
 * unitree_motor_node.cpp
 *
 * 独立的宇树 GO-M8010-6 电机控制节点
 *
 * 与 DJI 底盘控制节点完全解耦，不共享任何状态或资源。
 * 控制循环独立运行，互不干扰。
 *
 * 通信: RS485 半双工 @ 4Mbps
 * 协议: UnitreeMotorNative (CRC-CCITT, 17字节命令/16字节反馈)
 */

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <functional>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "motor_control_ros2/config_parser.hpp"
#include "motor_control_ros2/unitree_motor_native.hpp"
#include "motor_control_ros2/hardware/serial_interface.hpp"
#include "motor_control_ros2/msg/unitree_go8010_command.hpp"
#include "motor_control_ros2/msg/unitree_go8010_state.hpp"

namespace motor_control {

class UnitreeMotorNode : public rclcpp::Node {
public:
  UnitreeMotorNode() : Node("unitree_motor_node") {
    this->declare_parameter("control_frequency", 200.0);
    this->declare_parameter("command_timeout", 0.5);
    this->declare_parameter("config_file", "");

    loadConfig();
    serial_network_ = std::make_shared<hardware::SerialNetwork>();
    initializeSerialInterfaces();

    state_pub_ = this->create_publisher<
      motor_control_ros2::msg::UnitreeGO8010State>(
      "unitree_go8010_states", 10);

    cmd_sub_ = this->create_subscription<
      motor_control_ros2::msg::UnitreeGO8010Command>(
      "unitree_go8010_command", 10,
      std::bind(&UnitreeMotorNode::commandCallback, this, std::placeholders::_1));

    target_control_freq_ = this->get_parameter("control_frequency").as_double();
    command_timeout_ = this->get_parameter("command_timeout").as_double();
    last_freq_time_ = this->now();

    const auto period = std::chrono::duration<double>(1.0 / target_control_freq_);
    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&UnitreeMotorNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(),
      "unitree_motor_node 启动: motors=%zu, freq=%.1fHz",
      unitree_motors_.size(), target_control_freq_);
  }

  ~UnitreeMotorNode() override {
    if (serial_network_) {
      serial_network_->stopAll();
      serial_network_->closeAll();
    }
  }

private:
  // ========== 配置加载 ==========

  std::string packageShareDir() const {
    return ament_index_cpp::get_package_share_directory("motor_control_ros2");
  }

  void loadConfig() {
    std::string config_file = this->get_parameter("config_file").as_string();
    if (config_file.empty()) {
      config_file = packageShareDir() + "/config/motors.yaml";
    } else if (config_file.front() != '/') {
      config_file = packageShareDir() + "/" + config_file;
    }
    config_ = ConfigParser::loadConfig(config_file);
  }

  void initializeSerialInterfaces() {
    int interface_index = 0;
    for (const auto& serial_config : config_.serial_interfaces) {
      const std::string iface_name =
        "serial_" + std::to_string(interface_index++);

      bool ok = serial_network_->addInterface(
        iface_name, serial_config.device, serial_config.baudrate);
      if (!ok) {
        RCLCPP_WARN(this->get_logger(),
          "跳过无法打开的串口: %s (%s)",
          iface_name.c_str(), serial_config.device.c_str());
        continue;
      }

      for (const auto& motor_cfg : serial_config.motors) {
        if (motor_cfg.type != "UNITREE_GO8010") {
          continue;
        }
        addMotor(motor_cfg, iface_name);
      }
    }
  }

  void addMotor(const MotorConfig& cfg, const std::string& iface_name) {
    // gear_ratio=6.33 是 GO-M8010-6 的默认减速比
    auto motor = std::make_shared<UnitreeMotorNative>(
      cfg.name, static_cast<uint8_t>(cfg.id), 6.33);
    motor->setInterfaceName(iface_name);

    // 启动时设为 FOC 模式，零增益（可自由拖动，安全）
    motor->enable();
    motor->setFOCCommand(0.0, 0.0, 0.0, 0.0, 0.0);

    motors_[cfg.name] = motor;
    iface_motors_[iface_name].push_back(motor);
    unitree_motors_.push_back(motor);

    // 存储 offset 用于位置补偿
    motor_offsets_[cfg.name] = cfg.offset;

    RCLCPP_INFO(this->get_logger(),
      "添加电机: %s (id=%d, iface=%s, gear=6.33, offset=%.4f)",
      cfg.name.c_str(), cfg.id, iface_name.c_str(), cfg.offset);
  }

  // ========== 控制循环 ==========

  void controlLoop() {
    const auto now = this->now();

    // 频率统计
    loop_count_++;
    auto loop_elapsed = (now - last_freq_time_).seconds();
    if (loop_elapsed >= 10.0) {
      double actual_hz = static_cast<double>(loop_count_) / loop_elapsed;
      RCLCPP_INFO(this->get_logger(), "控制频率: %.1f Hz (目标: %.1f Hz)",
        actual_hz, target_control_freq_);
      loop_count_ = 0;
      last_freq_time_ = now;
    }

    applyCommandTimeout(now);

    // 多串口并行通信（每个串口独立，互不阻塞）
    std::vector<std::thread> threads;
    for (auto& [iface_name, motors] : iface_motors_) {
      threads.emplace_back([this, iface_name, &motors]() {
        auto iface = serial_network_->getInterface(iface_name);
        if (!iface || !iface->isOpen()) return;

        for (auto& motor : motors) {
          uint8_t cmd_buf[17];
          motor->getCommandPacket(cmd_buf);

          // 4Mbps 下 16 字节 ≈ 0.032ms，1ms timeout 已足够
          uint8_t fbk_buf[32];
          ssize_t fbk_len = iface->sendRecvAccumulate(
            cmd_buf, 17, fbk_buf, sizeof(fbk_buf), 0, 1);

          if (fbk_len >= 16) {
            motor->parseFeedback(fbk_buf, static_cast<size_t>(fbk_len));
          }

          const int64_t now_ns =
            std::chrono::steady_clock::now().time_since_epoch().count();
          motor->checkHeartbeat(100.0, now_ns);
        }
      });
    }

    // 等待所有串口线程完成（同一总线上的电机串行，不同总线并行）
    for (auto& t : threads) {
      if (t.joinable()) t.join();
    }

    publishStates(now);
  }

  void applyCommandTimeout(const rclcpp::Time& now) {
    for (auto& [name, time] : last_cmd_time_) {
      if ((now - time).seconds() > command_timeout_) {
        auto motor = std::dynamic_pointer_cast<UnitreeMotorNative>(
          motors_[name]);
        if (motor) {
          motor->setFOCCommand(0.0, 0.0, 0.0, 0.0, 0.0);
          motor->disable();
        }
      }
    }
  }

  void publishStates(const rclcpp::Time& now) {
    for (auto& [iface_name, motors] : iface_motors_) {
      for (auto& motor : motors) {
        auto msg = motor_control_ros2::msg::UnitreeGO8010State();
        msg.header.stamp = now;
        msg.joint_name = motor->getJointName();
        msg.motor_id = motor->getMotorId();
        msg.online = motor->isOnline();
        // 应用 offset 补偿
        double pos = motor->getOutputPosition();
        auto off_it = motor_offsets_.find(motor->getJointName());
        if (off_it != motor_offsets_.end()) {
          pos -= off_it->second;
        }

        msg.position = static_cast<float>(pos);
        msg.velocity = static_cast<float>(motor->getOutputVelocity());
        msg.torque = static_cast<float>(motor->getOutputTorque());
        msg.temperature = static_cast<int8_t>(motor->getTemperature());
        msg.error = static_cast<int8_t>(motor->getErrorCode());

        state_pub_->publish(msg);
      }
    }
  }

  // ========== ROS 命令回调 ==========

  void commandCallback(
    const motor_control_ros2::msg::UnitreeGO8010Command::SharedPtr msg) {
    // 按设备名查找
    auto it = motors_.find(msg->device);
    if (it != motors_.end()) {
      auto motor = std::dynamic_pointer_cast<UnitreeMotorNative>(it->second);
      if (motor) { applyCommand(motor, msg); }
      return;
    }
    // 按电机 ID 查找
    for (auto& [name, base] : motors_) {
      auto motor = std::dynamic_pointer_cast<UnitreeMotorNative>(base);
      if (motor && motor->getMotorId() == msg->id) {
        applyCommand(motor, msg);
        return;
      }
    }
  }

  void applyCommand(
    std::shared_ptr<UnitreeMotorNative> motor,
    const motor_control_ros2::msg::UnitreeGO8010Command::SharedPtr& msg) {
    switch (msg->mode) {
      case motor_control_ros2::msg::UnitreeGO8010Command::MODE_BRAKE:
        motor->disable();
        break;
      case motor_control_ros2::msg::UnitreeGO8010Command::MODE_CALIBRATE:
        motor->setCalibrateCommand();
        break;
      case motor_control_ros2::msg::UnitreeGO8010Command::MODE_FOC:
      default:
        motor->enable();
        motor->setFOCCommand(
          msg->position_target,
          msg->velocity_target,
          msg->kp,
          msg->kd,
          msg->torque_ff);
        break;
    }
    last_cmd_time_[motor->getJointName()] = this->now();
  }

  // ========== 成员变量 ==========

  SystemConfig config_;
  std::shared_ptr<hardware::SerialNetwork> serial_network_;

  std::map<std::string, std::shared_ptr<MotorBase>> motors_;
  std::map<std::string, std::vector<std::shared_ptr<UnitreeMotorNative>>> iface_motors_;
  std::vector<std::shared_ptr<UnitreeMotorNative>> unitree_motors_;
  std::map<std::string, double> motor_offsets_;
  std::map<std::string, rclcpp::Time> last_cmd_time_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Publisher<motor_control_ros2::msg::UnitreeGO8010State>::SharedPtr state_pub_;
  rclcpp::Subscription<motor_control_ros2::msg::UnitreeGO8010Command>::SharedPtr cmd_sub_;

  double target_control_freq_ = 200.0;
  double command_timeout_ = 0.5;
  int loop_count_ = 0;
  rclcpp::Time last_freq_time_;
};

} // namespace motor_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<motor_control::UnitreeMotorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
