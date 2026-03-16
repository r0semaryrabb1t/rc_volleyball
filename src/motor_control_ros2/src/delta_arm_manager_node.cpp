#include "motor_control_ros2/delta_arm_manager_node.hpp"
#include <yaml-cpp/yaml.h>

DeltaArmManager::DeltaArmManager()
    : Node("delta_arm_manager"),
      downward_torque_(-1.0),
      landing_timeout_(5.0),
      landing_angle_threshold_(0.02),
      landing_stable_duration_(0.5),
      kp_(0.50),
      kd_(0.10),
      max_velocity_(2.0),
      control_frequency_(200.0),
      position_tolerance_(0.05),
      state_(State::INIT),
      landing_stability_started_(false),
      ready_published_(false)
{
  current_positions_.fill(0.0);
  current_velocities_.fill(0.0);
  motors_online_.fill(false);
  target_positions_.fill(0.0);
  planned_positions_.fill(0.0);
  motor_names_ = {"arm_motor_1", "arm_motor_2", "arm_motor_3"};
  motor_ids_   = {0, 1, 2};

  // 加载配置文件
  try {
    std::string config_file =
        ament_index_cpp::get_package_share_directory("motor_control_ros2") +
        "/config/arm_config.yaml";
    loadConfig(config_file);
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(), "配置文件加载失败（使用默认值）: %s", e.what());
  }

  // 订阅 ArmTarget 命令
  target_sub_ = this->create_subscription<motor_control_ros2::msg::ArmTarget>(
      "/delta_arm/target", 10,
      std::bind(&DeltaArmManager::armTargetCallback, this, std::placeholders::_1));

  // 订阅 GO8010 电机状态
  state_sub_ = this->create_subscription<motor_control_ros2::msg::UnitreeGO8010State>(
      "unitree_go8010_states", 10,
      std::bind(&DeltaArmManager::motorStateCallback, this, std::placeholders::_1));

  // 发布 GO8010 电机命令
  cmd_pub_ = this->create_publisher<motor_control_ros2::msg::UnitreeGO8010Command>(
      "unitree_go8010_command", 10);

  // 发布 ready 信号
  ready_pub_ = this->create_publisher<std_msgs::msg::String>("/delta_arm/ready", 10);

  // 创建控制定时器
  auto period = std::chrono::duration<double>(1.0 / control_frequency_);
  control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&DeltaArmManager::controlLoop, this));

  init_start_time_ = this->now();
  RCLCPP_INFO(this->get_logger(),
      "delta_arm_manager 启动，控制频率 %.1f Hz，电机: [%s, %s, %s]",
      control_frequency_,
      motor_names_[0].c_str(), motor_names_[1].c_str(), motor_names_[2].c_str());
  RCLCPP_INFO(this->get_logger(), "状态: INIT → 进入软着陆流程");
}

// ========== 配置加载 ==========

void DeltaArmManager::loadConfig(const std::string& config_file)
{
  YAML::Node cfg = YAML::LoadFile(config_file);

  if (cfg["control_frequency"]) {
    control_frequency_ = cfg["control_frequency"].as<double>();
  }

  auto init = cfg["initialization"];
  if (init) {
    if (init["downward_torque"])        downward_torque_         = init["downward_torque"].as<double>();
    if (init["landing_timeout"])        landing_timeout_          = init["landing_timeout"].as<double>();
    if (init["landing_angle_threshold"]) landing_angle_threshold_ = init["landing_angle_threshold"].as<double>();
    if (init["landing_stable_duration"]) landing_stable_duration_ = init["landing_stable_duration"].as<double>();
  }

  auto pd = cfg["pd"];
  if (pd) {
    if (pd["kp"]) kp_ = pd["kp"].as<double>();
    if (pd["kd"]) kd_ = pd["kd"].as<double>();
  }

  auto mp = cfg["motion_profile"];
  if (mp && mp["max_velocity"]) {
    max_velocity_ = mp["max_velocity"].as<double>();
  }

  auto motors = cfg["motors"];
  if (motors && motors.IsSequence()) {
    for (size_t i = 0; i < 3 && i < motors.size(); ++i) {
      if (motors[i]["name"]) {
        motor_names_[i] = motors[i]["name"].as<std::string>();
      }
      if (motors[i]["id"]) {
        motor_ids_[i] = motors[i]["id"].as<uint8_t>();
      }
    }
  }

  if (cfg["position_tolerance"]) {
    position_tolerance_ = cfg["position_tolerance"].as<double>();
  }

  RCLCPP_INFO(this->get_logger(),
      "配置加载完成: kp=%.2f kd=%.2f max_vel=%.2f landing_torque=%.2f landing_timeout=%.1fs",
      kp_, kd_, max_velocity_, downward_torque_, landing_timeout_);
}

// ========== 回调 ==========

void DeltaArmManager::armTargetCallback(
    const motor_control_ros2::msg::ArmTarget::SharedPtr msg)
{
  if (!msg->execute) {
    return;
  }

  if (state_ != State::READY) {
    RCLCPP_WARN(this->get_logger(),
        "收到目标命令，但当前状态为 %s，拒绝执行（仅 READY 状态接受命令）",
        state_ == State::INIT ? "INIT" :
        state_ == State::SOFT_LANDING ? "SOFT_LANDING" : "EXECUTE");
    return;
  }

  for (size_t i = 0; i < 3; ++i) {
    target_positions_[i] = msg->target_angles[i];
    planned_positions_[i] = current_positions_[i];  // 从当前位置出发
  }
  state_ = State::EXECUTE;
  RCLCPP_INFO(this->get_logger(),
      "READY → EXECUTE: 目标角度 [%.3f, %.3f, %.3f] rad",
      target_positions_[0], target_positions_[1], target_positions_[2]);
}

void DeltaArmManager::motorStateCallback(
    const motor_control_ros2::msg::UnitreeGO8010State::SharedPtr msg)
{
  for (size_t i = 0; i < 3; ++i) {
    if (msg->joint_name == motor_names_[i]) {
      current_positions_[i]  = static_cast<double>(msg->position);
      current_velocities_[i] = static_cast<double>(msg->velocity);
      motors_online_[i]      = msg->online;
      break;
    }
  }
}

// ========== 主控制循环 ==========

void DeltaArmManager::controlLoop()
{
  const double dt = 1.0 / control_frequency_;

  switch (state_) {

    case State::INIT: {
      // 进入软着陆阶段
      state_ = State::SOFT_LANDING;
      landing_stability_started_ = false;
      init_start_time_ = this->now();
      RCLCPP_INFO(this->get_logger(), "INIT → SOFT_LANDING: 施加向下力矩 %.2f Nm", downward_torque_);
      break;
    }

    case State::SOFT_LANDING: {
      // 检查超时
      double elapsed = (this->now() - init_start_time_).seconds();
      if (elapsed > landing_timeout_) {
        RCLCPP_WARN(this->get_logger(),
            "软着陆超时 (%.1f s)，强制进入 READY 状态", landing_timeout_);
        state_ = State::READY;
        publishReady();
        break;
      }

      // 向所有电机施加向下力矩（力矩控制，kp=kd=0）
      for (size_t i = 0; i < 3; ++i) {
        publishCommand(i, 0.0, 0.0, downward_torque_, 0.0, 0.0);
      }

      // 检查着陆稳定条件
      if (allMotorsLanded()) {
        if (!landing_stability_started_) {
          landing_stable_since_ = this->now();
          landing_stability_started_ = true;
        }
        double stable_time = (this->now() - landing_stable_since_).seconds();
        if (stable_time >= landing_stable_duration_) {
          RCLCPP_INFO(this->get_logger(),
              "软着陆完成（稳定 %.2f s），SOFT_LANDING → READY", stable_time);
          state_ = State::READY;
          publishReady();
        }
      } else {
        landing_stability_started_ = false;
      }
      break;
    }

    case State::READY: {
      // 发布 READY 信号（每秒一次，防止订阅者错过）
      if (!ready_published_) {
        publishReady();
      }
      // 在 READY 阶段保持当前位置（轻 PD 控制）
      for (size_t i = 0; i < 3; ++i) {
        publishCommand(i, current_positions_[i], 0.0, 0.0, kp_, kd_);
      }
      break;
    }

    case State::EXECUTE: {
      // 梯形速度规划：pos_next = pos_now + clamp(err, ±max_velocity*dt)
      double max_step = max_velocity_ * dt;

      for (size_t i = 0; i < 3; ++i) {
        double err = target_positions_[i] - planned_positions_[i];
        double step = std::clamp(err, -max_step, max_step);
        planned_positions_[i] += step;

        publishCommand(i, planned_positions_[i], 0.0, 0.0, kp_, kd_);
      }

      if (allMotorsReached()) {
        RCLCPP_INFO(this->get_logger(), "目标到达，EXECUTE → READY");
        state_ = State::READY;
        ready_published_ = false;  // 重新触发一次 READY 发布
      }
      break;
    }
  }
}

// ========== 辅助函数 ==========

bool DeltaArmManager::allMotorsLanded() const
{
  // 当三电机绝对角度均 < landing_angle_threshold_ 时视为已着陆（处于底部）
  for (size_t i = 0; i < 3; ++i) {
    if (std::abs(current_positions_[i]) >= landing_angle_threshold_) {
      return false;  // 至少有一路电机尚未到达底部
    }
  }
  return true;
}

bool DeltaArmManager::allMotorsReached() const
{
  for (size_t i = 0; i < 3; ++i) {
    if (std::abs(target_positions_[i] - current_positions_[i]) > position_tolerance_) {
      return false;
    }
  }
  return true;
}

void DeltaArmManager::publishReady()
{
  auto msg = std_msgs::msg::String();
  msg.data = "READY";
  ready_pub_->publish(msg);
  ready_published_ = true;
  RCLCPP_INFO(this->get_logger(), "发布 /delta_arm/ready: READY");
}

void DeltaArmManager::publishCommand(size_t idx,
    double pos_des, double vel_des, double torque_ff,
    double kp, double kd)
{
  auto cmd = motor_control_ros2::msg::UnitreeGO8010Command();
  cmd.header.stamp = this->now();
  cmd.id = motor_ids_[idx];  // motor_control_node 通过 id 路由命令
  cmd.mode = motor_control_ros2::msg::UnitreeGO8010Command::MODE_FOC;
  cmd.position_target = pos_des;
  cmd.velocity_target = vel_des;
  cmd.torque_ff       = static_cast<float>(torque_ff);
  cmd.kp              = static_cast<float>(kp);
  cmd.kd              = static_cast<float>(kd);
  cmd_pub_->publish(cmd);
}

// ========== main ==========

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DeltaArmManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
