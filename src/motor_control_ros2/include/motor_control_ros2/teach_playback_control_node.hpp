#ifndef MOTOR_CONTROL_ROS2__TEACH_PLAYBACK_CONTROL_NODE_HPP_
#define MOTOR_CONTROL_ROS2__TEACH_PLAYBACK_CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include "motor_control_ros2/msg/unitree_go8010_command.hpp"
#include "motor_control_ros2/msg/unitree_go8010_state.hpp"

#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <array>

namespace motor_control {

/**
 * @brief 示教回放控制节点
 *
 * 状态机: CALIBRATING → IDLE → GOTO_START → PLAYBACK → HOLD → RETURN → IDLE
 *
 * 启动后自动零位校准，IDLE 保持零位 + 重力补偿。
 * 收到触发信号后: 移至起点 → 回放轨迹 → 短暂保持 → 回零。
 *
 * 触发源:
 *   - joy: 手柄 RB 键 (button 5)
 *   - ir:  红外传感器 /ir_trigger (std_msgs/Bool)
 */

// 电机常量
constexpr double GEAR_RATIO = 6.33;
constexpr int NUM_MOTORS = 4;
constexpr std::array<int, 4> ALL_IDS = {0, 1, 2, 3};
constexpr double NODE_TIMEOUT = 8.0;
constexpr double TEMP_CRITICAL = 85.0;

// 状态机枚举
enum class State {
    CALIBRATING,
    IDLE,
    GOTO_START,
    PLAYBACK,
    HOLD,
    RETURN_ZERO
};

// 单帧数据
struct TeachFrame {
    double t;
    std::array<double, 4> pos;  // m0, m1, m2, m3
    std::array<double, 4> vel;  // v0, v1, v2, v3
};

// 处理后的轨迹
struct ProcessedTrajectory {
    double duration;
    int n_frames;
    std::vector<double> t;                    // 均匀时间轴
    std::array<std::vector<double>, 4> pos;   // 平滑后位置
    std::array<std::vector<double>, 4> vel;   // 平滑后速度

    // 插值获取 t 时刻状态
    void get_state(double time, std::array<double, 4>& out_pos,
                   std::array<double, 4>& out_vel) const;
};

class TeachPlaybackControlNode : public rclcpp::Node {
public:
    TeachPlaybackControlNode();

    /// 运行阻塞式主循环（校准 → 控制）
    void run();

    /// 刹车所有电机
    void brake_all();

private:
    // ── 初始化 ──
    void load_trajectory();
    bool calibrate_zero();

    // ── 轨迹处理 ──
    ProcessedTrajectory process_frames(
        const std::vector<TeachFrame>& frames,
        int rate_hz, int smooth_window, double spike_thresh);

    // ── 回调 ──
    void state_callback(
        const motor_control_ros2::msg::UnitreeGO8010State::SharedPtr msg);
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void ir_callback(const std_msgs::msg::Bool::SharedPtr msg);

    // ── 控制辅助 ──
    void send_cmd(int id, double torque_ff, double kp, double kd,
                  double pos_target, double vel_target = 0.0);
    double smoothstep(double t);
    double gravity_torque(int mid, double theta1, double theta2);
    std::pair<double, double> get_theta_pair();
    bool check_safety();
    bool motors_online();

    // ── ROS 接口 ──
    rclcpp::Publisher<motor_control_ros2::msg::UnitreeGO8010Command>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_pub_;
    rclcpp::Subscription<motor_control_ros2::msg::UnitreeGO8010State>::SharedPtr motor_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ir_sub_;

    // ── 参数 ──
    std::string teach_file_;
    int rec_index_;
    double kp_, kd_;
    double kp_hold_, kd_hold_;
    double speed_;
    int smooth_window_;
    double spike_thresh_;
    double tau_inner_, tau_outer_;
    double goto_time_, return_time_, hold_time_;
    int rate_hz_;
    std::string trigger_source_;
    int joy_button_;

    // ── 电机状态 ──
    std::map<int, double> positions_;
    std::map<int, double> velocities_;
    std::map<int, double> temperatures_;
    std::map<int, int> errors_;
    std::map<int, double> last_update_;

    // ── 状态机 ──
    State state_;
    bool trigger_pending_;
    double phase_start_;
    std::array<double, 4> initial_pos_;

    // ── 轨迹 ──
    ProcessedTrajectory traj_;
    std::array<double, 4> start_pos_;
    std::array<double, 4> end_pos_;
    double playback_duration_;

    // ── 内联工具 ──
    static bool is_inner(int mid) { return mid == 0 || mid == 2; }
};

}  // namespace motor_control

#endif  // MOTOR_CONTROL_ROS2__TEACH_PLAYBACK_CONTROL_NODE_HPP_
