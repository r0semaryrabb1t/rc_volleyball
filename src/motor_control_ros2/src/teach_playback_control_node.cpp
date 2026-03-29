#include "motor_control_ros2/teach_playback_control_node.hpp"

#include <nlohmann/json.hpp>
#include <rclcpp/executors.hpp>
#include <fstream>
#include <algorithm>
#include <numeric>
#include <chrono>
#include <thread>
#include <cstdlib>

using json = nlohmann::json;
using namespace std::chrono_literals;

#ifndef WORKSPACE_DIR
#define WORKSPACE_DIR "/home/rosemaryrabbit/USB2CAN_motor"
#endif

namespace motor_control {

// ── ProcessedTrajectory ────────────────────────────────────────

void ProcessedTrajectory::get_state(
    double time, std::array<double, 4>& out_pos,
    std::array<double, 4>& out_vel) const
{
    time = std::clamp(time, 0.0, duration);

    // 二分搜索找到 time 所在区间 [t[idx-1], t[idx])
    auto it = std::lower_bound(t.begin(), t.end(), time);
    int idx = static_cast<int>(std::distance(t.begin(), it));
    // 映射到段索引 i, 使得 t[i] <= time < t[i+1]
    int seg = idx - 1;
    if (seg < 0) seg = 0;
    if (seg >= n_frames - 1) seg = n_frames - 2;

    double dx = time - t[seg];

    for (int m = 0; m < 4; ++m) {
        const auto& s = spline[m];
        // S(x) = a + b*dx + c*dx^2 + d*dx^3
        out_pos[m] = s.a[seg] + dx * (s.b[seg] + dx * (s.c[seg] + dx * s.d[seg]));
        // S'(x) = b + 2*c*dx + 3*d*dx^2
        out_vel[m] = s.b[seg] + dx * (2.0 * s.c[seg] + 3.0 * s.d[seg] * dx);
    }
}

// ── 轨迹处理 ──────────────────────────────────────────────────

ProcessedTrajectory TeachPlaybackControlNode::process_frames(
    const std::vector<TeachFrame>& frames,
    int /* rate_hz */, int /* smooth_window */, double spike_thresh)
{
    int n_raw = static_cast<int>(frames.size());
    std::vector<double> raw_t(n_raw);
    std::array<std::vector<double>, 4> raw_pos;

    for (auto& rp : raw_pos) rp.resize(n_raw);
    for (int i = 0; i < n_raw; ++i) {
        raw_t[i] = frames[i].t;
        for (int m = 0; m < 4; ++m) {
            raw_pos[m][i] = frames[i].pos[m];
        }
    }

    double t_start = raw_t.front();
    double duration = raw_t.back() - t_start;

    // 毛刺去除
    for (int m = 0; m < 4; ++m) {
        std::vector<bool> spike(n_raw, false);
        for (int i = 1; i < n_raw; ++i) {
            double dt = raw_t[i] - raw_t[i - 1];
            if (dt <= 0.0) { spike[i] = true; continue; }
            if (std::abs(raw_pos[m][i] - raw_pos[m][i - 1]) / dt > spike_thresh) {
                spike[i] = true;
            }
        }
        // 线性插值替换
        std::vector<int> good;
        for (int i = 0; i < n_raw; ++i) {
            if (!spike[i]) good.push_back(i);
        }
        if (good.size() >= 2) {
            std::vector<double> patched(n_raw);
            int gi = 0;
            for (int i = 0; i < n_raw; ++i) {
                while (gi + 1 < (int)good.size() && raw_t[good[gi + 1]] < raw_t[i]) ++gi;
                int a = good[gi], b = good[std::min(gi + 1, (int)good.size() - 1)];
                if (a == b) {
                    patched[i] = raw_pos[m][a];
                } else {
                    double frac = (raw_t[i] - raw_t[a]) / (raw_t[b] - raw_t[a]);
                    patched[i] = raw_pos[m][a] + frac * (raw_pos[m][b] - raw_pos[m][a]);
                }
            }
            raw_pos[m] = patched;
        }
    }

    // 去重复时间戳 (保留最后一个)
    std::vector<int> keep;
    for (int i = 0; i < n_raw; ++i) {
        if (i + 1 < n_raw && raw_t[i + 1] <= raw_t[i]) continue;
        keep.push_back(i);
    }

    int n = static_cast<int>(keep.size());
    ProcessedTrajectory traj;
    traj.duration = duration;
    traj.n_frames = n;
    traj.t.resize(n);
    for (int m = 0; m < 4; ++m) traj.pos[m].resize(n);

    for (int i = 0; i < n; ++i) {
        traj.t[i] = raw_t[keep[i]] - t_start;  // 归零
        for (int m = 0; m < 4; ++m) {
            traj.pos[m][i] = raw_pos[m][keep[i]];
        }
    }

    // ── 非均匀三次自然样条 (natural cubic spline) ──
    // 直接在原始数据点上构建，充分利用每一帧
    // S_i(x) = a_i + b_i*(x-x_i) + c_i*(x-x_i)^2 + d_i*(x-x_i)^3

    for (int m = 0; m < 4; ++m) {
        int ns = n - 1;  // 段数
        auto& sp = traj.spline[m];
        sp.a.resize(ns);
        sp.b.resize(ns);
        sp.c.resize(ns);
        sp.d.resize(ns);

        const auto& x = traj.t;
        const auto& y = traj.pos[m];

        // 计算各段间距 h[i] = x[i+1] - x[i]
        std::vector<double> h(ns);
        for (int i = 0; i < ns; ++i) {
            h[i] = x[i + 1] - x[i];
        }

        // 构造三对角方程组求二阶导 c[] (natural: c[0] = c[n-1] = 0)
        // 内部节点 i = 1..n-2:
        // h[i-1]*c[i-1] + 2*(h[i-1]+h[i])*c[i] + h[i]*c[i+1]
        //   = 3*((y[i+1]-y[i])/h[i] - (y[i]-y[i-1])/h[i-1])
        std::vector<double> c_all(n, 0.0);  // c[0] = c[n-1] = 0

        if (n > 2) {
            int m_sz = n - 2;
            // 三对角: sub=h[i-1], diag=2*(h[i-1]+h[i]), sup=h[i]
            std::vector<double> sub(m_sz), diag(m_sz), sup(m_sz), rhs(m_sz);
            for (int i = 0; i < m_sz; ++i) {
                int k = i + 1;  // 原节点索引
                sub[i] = h[k - 1];
                diag[i] = 2.0 * (h[k - 1] + h[k]);
                sup[i] = h[k];
                rhs[i] = 3.0 * ((y[k + 1] - y[k]) / h[k] - (y[k] - y[k - 1]) / h[k - 1]);
            }

            // Thomas 算法 (追赶法)
            std::vector<double> cp(m_sz, 0.0);
            std::vector<double> dp(m_sz, 0.0);

            cp[0] = sup[0] / diag[0];
            dp[0] = rhs[0] / diag[0];
            for (int i = 1; i < m_sz; ++i) {
                double w = diag[i] - sub[i] * cp[i - 1];
                cp[i] = sup[i] / w;
                dp[i] = (rhs[i] - sub[i] * dp[i - 1]) / w;
            }

            c_all[m_sz] = dp[m_sz - 1];
            for (int i = m_sz - 2; i >= 0; --i) {
                c_all[i + 1] = dp[i] - cp[i] * c_all[i + 2];
            }
        }

        // 由 c 计算 a, b, d
        for (int i = 0; i < ns; ++i) {
            sp.a[i] = y[i];
            sp.b[i] = (y[i + 1] - y[i]) / h[i] - h[i] * (2.0 * c_all[i] + c_all[i + 1]) / 3.0;
            sp.c[i] = c_all[i];
            sp.d[i] = (c_all[i + 1] - c_all[i]) / (3.0 * h[i]);
        }
    }

    RCLCPP_INFO(get_logger(), "轨迹处理: %d帧(去毛刺后%d点), %.2fs, 三次样条插值",
                n_raw, n, duration);
    return traj;
}

// ── JSON 加载 ─────────────────────────────────────────────────

void TeachPlaybackControlNode::load_trajectory() {
    RCLCPP_INFO(get_logger(), "加载示教数据: %s", teach_file_.c_str());

    std::ifstream ifs(teach_file_);
    if (!ifs.is_open()) {
        RCLCPP_ERROR(get_logger(), "无法打开文件: %s", teach_file_.c_str());
        throw std::runtime_error("teach file not found");
    }

    json raw = json::parse(ifs);
    json data;

    if (raw.contains("recordings")) {
        auto& recs = raw["recordings"];
        int ri = rec_index_;
        if (ri < 0) ri = static_cast<int>(recs.size()) + ri;
        ri = std::clamp(ri, 0, static_cast<int>(recs.size()) - 1);
        data = recs[ri];
        RCLCPP_INFO(get_logger(), "共 %zu 条录制, 使用第 %d 条 (%.1fs)",
                     recs.size(), ri, data.value("duration", 0.0));
    } else {
        data = raw;
    }

    // tau 优先从 JSON 读取
    if (data.contains("tau_inner") && data["tau_inner"].get<double>() != 0.0) {
        tau_inner_ = data["tau_inner"].get<double>();
    }
    if (data.contains("tau_outer") && data["tau_outer"].get<double>() != 0.0) {
        tau_outer_ = data["tau_outer"].get<double>();
    }
    RCLCPP_INFO(get_logger(), "重力补偿: 大臂=%.1fNm 小臂=%.1fNm",
                tau_inner_, tau_outer_);

    // 解析帧数据
    auto& jframes = data["frames"];
    std::vector<TeachFrame> frames;
    frames.reserve(jframes.size());
    for (auto& jf : jframes) {
        TeachFrame f;
        f.t = jf["t"].get<double>();
        f.pos = {jf["m0"].get<double>(), jf["m1"].get<double>(),
                 jf["m2"].get<double>(), jf["m3"].get<double>()};
        f.vel = {jf.value("v0", 0.0), jf.value("v1", 0.0),
                 jf.value("v2", 0.0), jf.value("v3", 0.0)};
        frames.push_back(f);
    }

    traj_ = process_frames(frames, smooth_factor_, spike_thresh_);

    // 缓存起点/终点
    traj_.get_state(0.0, start_pos_, end_pos_);  // 临时存 end
    std::array<double, 4> tmp_vel;
    traj_.get_state(traj_.duration, end_pos_, tmp_vel);
    playback_duration_ = traj_.duration / speed_;

    RCLCPP_INFO(get_logger(), "轨迹: %.2fs × %.1fx = %.2fs",
                traj_.duration, speed_, playback_duration_);
}

// ── 零位校准 ──────────────────────────────────────────────────

bool TeachPlaybackControlNode::calibrate_zero() {
    RCLCPP_INFO(get_logger(), "执行零位校准 (调用校准脚本)...");

    std::string script = std::string(WORKSPACE_DIR) +
        "/src/ROS_test/calibrate_zero_position.py";

    // 检查脚本是否存在
    std::ifstream check(script);
    if (!check.is_open()) {
        RCLCPP_WARN(get_logger(), "校准脚本不存在: %s，跳过校准", script.c_str());
        return false;
    }
    check.close();

    std::string cmd = "cd " + std::string(WORKSPACE_DIR) +
        "/src/ROS_test && python3 calibrate_zero_position.py";
    int ret = std::system(cmd.c_str());
    if (ret != 0) {
        RCLCPP_WARN(get_logger(), "零位校准返回 %d", ret);
        return false;
    }
    RCLCPP_INFO(get_logger(), "零位校准完成");
    return true;
}

// ── 构造函数 ──────────────────────────────────────────────────

TeachPlaybackControlNode::TeachPlaybackControlNode()
    : Node("teach_playback_control_node"),
      state_(State::IDLE),
      trigger_pending_(false),
      phase_start_(0.0),
      playback_duration_(0.0)
{
    // 声明参数
    this->declare_parameter("teach_file",
        std::string(WORKSPACE_DIR "/teach_data.json"));
    this->declare_parameter("rec_index", -1);
    this->declare_parameter("kp", 1.0);
    this->declare_parameter("kd", 0.15);
    this->declare_parameter("kp_hold", 0.8);
    this->declare_parameter("kd_hold", 0.15);
    this->declare_parameter("speed", 1.0);
    this->declare_parameter("smooth_factor", 0.001);
    this->declare_parameter("spike_thresh", 20.0);
    this->declare_parameter("tau_inner", 3.1);
    this->declare_parameter("tau_outer", 1.5);
    this->declare_parameter("gravity_offset_inner", -M_PI / 2.0);
    this->declare_parameter("gravity_offset_outer", -M_PI / 2.0);
    this->declare_parameter("goto_time", 5.0);
    this->declare_parameter("hold_time", 1.0);
    this->declare_parameter("rate_hz", 200);
    this->declare_parameter("trigger_source", std::string("joy"));
    this->declare_parameter("joy_button", 5);

    // 读取参数
    teach_file_ = this->get_parameter("teach_file").as_string();
    rec_index_ = this->get_parameter("rec_index").as_int();
    kp_ = this->get_parameter("kp").as_double();
    kd_ = this->get_parameter("kd").as_double();
    kp_hold_ = this->get_parameter("kp_hold").as_double();
    kd_hold_ = this->get_parameter("kd_hold").as_double();
    speed_ = this->get_parameter("speed").as_double();
    smooth_factor_ = this->get_parameter("smooth_factor").as_double();
    spike_thresh_ = this->get_parameter("spike_thresh").as_double();
    tau_inner_ = this->get_parameter("tau_inner").as_double();
    tau_outer_ = this->get_parameter("tau_outer").as_double();
    gravity_offset_inner_ = this->get_parameter("gravity_offset_inner").as_double();
    gravity_offset_outer_ = this->get_parameter("gravity_offset_outer").as_double();
    goto_time_ = this->get_parameter("goto_time").as_double();
    hold_time_ = this->get_parameter("hold_time").as_double();
    rate_hz_ = this->get_parameter("rate_hz").as_int();
    trigger_source_ = this->get_parameter("trigger_source").as_string();
    joy_button_ = this->get_parameter("joy_button").as_int();

    initial_pos_.fill(0.0);
    start_pos_.fill(0.0);
    end_pos_.fill(0.0);

    // ROS 接口
    cmd_pub_ = this->create_publisher<motor_control_ros2::msg::UnitreeGO8010Command>(
        "/unitree_go8010_command", 10);
    state_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/strike_busy", 10);
    motor_sub_ = this->create_subscription<motor_control_ros2::msg::UnitreeGO8010State>(
        "/unitree_go8010_states", 10,
        std::bind(&TeachPlaybackControlNode::state_callback, this, std::placeholders::_1));

    if (trigger_source_ == "joy") {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&TeachPlaybackControlNode::joy_callback, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "触发源: 手柄 button %d", joy_button_);
    } else {
        ir_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/ir_trigger", 10,
            std::bind(&TeachPlaybackControlNode::ir_callback, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "触发源: 红外传感器 /ir_trigger");
    }

    RCLCPP_INFO(get_logger(), "节点创建完成");
}

// ── 回调 ──────────────────────────────────────────────────────

void TeachPlaybackControlNode::state_callback(
    const motor_control_ros2::msg::UnitreeGO8010State::SharedPtr msg)
{
    positions_[msg->motor_id] = msg->position;
    velocities_[msg->motor_id] = msg->velocity;
    temperatures_[msg->motor_id] = msg->temperature;
    errors_[msg->motor_id] = msg->error;
    last_update_[msg->motor_id] =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
}

void TeachPlaybackControlNode::joy_callback(
    const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (state_ == State::IDLE &&
        static_cast<int>(msg->buttons.size()) > joy_button_ &&
        msg->buttons[joy_button_]) {
        trigger_pending_ = true;
        RCLCPP_INFO(get_logger(), "收到手柄触发信号");
    }
}

void TeachPlaybackControlNode::ir_callback(
    const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data && state_ == State::IDLE) {
        trigger_pending_ = true;
        RCLCPP_INFO(get_logger(), "收到红外触发信号");
    }
}

// ── 控制辅助 ──────────────────────────────────────────────────

void TeachPlaybackControlNode::send_cmd(
    int id, double torque_ff, double kp, double kd,
    double pos_target, double vel_target)
{
    auto cmd = motor_control_ros2::msg::UnitreeGO8010Command();
    cmd.id = id;
    cmd.mode = 1;
    cmd.position_target = pos_target;
    cmd.velocity_target = vel_target;
    cmd.kp = kp;
    cmd.kd = kd;
    cmd.torque_ff = torque_ff;
    cmd_pub_->publish(cmd);
}

void TeachPlaybackControlNode::brake_all() {
    for (int mid : ALL_IDS) {
        auto cmd = motor_control_ros2::msg::UnitreeGO8010Command();
        cmd.id = mid;
        cmd.mode = 0;
        cmd_pub_->publish(cmd);
    }
}

double TeachPlaybackControlNode::smoothstep(double t) {
    t = std::clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

double TeachPlaybackControlNode::gravity_torque(int mid, double theta1, double theta2) {
    if (is_inner(mid)) {
        return tau_inner_ * std::cos(theta1 + gravity_offset_inner_) / GEAR_RATIO;
    } else {
        return -tau_outer_ * std::cos(theta1 + theta2 + gravity_offset_outer_) / GEAR_RATIO;
    }
}

std::pair<double, double> TeachPlaybackControlNode::get_theta_pair() {
    double t1 = (positions_.count(0) ? positions_[0] : 0.0) +
                (positions_.count(2) ? positions_[2] : 0.0);
    double t2 = (positions_.count(1) ? positions_[1] : 0.0) +
                (positions_.count(3) ? positions_[3] : 0.0);
    return {t1 / 2.0, t2 / 2.0};
}

bool TeachPlaybackControlNode::check_safety() {
    for (int mid : ALL_IDS) {
        auto it = temperatures_.find(mid);
        if (it != temperatures_.end() && it->second >= TEMP_CRITICAL) {
            RCLCPP_ERROR(get_logger(), "M%d 温度=%.0fC 超限!", mid, it->second);
            return false;
        }
        auto eit = errors_.find(mid);
        if (eit != errors_.end() && eit->second >= 1 && eit->second <= 4) {
            RCLCPP_ERROR(get_logger(), "M%d 错误码=%d", mid, eit->second);
            return false;
        }
    }
    return true;
}

bool TeachPlaybackControlNode::motors_online() {
    return static_cast<int>(positions_.size()) >= 4;
}

// ── 主循环 ────────────────────────────────────────────────────

static double now_sec() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

void TeachPlaybackControlNode::run() {
    // 加载轨迹
    load_trajectory();

    state_ = State::IDLE;
    RCLCPP_INFO(get_logger(), "进入 IDLE, 等待触发 (零位校准已跳过, 请手动校准)...");

    // 创建 executor 用于 spin
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(this->shared_from_this());

    double dt = 1.0 / rate_hz_;
    int loop_count = 0;

    while (rclcpp::ok()) {
        double t_now = now_sec();

        // 排空消息队列
        executor.spin_some(std::chrono::milliseconds(0));

        if (!motors_online()) {
            if (loop_count % rate_hz_ == 0) {
                RCLCPP_WARN(get_logger(), "等待电机上线...");
            }
            loop_count++;
            std::this_thread::sleep_for(
                std::chrono::duration<double>(dt));
            continue;
        }

        // 安全检查 (每秒)
        if (loop_count % rate_hz_ == 0 && loop_count > 0) {
            if (!check_safety()) {
                brake_all();
                state_ = State::IDLE;
                RCLCPP_ERROR(get_logger(), "安全检查失败, 刹车!");
                std::this_thread::sleep_for(1s);
                continue;
            }
        }

        auto [theta1, theta2] = get_theta_pair();

        // ── IDLE ──
        if (state_ == State::IDLE) {
            {
                auto msg = std_msgs::msg::Bool();
                msg.data = false;
                state_pub_->publish(msg);
            }

            for (int mid : ALL_IDS) {
                double tau_g = gravity_torque(mid, theta1, theta2);
                send_cmd(mid, tau_g, kp_hold_, kd_hold_, 0.0);
            }

            if (trigger_pending_) {
                trigger_pending_ = false;
                phase_start_ = now_sec();
                for (int m = 0; m < 4; ++m) {
                    initial_pos_[m] = positions_.count(m) ? positions_[m] : 0.0;
                }
                {
                    auto msg = std_msgs::msg::Bool();
                    msg.data = true;
                    state_pub_->publish(msg);
                }

                // 起点接近当前位置 → 跳过 GOTO_START 直接回放
                double max_diff = 0.0;
                for (int m = 0; m < 4; ++m) {
                    max_diff = std::max(max_diff,
                        std::abs(initial_pos_[m] - start_pos_[m]));
                }
                if (max_diff < 0.05) {  // < 3° 视为已在起点
                    state_ = State::PLAYBACK;
                    RCLCPP_INFO(get_logger(), "触发! 起点近零 → 直接 PLAYBACK");
                } else {
                    state_ = State::GOTO_START;
                    RCLCPP_INFO(get_logger(), "触发! → GOTO_START (偏差 %.1f°)",
                                max_diff * 180.0 / M_PI);
                }
            }
        }
        // ── GOTO_START ──
        else if (state_ == State::GOTO_START) {
            double elapsed = now_sec() - phase_start_;
            double frac = (goto_time_ > 0) ? smoothstep(elapsed / goto_time_) : 1.0;

            for (int mid : ALL_IDS) {
                double p0 = initial_pos_[mid];
                double pf = start_pos_[mid];
                double p_des = p0 + (pf - p0) * frac;
                double tau_g = gravity_torque(mid, theta1, theta2);
                send_cmd(mid, tau_g, kp_, kd_, p_des);
            }

            if (elapsed >= goto_time_) {
                state_ = State::PLAYBACK;
                phase_start_ = now_sec();
                RCLCPP_INFO(get_logger(), "到达起点 → PLAYBACK");
            }
        }
        // ── PLAYBACK ──
        else if (state_ == State::PLAYBACK) {
            double elapsed = now_sec() - phase_start_;
            double t_traj = elapsed * speed_;

            if (t_traj >= traj_.duration) {
                state_ = State::HOLD;
                phase_start_ = now_sec();
                RCLCPP_INFO(get_logger(), "回放完成 → HOLD");
            } else {
                std::array<double, 4> pos_d, vel_d;
                traj_.get_state(t_traj, pos_d, vel_d);
                for (int mid : ALL_IDS) {
                    double tau_g = gravity_torque(mid, theta1, theta2);
                    send_cmd(mid, tau_g, kp_, kd_,
                             pos_d[mid], vel_d[mid] * speed_);
                }
            }
        }
        // ── HOLD ──
        else if (state_ == State::HOLD) {
            double elapsed = now_sec() - phase_start_;

            for (int mid : ALL_IDS) {
                double tau_g = gravity_torque(mid, theta1, theta2);
                send_cmd(mid, tau_g, kp_hold_, kd_hold_, end_pos_[mid]);
            }

            if (elapsed >= hold_time_) {
                state_ = State::RETURN_ZERO;
                phase_start_ = now_sec();
                RCLCPP_INFO(get_logger(), "保持结束 → RETURN (反向回放)");
            }
        }
        // ── RETURN (反向回放录制轨迹) ──
        else if (state_ == State::RETURN_ZERO) {
            double elapsed = now_sec() - phase_start_;
            double t_reverse = traj_.duration - elapsed * speed_;

            if (t_reverse <= 0.0) {
                // 倒放完成，已回到轨迹起点
                state_ = State::IDLE;
                RCLCPP_INFO(get_logger(), "倒放完成 → IDLE, 等待下次触发");
            } else {
                std::array<double, 4> pos_d, vel_d;
                traj_.get_state(t_reverse, pos_d, vel_d);
                for (int mid : ALL_IDS) {
                    double tau_g = gravity_torque(mid, theta1, theta2);
                    // 反向回放：位置正常，速度取反
                    send_cmd(mid, tau_g, kp_, kd_,
                             pos_d[mid], -vel_d[mid] * speed_);
                }
            }
        }

        // 状态汇报 (每 2 秒)
        loop_count++;
        if (loop_count % (rate_hz_ * 2) == 0) {
            const char* state_str = "?";
            switch (state_) {
                case State::CALIBRATING: state_str = "CALIBRATING"; break;
                case State::IDLE:        state_str = "IDLE"; break;
                case State::GOTO_START:  state_str = "GOTO_START"; break;
                case State::PLAYBACK:    state_str = "PLAYBACK"; break;
                case State::HOLD:        state_str = "HOLD"; break;
                case State::RETURN_ZERO: state_str = "RETURN"; break;
            }
            RCLCPP_INFO(get_logger(), "[%s] M0:%+.1f° M1:%+.1f° M2:%+.1f° M3:%+.1f°",
                        state_str,
                        positions_.count(0) ? positions_[0] * 180.0 / M_PI : 0.0,
                        positions_.count(1) ? positions_[1] * 180.0 / M_PI : 0.0,
                        positions_.count(2) ? positions_[2] * 180.0 / M_PI : 0.0,
                        positions_.count(3) ? positions_[3] * 180.0 / M_PI : 0.0);
        }

        double sleep_time = dt - (now_sec() - t_now);
        if (sleep_time > 0) {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(sleep_time));
        }
    }

    brake_all();
}

}  // namespace motor_control

// ── main ──────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<motor_control::TeachPlaybackControlNode>();

    try {
        node->run();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "异常退出: %s", e.what());
    }

    node->brake_all();
    rclcpp::shutdown();
    return 0;
}
