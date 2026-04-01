#ifndef MOTOR_CONTROL_ROS2__PARALLEL_ARM_KINEMATICS_HPP_
#define MOTOR_CONTROL_ROS2__PARALLEL_ARM_KINEMATICS_HPP_

#include <array>
#include <cmath>
#include <optional>
#include <vector>

namespace motor_control {

/**
 * @brief 双臂并联平面RR机构运动学解算器
 *
 * 机构描述：
 *   两条完全相同的平面 RR 串联臂左右对称安装，末端刚性连接击球板。
 *   肩关节中心距 = 击球板连接点间距 = D。
 *   系统自由度 = 2（击球板中心高度 H、俯仰角 φ）。
 *
 * 单臂结构：
 *   - 大臂电机固定于基座，直驱大臂绕肩关节转动
 *   - 小臂电机固定在大臂上，与大臂电机共轴，随大臂一同转动
 *   - 小臂电机通过四连杆机构驱动小臂绕肘关节转动
 *
 * 四连杆参数：
 *   曲柄 r = 30 mm（小臂电机轴上）
 *   连杆 l = 200 mm
 *   小臂上连杆铰接点距肘关节 d = 20 mm
 *
 * 连杆长度：
 *   大臂 L1 = 200 mm，小臂 L2 = 221 mm
 *
 * 角度定义（全局坐标系：以大臂电机轴心为原点，X 水平向右，Y 垂直向上）：
 *   q1 = 大臂从垂直向下顺时针旋转的角度（零点 = 垂直向下，即 -Y 方向，顺时针为正）
 *   q2 = 小臂相对大臂的夹角（逆时针为正，零点 = 小臂沿大臂延伸方向）
 *   θ2 = 小臂电机曲柄在大臂坐标系中，从垂直向下逆时针旋转的角度
 *
 * 注意：小臂电机轴心与大臂电机轴心重合（共轴安装在肩关节处）。
 *       四连杆约束在大臂局部坐标系中建模，θ2 与 q2 的映射不依赖 q1。
 *
 * 电机限位：大臂 0–110°，小臂 0–80°
 */
class ParallelArmKinematics {
public:
    // ===================== 机构几何常量 =====================
    struct Params {
        double L1 = 0.200;          // 大臂长度 (m)
        double L2 = 0.221;          // 小臂长度 (m)
        double r  = 0.030;          // 曲柄长度 (m)
        double l  = 0.200;          // 连杆长度 (m)
        double d  = 0.020;          // 小臂上连杆铰接点距肘关节 (m)
        double D  = 0.3506;         // 肩关节中心距 = 击球板连接点间距 (m)

        // 电机限位 (rad)
        double q1_min = 0.0;
        double q1_max = 110.0 * M_PI / 180.0;
        double theta2_min = 0.0;
        double theta2_max = 135.0 * M_PI / 180.0;
    };

    // ===================== 结果类型 =====================
    struct EndPoint {
        double x;
        double y;
    };

    struct SingleArmSolution {
        double q1;      // 大臂角 (rad)
        double theta2;  // 小臂电机角 (rad)
        double q2;      // 小臂相对角 (rad)
    };

    struct DualArmSolution {
        SingleArmSolution left;
        SingleArmSolution right;
    };

    struct TrajectoryPoint {
        double time;
        double position;
        double velocity;
        double acceleration;
    };

    ParallelArmKinematics();
    explicit ParallelArmKinematics(const Params& params);

    // ===================== 四连杆约束 =====================

    /**
     * @brief 四连杆正解：小臂电机角 θ2 → 小臂相对角 q2
     * @return q2 (rad)，无解时返回 std::nullopt
     */
    std::optional<double> fourBarForward(double theta2) const;

    /**
     * @brief 四连杆逆解：小臂相对角 q2 → 小臂电机角 θ2
     * @return θ2 (rad)，无解时返回 std::nullopt
     */
    std::optional<double> fourBarInverse(double q2) const;

    // ===================== 单臂运动学 =====================

    /**
     * @brief 单臂正运动学：(q1, θ2) → 末端连接点坐标
     *        坐标系以该臂肩关节（大臂电机轴心）为原点
     * @return 末端 (x, y)，无解时返回 std::nullopt
     */
    std::optional<EndPoint> singleArmFK(double q1, double theta2) const;

    /**
     * @brief 单臂逆运动学：末端连接点 (x, y) → (q1, θ2)
     *        坐标系以该臂肩关节（大臂电机轴心）为原点
     * @return 解算结果，无解时返回 std::nullopt
     */
    std::optional<SingleArmSolution> singleArmIK(double x, double y) const;

    // ===================== 双臂并联 =====================

    /**
     * @brief 并联正运动学：四个电机角 → 击球板中心高度 H 和俯仰角 φ
     * @param left_q1, left_theta2  左臂电机角
     * @param right_q1, right_theta2 右臂电机角
     * @return {H, phi}，无解时返回 std::nullopt
     */
    std::optional<std::array<double, 2>> parallelFK(
        double left_q1, double left_theta2,
        double right_q1, double right_theta2) const;

    /**
     * @brief 并联逆运动学：(H, φ) → 四个电机角
     * @param H   击球板中心高度 (m)
     * @param phi 俯仰角 (rad)，板面法线与水平夹角
     * @return 四个电机角，无解时返回 std::nullopt
     */
    std::optional<DualArmSolution> parallelIK(double H, double phi) const;

    // ===================== 轨迹规划 =====================

    /**
     * @brief 五次多项式轨迹规划（位置+速度+加速度边界条件）
     * @param p0, v0, a0  起始位置、速度、加速度
     * @param pf, vf, af  终止位置、速度、加速度
     * @param T   运动总时间 (s)
     * @param dt  采样周期 (s)
     * @return 离散轨迹点序列
     */
    static std::vector<TrajectoryPoint> quinticTrajectory(
        double p0, double v0, double a0,
        double pf, double vf, double af,
        double T, double dt);

    // ===================== 限位检查 =====================
    bool checkLimits(const SingleArmSolution& sol) const;
    bool checkLimits(const DualArmSolution& sol) const;

    // ===================== 速度与力矩映射 =====================

    /**
     * @brief 四连杆传动比：dq2/dθ2
     * @param theta2 小臂电机角 (rad)
     * @return 传动比，无解时返回 std::nullopt
     */
    std::optional<double> fourBarRatio(double theta2) const;

    /**
     * @brief 任务空间速度 → 电机速度
     *        给定当前电机角和末端速度 (Ḣ, φ̇)，求 4 个电机的角速度
     * @param sol    当前双臂关节角
     * @param dH     击球板中心高度变化率 (m/s)
     * @param dphi   俯仰角变化率 (rad/s)
     * @return {dq1_L, dth2_L, dq1_R, dth2_R} (rad/s)，奇异时返回 std::nullopt
     */
    std::optional<std::array<double, 4>> inverseVelocity(
        const DualArmSolution& sol,
        double dH, double dphi) const;

    /**
     * @brief 末端力 → 电机力矩（静力学映射）
     *        给定当前机构构型和末端力/力矩，求 4 个电机的力矩
     * @param sol     当前双臂关节角
     * @param F_H     沿 H 方向（竖直）的力 (N)
     * @param T_phi   绕俯仰轴的力矩 (Nm)
     * @return {τ_q1L, τ_th2L, τ_q1R, τ_th2R} (Nm)，奇异时返回 std::nullopt
     */
    std::optional<std::array<double, 4>> inverseTorque(
        const DualArmSolution& sol,
        double F_H, double T_phi) const;

    const Params& params() const { return params_; }

private:
    Params params_;
    double half_D_;  // D/2
};

}  // namespace motor_control

#endif  // MOTOR_CONTROL_ROS2__PARALLEL_ARM_KINEMATICS_HPP_
