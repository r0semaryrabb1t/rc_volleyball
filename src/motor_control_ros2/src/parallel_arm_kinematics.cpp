#include "motor_control_ros2/parallel_arm_kinematics.hpp"
#include <cmath>
#include <stdexcept>

namespace motor_control {

ParallelArmKinematics::ParallelArmKinematics()
    : params_(), half_D_(params_.D / 2.0) {}

ParallelArmKinematics::ParallelArmKinematics(const Params& params)
    : params_(params), half_D_(params.D / 2.0) {}

// ======================== 四连杆约束 ========================

std::optional<double> ParallelArmKinematics::fourBarForward(double theta2) const {
    /*
     * 四连杆正解：θ2 → q2（在大臂局部坐标系中建模）
     *
     * 为什么不含 q1：
     *   小臂电机与大臂电机共轴，小臂电机的定子固定在大臂上，随大臂整体旋转。
     *   θ2 是曲柄在大臂坐标系中的角度；q2 是小臂相对大臂的夹角。
     *   四连杆四个铰接点（曲柄轴心、曲柄端点、肘关节、小臂铰接点）在大臂坐标系中
     *   的位置均不依赖 q1 —— q1 只决定整个大臂坐标系在全局坐标系中的朝向。
     *
     * 大臂局部坐标系：原点在肩关节（=曲柄轴心），+Y_local 沿大臂方向（肩→肘）。
     *
     * 曲柄端点 C（零点垂直向下 = -Y_local，逆时针为正）：
     *   C = (r·sin(θ2), -r·cos(θ2))
     *
     * 肘关节 E = (0, L1)
     *
     * 小臂上铰接点 P = E + d·(小臂方向) = (d·sin(q2), L1 + d·cos(q2))
     *   （q2=0 时小臂沿 +Y_local 延伸，逆时针为正）
     *
     * 闭环约束：|C - P| = l
     * → A·sin(q2) + B·cos(q2) = K
     */

    const double r = params_.r;
    const double l = params_.l;
    const double d = params_.d;
    const double L1 = params_.L1;

    // 曲柄端点坐标（大臂局部坐标系）
    double Cx = r * std::sin(theta2);
    double Cy = -r * std::cos(theta2);

    // 肘关节在大臂末端 (0, L1)

    // |P - C|^2 = l^2 展开
    // P = (d*sin(q2), L1 + d*cos(q2))
    // (d*sin(q2) - Cx)^2 + (L1 + d*cos(q2) - Cy)^2 = l^2
    //
    // d^2 - 2*d*Cx*sin(q2) - 2*d*(L1-Cy)*cos(q2) + Cx^2 + (L1-Cy)^2 = l^2
    //
    // => A*sin(q2) + B*cos(q2) = C

    double delta_y = L1 - Cy;  // = L1 + r*cos(θ2)
    double A = -2.0 * d * Cx;
    double B = 2.0 * d * delta_y;
    double C_val = l * l - d * d - Cx * Cx - delta_y * delta_y;

    // 求解 A*sin(q2) + B*cos(q2) = C_val
    double R = std::sqrt(A * A + B * B);
    if (R < 1e-12) {
        return std::nullopt;
    }

    double ratio = C_val / R;
    if (std::abs(ratio) > 1.0) {
        return std::nullopt;  // 无解
    }

    double gamma = std::atan2(A, B);
    // cos(q2 - gamma) = C_val / R
    double alpha = std::acos(ratio);

    // 两个解：q2 = gamma ± alpha，选物理合理的（q2 ≥ 0）
    double q2_1 = gamma + alpha;
    double q2_2 = gamma - alpha;

    // 归一化到 [-π, π]
    auto normalize = [](double a) {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    };
    q2_1 = normalize(q2_1);
    q2_2 = normalize(q2_2);

    // 选择在合理范围 [0, ~π] 内且更接近正值的解
    bool v1 = (q2_1 >= -0.1 && q2_1 <= M_PI);
    bool v2 = (q2_2 >= -0.1 && q2_2 <= M_PI);

    if (v1 && v2) {
        return (q2_1 >= 0.0) ? q2_1 : q2_2;
    } else if (v1) {
        return q2_1;
    } else if (v2) {
        return q2_2;
    }

    return std::nullopt;
}

std::optional<double> ParallelArmKinematics::fourBarInverse(double q2) const {
    /*
     * 四连杆逆解：q2 → θ2（大臂局部坐标系，同正解不含 q1）
     *
     * 已知 P = (d·sin(q2), L1 + d·cos(q2))，求 θ2 使 |C-P|=l，
     * 其中 C = (r·sin(θ2), -r·cos(θ2))。
     * → A'·sin(θ2) + B'·cos(θ2) = K'
     */

    const double r = params_.r;
    const double l = params_.l;
    const double d = params_.d;
    const double L1 = params_.L1;

    // 铰接点 P 坐标
    double Px = d * std::sin(q2);
    double Py = L1 + d * std::cos(q2);

    // |C - P|^2 = l^2
    // (r*sin(θ2) - Px)^2 + (-r*cos(θ2) - Py)^2 = l^2
    // r^2 - 2*r*Px*sin(θ2) + 2*r*Py*cos(θ2) + Px^2 + Py^2 = l^2
    // A'*sin(θ2) + B'*cos(θ2) = C'

    double A = -2.0 * r * Px;
    double B = 2.0 * r * Py;
    double C_val = l * l - r * r - Px * Px - Py * Py;

    double R = std::sqrt(A * A + B * B);
    if (R < 1e-12) {
        return std::nullopt;
    }

    double ratio = C_val / R;
    if (std::abs(ratio) > 1.0) {
        return std::nullopt;
    }

    double gamma = std::atan2(A, B);
    double alpha = std::acos(ratio);

    double t1 = gamma + alpha;
    double t2 = gamma - alpha;

    auto normalize = [](double a) {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    };
    t1 = normalize(t1);
    t2 = normalize(t2);

    bool v1 = (t1 >= -0.1 && t1 <= M_PI);
    bool v2 = (t2 >= -0.1 && t2 <= M_PI);

    if (v1 && v2) {
        return (t1 >= 0.0) ? t1 : t2;
    } else if (v1) {
        return t1;
    } else if (v2) {
        return t2;
    }

    return std::nullopt;
}

// ======================== 单臂正运动学 ========================

std::optional<ParallelArmKinematics::EndPoint>
ParallelArmKinematics::singleArmFK(double q1, double theta2) const {
    /*
     * 全局坐标系：以该臂肩关节（大臂电机轴心）为原点，X 水平向右，Y 垂直向上。
     *
     * q1 = 大臂从垂直向下（-Y 方向）顺时针旋转的角度（顺时针为正）。
     *   物理含义：q1 > 0 时大臂向内侧（肩距中心）偏转。
     *
     * 内部角 α = -q1（转换为逆时针正方向供三角函数使用）
     *
     * 大臂方向角 = -π/2 + α = -π/2 - q1
     *   → 方向向量 = (sin(-q1), -cos(-q1)) = (-sin(q1), -cos(q1))
     *
     * 肘关节 E = L1 · (-sin(q1), -cos(q1))
     *
     * 四连杆约束在大臂局部坐标系中，q2 不依赖 q1。
     * q2 定义不变：小臂相对大臂逆时针为正。
     *
     * 小臂绝对方向角 = -π/2 - q1 + q2
     *   → 方向向量 = (sin(-q1+q2), -cos(-q1+q2))
     *
     * 末端 P = E + L2 · (sin(-q1+q2), -cos(-q1+q2))
     */

    auto q2_opt = fourBarForward(theta2);
    if (!q2_opt) {
        return std::nullopt;
    }
    double q2 = *q2_opt;

    double L1 = params_.L1;
    double L2 = params_.L2;

    // 肘关节（q1 顺时针为正 → 使用 -q1）
    double Ex = -L1 * std::sin(q1);
    double Ey = -L1 * std::cos(q1);

    // 末端
    double abs_forearm = -q1 + q2;  // 小臂绝对角（逆时针正）
    double Px = Ex + L2 * std::sin(abs_forearm);
    double Py = Ey - L2 * std::cos(abs_forearm);

    return EndPoint{Px, Py};
}

// ======================== 单臂逆运动学 ========================

std::optional<ParallelArmKinematics::SingleArmSolution>
ParallelArmKinematics::singleArmIK(double x, double y) const {
    /*
     * 给定末端连接点 (x, y)，以肩关节为原点。
     *
     * 标准 2R 逆运动学：
     *   d^2 = x^2 + y^2
     *   cos(q2) = (d^2 - L1^2 - L2^2) / (2*L1*L2)
     *   q2 = acos(…)  选正值（肘角逆时针）
     *
     * FK 中使用内部角 α_int = -q1（逆时针正）:
     *   x = sin(α_int + q2) * ... , y = -cos(α_int + q2) * ...
     *   极角 α = atan2(x, -y) = α_int + β  → α_int = α - β
     *
     * 最终 q1 = -α_int = -(α - β) = β - α
     */

    double L1 = params_.L1;
    double L2 = params_.L2;

    double d2 = x * x + y * y;
    double d = std::sqrt(d2);

    // 可达性检查
    if (d > (L1 + L2) || d < std::abs(L1 - L2)) {
        return std::nullopt;
    }

    // 余弦定理求 q2
    double cos_q2 = (d2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
    if (std::abs(cos_q2) > 1.0) {
        return std::nullopt;
    }

    double q2 = std::acos(cos_q2);  // 取正值（肘角逆时针展开）

    // 求内部角（逆时针正）
    double alpha = std::atan2(x, -y);
    double beta = std::atan2(L2 * std::sin(q2), L1 + L2 * std::cos(q2));
    double alpha_int = alpha - beta;  // 大臂在逆时针正系统中的角度

    // 转换为 q1（顺时针正）
    double q1 = -alpha_int;  // q1 = -(alpha - beta) = beta - alpha

    // 四连杆逆解：q2 → θ2
    auto theta2_opt = fourBarInverse(q2);
    if (!theta2_opt) {
        return std::nullopt;
    }

    return SingleArmSolution{q1, *theta2_opt, q2};
}

// ======================== 双臂并联正运动学 ========================

std::optional<std::array<double, 2>> ParallelArmKinematics::parallelFK(
    double left_q1, double left_theta2,
    double right_q1, double right_theta2) const {
    /*
     * 两臂镜像对称安装：
     *   左臂肩在 (-D/2, 0)，singleArmFK 的 x 正方向 = 全局 x 正方向
     *   右臂肩在 (+D/2, 0)，镜像安装 → x 正方向 = 全局 x 负方向
     *
     * 全局坐标：
     *   P_L = (x_L - D/2,  y_L)
     *   P_R = (D/2 - x_R,  y_R)     ← x 翻转（镜像）
     *
     * 击球板中心高度 H = (P_L.y + P_R.y) / 2
     * 俯仰角 φ = atan2(P_Ly - P_Ry, P_Rx - P_Lx)
     *   φ > 0 表示左高右低
     */

    auto left_end = singleArmFK(left_q1, left_theta2);
    auto right_end = singleArmFK(right_q1, right_theta2);

    if (!left_end || !right_end) {
        return std::nullopt;
    }

    // 转换到全局坐标系
    double Lx = left_end->x - half_D_;
    double Ly = left_end->y;
    double Rx = half_D_ - right_end->x;   // 镜像：x 翻转
    double Ry = right_end->y;

    double H = (Ly + Ry) / 2.0;

    // 板方向向量 P_L→P_R = (Rx-Lx, Ry-Ly)
    // φ = 从水平方向的倾角：左高右低为正
    double dx = Rx - Lx;
    double dy = Ry - Ly;
    double phi = std::atan2(-dy, dx);

    return std::array<double, 2>{H, phi};
}

// ======================== 双臂并联逆运动学 ========================

std::optional<ParallelArmKinematics::DualArmSolution>
ParallelArmKinematics::parallelIK(double H, double phi) const {
    /*
     * 给定击球板中心高度 H 和俯仰角 φ（左高右低为正）。
     *
     * 板中心在 (0, H)（对称假设 x_c = 0）。
     * 板长 D，方向向量 (cos(φ), -sin(φ))（水平向右且右端低）。
     *
     * 全局坐标系下板的两个连接点：
     *   P_L = (-D/2·cos(φ),  H + D/2·sin(φ))
     *   P_R = ( D/2·cos(φ),  H - D/2·sin(φ))
     *
     * 转换到各臂局部坐标系：
     *   左臂（肩 (-D/2, 0)）：直接平移
     *   右臂（肩 (+D/2, 0)，镜像安装）：x_local = D/2 - P_R.x
     */

    double a = half_D_;

    // 全局坐标系下末端连接点
    double cos_phi = std::cos(phi);
    double sin_phi = std::sin(phi);
    double PLx_global = -a * cos_phi;
    double PLy_global = H + a * sin_phi;
    double PRx_global = a * cos_phi;
    double PRy_global = H - a * sin_phi;

    // 转换到左臂局部坐标系（左肩在全局 (-D/2, 0)）
    double PLx_local = PLx_global + half_D_;   // = D/2 * (1 - cos(φ))
    double PLy_local = PLy_global;

    // 转换到右臂局部坐标系（右肩在全局 (+D/2, 0)，镜像安装）
    double PRx_local = half_D_ - PRx_global;   // = D/2 * (1 - cos(φ))
    double PRy_local = PRy_global;

    // 分别调用单臂 IK
    auto left_sol = singleArmIK(PLx_local, PLy_local);
    auto right_sol = singleArmIK(PRx_local, PRy_local);

    if (!left_sol || !right_sol) {
        return std::nullopt;
    }

    DualArmSolution sol{*left_sol, *right_sol};

    if (!checkLimits(sol)) {
        return std::nullopt;
    }

    return sol;
}

// ======================== 轨迹规划 ========================

std::vector<ParallelArmKinematics::TrajectoryPoint>
ParallelArmKinematics::quinticTrajectory(
    double p0, double v0, double a0,
    double pf, double vf, double af,
    double T, double dt) {
    /*
     * 五次多项式：s(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
     *
     * 边界条件：
     *   s(0) = p0,  s'(0) = v0,  s''(0) = a0
     *   s(T) = pf,  s'(T) = vf,  s''(T) = af
     *
     * 求解 6 个系数。
     */

    if (T <= 0.0 || dt <= 0.0) {
        return {};
    }

    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;

    double c0 = p0;
    double c1 = v0;
    double c2 = a0 / 2.0;

    // 由边界条件解 c3, c4, c5（线性系统 3×3）
    double dp = pf - p0 - v0 * T - (a0 / 2.0) * T2;
    double dv = vf - v0 - a0 * T;
    double da = af - a0;

    // 矩阵逆解（Cramer法则）
    // [T^3    T^4    T^5  ] [c3]   [dp]
    // [3T^2   4T^3   5T^4 ] [c4] = [dv]
    // [6T     12T^2  20T^3] [c5]   [da]

    // 行列式 = 2*T^9（解析化简）

    // 简化行列式计算
    // | T3   T4   T5  |
    // | 3T2  4T3  5T4 |
    // | 6T   12T2 20T3|
    // det = T3*(4T3*20T3 - 5T4*12T2) - T4*(3T2*20T3 - 5T4*6T) + T5*(3T2*12T2 - 4T3*6T)
    //     = T3*(80T6 - 60T6) - T4*(60T5 - 30T5) + T5*(36T4 - 24T4)
    //     = T3*20T6 - T4*30T5 + T5*12T4
    //     = 20T9 - 30T9 + 12T9
    //     = 2T9

    // 使用更直接的方式
    double det_val = 2.0 * T3 * T3 * T3;  // 2*T^9

    if (std::abs(det_val) < 1e-30) {
        return {};
    }

    // Cramer 法则
    // c3 = (dp*(4T3*20T3 - 5T4*12T2) - T4*(dv*20T3 - 5T4*da) + T5*(dv*12T2 - 4T3*da)) / det
    // 简化后使用标准公式：

    double c3 = (20.0 * dp - 8.0 * T * dv + T2 * da) / (2.0 * T3);
    double c4 = (-30.0 * dp + 14.0 * T * dv - 2.0 * T2 * da) / (2.0 * T4);
    double c5 = (12.0 * dp - 6.0 * T * dv + T2 * da) / (2.0 * T5);

    std::vector<TrajectoryPoint> trajectory;
    size_t n = static_cast<size_t>(T / dt) + 1;
    trajectory.reserve(n);

    for (size_t i = 0; i < n; ++i) {
        double t = i * dt;
        if (t > T) t = T;

        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        double t5 = t4 * t;

        TrajectoryPoint pt;
        pt.time = t;
        pt.position     = c0 + c1 * t + c2 * t2 + c3 * t3 + c4 * t4 + c5 * t5;
        pt.velocity     = c1 + 2.0 * c2 * t + 3.0 * c3 * t2 + 4.0 * c4 * t3 + 5.0 * c5 * t4;
        pt.acceleration = 2.0 * c2 + 6.0 * c3 * t + 12.0 * c4 * t2 + 20.0 * c5 * t3;

        trajectory.push_back(pt);
    }

    return trajectory;
}

// ======================== 限位检查 ========================

bool ParallelArmKinematics::checkLimits(const SingleArmSolution& sol) const {
    return sol.q1 >= params_.q1_min && sol.q1 <= params_.q1_max &&
           sol.theta2 >= params_.theta2_min && sol.theta2 <= params_.theta2_max;
}

bool ParallelArmKinematics::checkLimits(const DualArmSolution& sol) const {
    return checkLimits(sol.left) && checkLimits(sol.right);
}

// ======================== 四连杆传动比 ========================

std::optional<double> ParallelArmKinematics::fourBarRatio(double theta2) const {
    /*
     * 四连杆约束方程：A·sin(q2) + B·cos(q2) = K
     *   A = 2·d·delta_x,  B = 2·d·delta_y,  K = delta_x² + delta_y² + d² - l²
     *   其中 delta_x = r·sin(θ2),  delta_y = -r·cos(θ2)
     *
     * 对 θ2 求导：
     *   A'·sin(q2) + A·cos(q2)·q2' + B'·cos(q2) - B·sin(q2)·q2' = K'
     *
     * 整理得：
     *   q2' = (K' - A'·sin(q2) - B'·cos(q2)) / (A·cos(q2) - B·sin(q2))
     */

    auto q2_opt = fourBarForward(theta2);
    if (!q2_opt) return std::nullopt;
    double q2 = *q2_opt;

    double r = params_.r;
    double d = params_.d;

    double delta_x = r * std::sin(theta2);
    double delta_y = -r * std::cos(theta2);

    double A = 2.0 * d * delta_x;
    double B = 2.0 * d * delta_y;

    // 对 θ2 求导
    double d_delta_x = r * std::cos(theta2);
    double d_delta_y = r * std::sin(theta2);

    double dA = 2.0 * d * d_delta_x;
    double dB = 2.0 * d * d_delta_y;
    double dK = 2.0 * delta_x * d_delta_x + 2.0 * delta_y * d_delta_y;

    double sin_q2 = std::sin(q2);
    double cos_q2 = std::cos(q2);

    double denom = A * cos_q2 - B * sin_q2;
    if (std::abs(denom) < 1e-12) return std::nullopt;  // 奇异构型

    double numer = dK - dA * sin_q2 - dB * cos_q2;
    return numer / denom;  // dq2/dθ2
}

// ======================== 速度逆映射 ========================

std::optional<std::array<double, 4>> ParallelArmKinematics::inverseVelocity(
    const DualArmSolution& sol,
    double dH, double dphi) const {
    /*
     * 并联机构速度映射。
     *
     * 单臂 FK（以左臂为例，q1 顺时针正）：
     *   x_L = -L1·sin(q1) + L2·sin(-q1+q2)    （局部坐标）
     *   y_L = -L1·cos(q1) - L2·cos(-q1+q2)
     *
     * 全局坐标：
     *   P_Lx_global = x_L - D/2
     *   P_Ly_global = y_L
     *
     * 任务空间 (H, φ) 与全局末端的关系：
     *   P_Ly = H + D/2·sin(φ)    (左臂)
     *   P_Ry = H - D/2·sin(φ)    (右臂)
     *
     * 对时间求导，建立数值雅可比。
     * 使用数值差分计算 ∂H/∂q1_L, ∂H/∂θ2_L, ∂H/∂q1_R, ∂H/∂θ2_R 等。
     */

    const double eps = 1e-7;
    double q_base[4] = {sol.left.q1, sol.left.theta2, sol.right.q1, sol.right.theta2};

    // 计算当前 (H, phi)
    auto fk0 = parallelFK(q_base[0], q_base[1], q_base[2], q_base[3]);
    if (!fk0) return std::nullopt;
    double H0 = (*fk0)[0], phi0 = (*fk0)[1];

    // 数值雅可比 J[2][4]：∂(H,φ)/∂(q1L, θ2L, q1R, θ2R)
    double J[2][4];
    for (int i = 0; i < 4; ++i) {
        double q_pert[4] = {q_base[0], q_base[1], q_base[2], q_base[3]};
        q_pert[i] += eps;
        auto fk_pert = parallelFK(q_pert[0], q_pert[1], q_pert[2], q_pert[3]);
        if (!fk_pert) return std::nullopt;
        J[0][i] = ((*fk_pert)[0] - H0) / eps;
        J[1][i] = ((*fk_pert)[1] - phi0) / eps;
    }

    // 由于系统 2 自由度、4 电机，使用伪逆 J^+ = J^T(JJ^T)^{-1}
    // JJ^T 是 2×2 矩阵
    double JJT[2][2] = {{0, 0}, {0, 0}};
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 4; ++k)
                JJT[i][j] += J[i][k] * J[j][k];

    double det = JJT[0][0] * JJT[1][1] - JJT[0][1] * JJT[1][0];
    if (std::abs(det) < 1e-20) return std::nullopt;  // 奇异

    double inv_det = 1.0 / det;
    double JJT_inv[2][2] = {
        { JJT[1][1] * inv_det, -JJT[0][1] * inv_det},
        {-JJT[1][0] * inv_det,  JJT[0][0] * inv_det}
    };

    // J+ = J^T · (JJ^T)^{-1}，大小 4×2
    // dq = J+ · [dH, dphi]^T
    double dx[2] = {dH, dphi};
    // 先算 (JJ^T)^{-1} · dx  →  tmp[2]
    double tmp[2] = {
        JJT_inv[0][0] * dx[0] + JJT_inv[0][1] * dx[1],
        JJT_inv[1][0] * dx[0] + JJT_inv[1][1] * dx[1]
    };
    // dq = J^T · tmp
    std::array<double, 4> dq;
    for (int i = 0; i < 4; ++i) {
        dq[i] = J[0][i] * tmp[0] + J[1][i] * tmp[1];
    }

    // dq 中的第 1、3 项是 dq1/dt，第 0、2 项也是 dq1/dt
    // 但电机直接控制的是 q1 和 θ2，需要通过四连杆传动比转换 dq2 为 dθ2
    // 注：parallelFK 直接接受 (q1, θ2) 作为输入，数值差分已自动包含四连杆
    // 所以 dq[0]=dq1_L, dq[1]=dθ2_L, dq[2]=dq1_R, dq[3]=dθ2_R 就是电机角速度

    return dq;
}

// ======================== 力矩逆映射 ========================

std::optional<std::array<double, 4>> ParallelArmKinematics::inverseTorque(
    const DualArmSolution& sol,
    double F_H, double T_phi) const {
    /*
     * 虚功原理：τ = J^T · F
     * 
     * 其中 J 是 inverseVelocity 中算出的同一个雅可比矩阵。
     * F = [F_H, T_phi]^T，τ = [τ_q1L, τ_θ2L, τ_q1R, τ_θ2R]^T
     */

    const double eps = 1e-7;
    double q_base[4] = {sol.left.q1, sol.left.theta2, sol.right.q1, sol.right.theta2};

    auto fk0 = parallelFK(q_base[0], q_base[1], q_base[2], q_base[3]);
    if (!fk0) return std::nullopt;
    double H0 = (*fk0)[0], phi0 = (*fk0)[1];

    double J[2][4];
    for (int i = 0; i < 4; ++i) {
        double q_pert[4] = {q_base[0], q_base[1], q_base[2], q_base[3]};
        q_pert[i] += eps;
        auto fk_pert = parallelFK(q_pert[0], q_pert[1], q_pert[2], q_pert[3]);
        if (!fk_pert) return std::nullopt;
        J[0][i] = ((*fk_pert)[0] - H0) / eps;
        J[1][i] = ((*fk_pert)[1] - phi0) / eps;
    }

    // τ = J^T · F
    double F[2] = {F_H, T_phi};
    std::array<double, 4> torque;
    for (int i = 0; i < 4; ++i) {
        torque[i] = J[0][i] * F[0] + J[1][i] * F[1];
    }

    return torque;
}

}  // namespace motor_control
