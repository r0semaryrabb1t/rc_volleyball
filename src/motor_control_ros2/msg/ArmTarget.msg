std_msgs/Header header

# 目标关节角度（3路 GO8010 电机，弧度）
float64[3] target_angles

# 执行标志：true 时进入 EXECUTE 状态，false 时忽略
bool execute
