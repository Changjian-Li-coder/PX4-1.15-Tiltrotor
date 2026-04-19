#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <Eigen/Eigen>
#include <nlopt.hpp>
#include <cmath>
#include <vector>
#include <iostream>

// 全局参数配置（可迁移到PX4参数系统）
static constexpr double L = 0.182;           // 轴距 (m)
static constexpr double k = 0.01;            // 反扭矩系数
static constexpr double T_max = 2.586 * 4;   // 最大推力 (4个电机总和)
static constexpr double tau_roll_max = L * 4 * sqrt(2)/2;
static constexpr double tau_pitch_max = L * 4 * sqrt(2)/2;
static constexpr double tau_yaw_max = L * 4;
static constexpr double LAMBDA_THRUST = 0.01;
static constexpr double LAMBDA_SERVO = 1.5;  // 舵机惩罚权重

// 分配矩阵A（固定不变）
static Eigen::Matrix<double, 6, 8> getAllocationMatrixA()
{
    const double sqrt2_2 = sqrt(2) / 2;
    Eigen::Matrix<double, 6, 8> A;
    // 逐行初始化矩阵A
    A << 0, -sqrt2_2, 0, sqrt2_2, 0, sqrt2_2, 0, -sqrt2_2,
         0, sqrt2_2, 0, -sqrt2_2, 0, sqrt2_2, 0, -sqrt2_2,
         -1, 0, -1, 0, -1, 0, -1, 0,
         -sqrt2_2*L, -k*sqrt2_2, sqrt2_2*L, k*sqrt2_2, sqrt2_2*L, k*sqrt2_2, -sqrt2_2*L, -k*sqrt2_2,
         sqrt2_2*L, k*sqrt2_2, -sqrt2_2*L, -k*sqrt2_2, sqrt2_2*L, k*sqrt2_2, -sqrt2_2*L, -k*sqrt2_2,
         -k, L, -k, L, k, L, k, L;
    return A;
}

// 优化目标函数（nlopt回调）
static double objectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    (void)grad; // 禁用梯度（SLSQP可自动数值求导）
    Eigen::VectorXd U_target = *static_cast<Eigen::VectorXd*>(data);
    Eigen::Matrix<double, 6, 8> A = getAllocationMatrixA();

    // 拆分推力F和角度alpha
    Eigen::Vector4d F;   // 4个电机推力
    Eigen::Vector4d alpha; // 4个舵机角度（弧度）
    for (int i = 0; i < 4; i++) {
        F(i) = x[2*i];
        alpha(i) = x[2*i+1];
    }

    // 构建b向量
    Eigen::Vector8d b;
    for (int i = 0; i < 4; i++) {
        b(2*i) = F(i) * cos(alpha(i));
        b(2*i+1) = F(i) * sin(alpha(i));
    }

    // 计算误差项 + 惩罚项
    Eigen::Vector6d error = A * b - U_target;
    double error_term = error.squaredNorm();
    double thrust_penalty = LAMBDA_THRUST * F.squaredNorm();
    double servo_penalty = LAMBDA_SERVO * alpha.squaredNorm();

    return error_term + thrust_penalty + servo_penalty;
}

// 推力分配核心函数
bool solveAllocation(const Eigen::VectorXd &U_target, Eigen::Vector4d &thrusts, Eigen::Vector4d &angles_deg)
{
    // 初始化nlopt优化器（SLSQP算法，对应Python的SLSQP）
    nlopt::opt opt(nlopt::LD_SLSQP, 8); // 8个优化变量（4F+4alpha）

    // 设置目标函数
    opt.set_min_objective(objectiveFunction, const_cast<Eigen::VectorXd*>(&U_target));

    // 设置变量边界
    std::vector<std::pair<double, double>> bounds;
    for (int i = 0; i < 4; i++) {
        bounds.emplace_back(0.1, 0.9); // 推力范围
        bounds.emplace_back(-M_PI/8, M_PI/8); // 角度范围（-22.5°~22.5°）
    }
    opt.set_lower_bounds(bounds);
    opt.set_upper_bounds(bounds);

    // 优化参数（PX4实时性要求，设置迭代次数和精度）
    opt.set_maxeval(100); // 最大迭代次数
    opt.set_ftol_rel(1e-6); // 相对精度

    // 初始猜测（全0）
    std::vector<double> x0(8, 0.0);
    double minf; // 最优目标函数值
    nlopt::result result = opt.optimize(x0, minf);

    // 检查优化是否成功
    if (result < 0) {
        PX4_ERR("Optimization failed: %d", result);
        return false;
    }

    // 提取结果
    for (int i = 0; i < 4; i++) {
        thrusts(i) = x0[2*i];
        angles_deg(i) = x0[2*i+1] * 180.0 / M_PI; // 弧度转角度
    }

    return true;
}

// PX4模块入口示例（可嵌入到mixers或control_allocator模块）
extern "C" __EXPORT int thrust_allocation_example_main(int argc, char *argv[])
{
    // 模拟控制输入（物理单位: N 和 N*m）
    Eigen::Vector6d my_control_input;
    my_control_input << 0.0, 0.2, -0.6, 0.0, 0.0, 0.0;

    PX4_INFO("Input control vector: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
             my_control_input(0), my_control_input(1), my_control_input(2),
             my_control_input(3), my_control_input(4), my_control_input(5));

    // 缩放控制输入（匹配Python逻辑）
    my_control_input(0) *= 4;
    my_control_input(1) *= 4;
    my_control_input(2) *= 4;
    my_control_input(3) *= tau_roll_max;
    my_control_input(4) *= tau_pitch_max;
    my_control_input(5) *= tau_yaw_max;

    // 求解推力分配
    Eigen::Vector4d thrusts;
    Eigen::Vector4d angles_deg;
    bool success = solveAllocation(my_control_input, thrusts, angles_deg);

    if (success) {
        for (int i = 0; i < 4; i++) {
            PX4_INFO("Motor %d: Thrust=%.8f, Servo=%.8f°",
                     i+1, thrusts(i), angles_deg(i));
        }
    }

    return 0;
}

// PX4模块注册（可选，如需独立模块）
MODULE_DESCRIPTION("PX4 Thrust Allocation Module");
MODULE_NAME("thrust_allocation");
MODULE_PARAMETERS(
    // 可添加PX4参数（如L, k等）
);
