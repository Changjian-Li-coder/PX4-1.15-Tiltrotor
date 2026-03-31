#include <matrix/math.hpp>
#include <cmath>
#include "allocation_calculation.hpp"
using namespace matrix;

void AllocationCalculation::set_parameters(float lambda_thrust, float lambda_servo, float thrust_min, float thrust_max,
                       float servo_angle_limit_deg, int max_iter, float lr_thrust, float lr_servo, const matrix::Vector<float, 6>& W)
{
    _lambda_thrust = fmaxf(0.0f, lambda_thrust);
    _lambda_servo = fmaxf(0.0f, lambda_servo);

    _thrust_min = fmaxf(0.0f, thrust_min);
    _thrust_max = fmaxf(_thrust_min + 1e-4f, thrust_max);

    const float servo_limit_abs_deg = fabsf(servo_angle_limit_deg);
    _servo_angle_limit_rad = fmaxf(1e-3f, servo_limit_abs_deg * (M_PI_F / 180.0f));

    _max_iter = max_iter > 0 ? max_iter : 1;

    // 分离推力和角度的学习率
    _lr_thrust = fmaxf(1e-6f, lr_thrust);
    _lr_servo = fmaxf(1e-6f, lr_servo);

    // 保存误差权重向量 (例如: {1, 1, 1, 5, 5, 1} 代表加大对 Roll(3) 和 Pitch(4) 的惩罚)
    _W = W;
}

// 核心分配求解器 (使用解析梯度下降)
void AllocationCalculation::solve_allocation(const matrix::Vector<float, 6>& U_target, const matrix::Matrix<float, 6, 8>& A, matrix::Vector<float, 8>& x_out) {
    // 热启动保护：如果全为0则初始化一个微小推力
    if (x_out.norm() < 0.01f) {
        for (int i = 0; i < 4; ++i) {
            x_out(2 * i) = _thrust_min;
            x_out(2 * i + 1) = 0.0f;
        }
    }

    // 预先计算转置矩阵
    A_T = A.transpose();

    // 投影梯度下降迭代
    for (int iter = 0; iter < _max_iter; ++iter) {
        // 1. 计算当前的虚拟控制量 b
        for (int i = 0; i < 4; ++i) {
            b(2 * i)     = x_out(2 * i) * cosf(x_out(2 * i + 1));
            b(2 * i + 1) = x_out(2 * i) * sinf(x_out(2 * i + 1));
        }

        // 2. 计算误差向量 e = A*b - U_target
        error = A * b - U_target;

        // 【新增】应用加权矩阵 W，提升特定轴（如 x/y 轴力矩）的敏感度
        for (int i = 0; i < 6; ++i) {
            error(i) *= _W(i);
        }

        // 3. 计算对 b 的梯度: g_b = 2 * A^T * error
        g_b = A_T * error * 2.0f;

        // 4. 使用链式法则计算对状态变量 x 的解析梯度
        for (int i = 0; i < 4; ++i) {
            float F = x_out(2 * i);
            float alpha = x_out(2 * i + 1);
            float cos_a = cosf(alpha);
            float sin_a = sinf(alpha);

            // 对推力 F_i 的梯度
            grad(2 * i) = g_b(2 * i) * cos_a + g_b(2 * i + 1) * sin_a + 2.0f * _lambda_thrust * F;

            // 对角度 alpha_i 的梯度
            grad(2 * i + 1) = -g_b(2 * i) * F * sin_a + g_b(2 * i + 1) * F * cos_a + 2.0f * _lambda_servo * alpha;
        }

        // 5. 更新状态
        for (int i = 0; i < 4; ++i) {
            x_out(2 * i)     -= grad(2 * i) * _lr_thrust;     // 更新推力
            x_out(2 * i + 1) -= grad(2 * i + 1) * _lr_servo;  // 更新舵机角度
        }

        // 6. 边界约束 (投影)
        for (int i = 0; i < 4; ++i) {
            // 推力约束
            if (x_out(2 * i) < _thrust_min) x_out(2 * i) = _thrust_min;
            if (x_out(2 * i) > _thrust_max) x_out(2 * i) = _thrust_max;

            // 舵机角度约束
            if (x_out(2 * i + 1) < -_servo_angle_limit_rad) x_out(2 * i + 1) = -_servo_angle_limit_rad;
            if (x_out(2 * i + 1) > _servo_angle_limit_rad)  x_out(2 * i + 1) = _servo_angle_limit_rad;
        }
    }
}
