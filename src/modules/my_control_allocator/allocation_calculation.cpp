#include <matrix/math.hpp>
#include <cmath>

#include "allocation_calculation.hpp"
#include <px4_log.h>
using namespace matrix;

#define MODULE_NAME "my_control_allocator"

// 新增：定义6维输出权重（可通过set_parameters注入，示例值）


void AllocationCalculation::set_parameters(float servo_angle_limit_deg, const matrix::Matrix<float, 8, 8>& W)
{
    _W = W;
    _servo_angle_limit_rad = servo_angle_limit_deg * M_PI_F / 180.f;
    // 可选：初始化6维输出权重（示例：加大Roll/Pitch权重）
    W_u.setZero();
    A_T.setZero();
    A_W_T.setZero();
    A_W_A_T.setZero();
    A_W_pinv.setZero();
    b.setZero();
    F.setZero();
    alpha.setZero();
    alpha_clamped_rad.setZero();
    C.setZero();
    W_u(0,0) = 1.f; W_u(1,1) = 1.f; W_u(2,2) = 1.f;
    W_u(3,3) = 5.f; W_u(4,4) = 5.f; W_u(5,5) = 1.f;
}

void AllocationCalculation::solve_allocation(const matrix::Vector<float, 6>& U_target, const matrix::Matrix<float, 6, 8>& A, matrix::Vector<float, 8>& x_out) {
    x_out.setZero();
    // 保存当前x_out（用于求逆失败时回退）
    matrix::Vector<float, 8> x_prev = x_out;

    // 1. 计算加权伪逆 A_W† = W·A^T·(A·W·A^T)^{-1}
    A_T = A.transpose();
    A_W_T = _W * A_T;
    A_W_A_T = A * A_W_T;

    matrix::Matrix<float, 6, 6> A_W_A_T_inv;
    if (!matrix::geninv(A_W_A_T, A_W_A_T_inv)) {
        PX4_ERR("Matrix inversion failed (A_W_A_T)");
        x_out = x_prev; // 回退到上一次值
        return;
    }

    A_W_pinv = A_W_T * A_W_A_T_inv;

    // 2. 求解中间变量b（8维x/y推力分量）
    b = A_W_pinv * U_target;

    // 3. 极坐标变换：b → F, α
    for (int i = 0; i < 4; ++i) {
        float F_ix = b(2 * i);
        float F_iy = b(2 * i + 1);
        F(i) = std::sqrt(F_ix*F_ix + F_iy*F_iy);
        alpha(i) = std::atan2(F_iy, F_ix);
    }

    // 4. 角度限幅（简化版）
    for (int i = 0; i < 4; i++) {
        x_out(i) = F(i);
        // 手动舵机角度限幅（替代constrain函数）
        if (alpha(i) > _servo_angle_limit_rad) {
            alpha_clamped_rad(i) = _servo_angle_limit_rad;
        } else if (alpha(i) < -_servo_angle_limit_rad) {
            alpha_clamped_rad(i) = -_servo_angle_limit_rad;
        } else {
            alpha_clamped_rad(i) = alpha(i);
        }
        x_out(i + 4) = alpha_clamped_rad(i) * 180.f / M_PI_F; // 限幅后的角度写入x_out后4位
    }


    // // ========== 第二步：固定α_clamped，重新求解F ==========
    // // 构造8x4的系数矩阵C
    // C.setZero();
    // for (int i = 0; i < 4; i++) {
    //     C(2*i,   i) = std::cos(alpha_clamped_rad(i));
    //     C(2*i+1, i) = std::sin(alpha_clamped_rad(i));
    // }

    // // 构建M = A*C（6x4）
    // matrix::Matrix<float, 6, 4> M = A * C;
    // matrix::Matrix<float, 4, 6> M_T = M.transpose();

    // // 加权最小二乘求解：F = (M^T * W_u * M)^{-1} * M^T * W_u * U_target
    // matrix::Matrix<float, 4, 4> M_T_Wu_M = M_T * W_u * M;
    // matrix::Matrix<float, 4, 4> M_T_Wu_M_inv;

    // if (!matrix::geninv(M_T_Wu_M, M_T_Wu_M_inv)) {
    //     PX4_WARN("Constrained matrix inversion failed, use initial F");
    //     // 回退到初始F，并保证非负
    //     for (int i = 0; i < 4; i++) {
    //         if (F(i) < 0) {
    //             F(i) = 0; // 推力非负
    //         }
    //         x_out(i) = F(i); // 推力非负
    //     }
    //     return;
    // }

    // // 求解最优F
    // matrix::Vector<float, 4> F_clamped = M_T_Wu_M_inv * M_T * W_u * U_target;

    // // 推力非负限幅（物理约束）
    // for (int i = 0; i < 4; i++) {
    //     if (F_clamped(i) < 0) {
    //         F_clamped(i) = 0; // 推力非负
    //     }
    //     x_out(i) = F_clamped(i); // 写入最终推力
    // }
}
