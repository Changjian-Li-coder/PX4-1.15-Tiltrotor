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
    A_T.setZero();
    A_W_T.setZero();
    A_W_A_T.setZero();
    A_W_pinv.setZero();
    b.setZero();
    F.setZero();
    alpha.setZero();
    alpha_clamped_rad.setZero();
    C.setZero();
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
    // ---------------------- 新增：反向验证分配误差 ----------------------
    // 1. 从x_out重构8维输入向量b_recon（还原推力的x/y分量）
    // matrix::Vector<float, 8> b_recon;
    // b_recon.setZero();
    // for (int i = 0; i < 4; ++i) {
    //     float F_i = x_out(i);                          // 第i个执行器的推力幅值
    //     float alpha_rad = x_out(i+4) * M_PI_F / 180.f; // 限幅后的角度转回弧度
    //     // 重构推力的x/y分量
    //     b_recon(2*i)   = F_i * std::cos(alpha_rad);
    //     b_recon(2*i+1) = F_i * std::sin(alpha_rad);
    // }

    // // 2. 反向计算实际输出U_recon = A * b_recon（分配矩阵正向计算）
    // matrix::Vector<float, 6> U_recon = A * b_recon;

    // // 3. 计算分配误差（各维度误差 + 整体范数误差）
    // matrix::Vector<float, 6> U_error = U_recon - U_target;
    // float error_norm = U_error.norm(); // 误差的2-范数（整体误差大小）

    // 4. 通过PX4_INFO输出误差信息
    // PX4_INFO("[Allocation Error] Norm: %.4f", (double)error_norm);
    // PX4_INFO("X-err: %.4f, Y-err: %.4f, Z-err: %.4f",(double)U_error(0), (double)U_error(1), (double)U_error(2));
    // PX4_INFO("Roll-err: %.4f, Pitch-err: %.4f, Yaw-err: %.4f",(double)U_error(3), (double)U_error(4), (double)U_error(5));
}
