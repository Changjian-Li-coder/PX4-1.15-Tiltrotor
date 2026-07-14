#pragma once

#include <matrix/math.hpp>

// C++ implementation of the optimization-based allocator
// ported from the original Python SLSQP prototype.
class AllocationCalculation
{
private:
	// 可调参数（由外部注入）

	matrix::Matrix<float, 8,8> _W;
	float _servo_angle_limit_rad; // 舵机角度限制（单位：弧度）

	matrix::Vector<float, 8> b;
	matrix::Matrix<float, 8, 6> A_T; // A的转置矩阵，用于加速计算
	matrix::Matrix<float, 8, 6> A_W_T;
	matrix::Matrix<float, 6, 6> A_W_A_T;
	matrix::Matrix<float, 8, 6> A_W_pinv;

	matrix::Vector<float, 4> F; // 推力大小
	matrix::Vector<float, 4> alpha; // 倾转角 （弧度）
	matrix::Vector<float, 4> alpha_clamped_rad; // 限幅后的角度（弧度）
	matrix::Matrix<float, 8, 4> C;


public:
	void set_parameters(float servo_angle_limit_deg, const matrix::Matrix<float, 8, 8>& W);
	void solve_allocation(const matrix::Vector<float, 6>& U_target, const matrix::Matrix<float, 6, 8>& A, matrix::Vector<float, 8>& x_out);
};
