#pragma once

#include <matrix/math.hpp>

// C++ implementation of the optimization-based allocator
// ported from the original Python SLSQP prototype.
class AllocationCalculation
{
private:
	// 可调参数（由外部注入）
	float _lambda_thrust = 0.001f;
	float _lambda_servo = 0.5f;
	float _thrust_min = 0.1f;
	float _thrust_max = 0.9f;
	float _servo_angle_limit_rad = M_PI_F / 8.0f;
	int _max_iter = 10;
	float _lr_thrust = 0.01f;
	float _lr_servo = 0.01f;

	matrix::Vector<float, 6> _W; // 误差权重向量
	matrix::Vector<float, 8> b;
	matrix::Vector<float, 6> error;
	matrix::Vector<float, 8> g_b;
	matrix::Vector<float, 8> grad;
	matrix::Matrix<float, 8, 6> A_T; // A的转置矩阵，用于加速计算

public:
	void set_parameters(float lambda_thrust, float lambda_servo, float thrust_min, float thrust_max,
				    float servo_angle_limit_deg, int max_iter, float lr_thrust, float lr_servo, const matrix::Vector<float, 6>& W);

	void solve_allocation(const matrix::Vector<float, 6>& U_target, const matrix::Matrix<float, 6, 8>& A, matrix::Vector<float, 8>& x_out);
};
