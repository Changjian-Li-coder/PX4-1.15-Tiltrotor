#pragma once

#include <px4_platform_common/px4_config.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/my_actuator_motors.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/hover_thrust_estimate.h>
#include <parameters/param.h>

#include <matrix/matrix/math.hpp>
#include "allocation_calculation.hpp"

class MyControlAllocator
{
public:
	static int main(int argc, char *argv[]);

private:
	static MyControlAllocator *_instance; // Singleton instance

	int start();
	int stop();
	int status();
	void usage();
	int task_main(int argc, char *argv[]);
	int run();

	int uart_init(const char *uart_name);
	int set_uart_baudrate(int fd, unsigned baud);
	void init();
	void initialize_matrices();
	void get_control_matrix(const vehicle_thrust_setpoint_s &thrust, const vehicle_torque_setpoint_s &torque, matrix::Vector<float, 6> &control_matrix);
	void normalization_thrust(matrix::Vector<float, 4> &motor_thrusts);
	void normalization_servo_angle(matrix::Vector<float, 4> &servo_angles);
	void write_to_uart(const matrix::Vector<float, 4> &servo_angles, int fd);
	void publish_actuator_motors(const matrix::Vector<float, 4> &motor_throttle);

	void log_data_at_2hz();
	void parameters_init();
	void parameters_update();
	void update_tau_max();

	bool _task_should_exit = false;
	bool _is_running = false;
	int _task_handle = -1;
	bool _is_initialized = false;
	const char *_device_name = "/dev/ttyS2";
	const unsigned _baudrate = 9600;

	matrix::Matrix<float, 6, 8> _matrix_A;
	matrix::Matrix<float, 8, 6> _matrix_mix;
	matrix::Vector<float, 6> _matrix_U;
	matrix::Vector<float, 8> _matrix_b;
	matrix::Vector<float, 4> _matrix_T;
	matrix::Vector<float, 4> _matrix_a;
	matrix::Vector<float, 8> _x_out;
	matrix::Vector<float, 8> _current_state;

	matrix::Vector<float, 6> _W;
	float sqrt2_2 = 0.7071f;

	// 参数句柄
	param_t _param_my_l = PARAM_INVALID;
	param_t _param_my_k = PARAM_INVALID;
	param_t _param_my_lambda_thrust = PARAM_INVALID;
	param_t _param_my_lambda_servo = PARAM_INVALID;
	param_t _param_my_thrust_min = PARAM_INVALID;
	param_t _param_my_thrust_max = PARAM_INVALID;
	param_t _param_my_ser_ang_lim = PARAM_INVALID;
	param_t _param_my_max_iter = PARAM_INVALID;
	param_t _param_my_lr_thrust = PARAM_INVALID;
	param_t _param_my_lr_servo = PARAM_INVALID;
	param_t _param_my_is_write = PARAM_INVALID;
	param_t _param_my_is_pub_ctl = PARAM_INVALID;
	param_t _param_my_write_time = PARAM_INVALID;
	param_t _param_my_servo_scale = PARAM_INVALID;
	param_t _param_my_w_xy = PARAM_INVALID;
	param_t _param_my_w_z = PARAM_INVALID;
	param_t _param_my_w_roll_pitch = PARAM_INVALID;
	param_t _param_my_w_yaw = PARAM_INVALID;
	param_t _param_my_CTL_ALL_RATE = PARAM_INVALID;

	// 参数缓存值
	float _L = 0.18f;  // 轴距365mm
	float _kf = 0.01f;
	float _tau_roll_max = _L * 4.f * sqrt2_2;
	float _tau_pitch_max = _L * 4.f * sqrt2_2;
	float _tau_yaw_max = _L * 4.f;
	float _lambda_thrust = 0.05f;
	float _lambda_servo = 0.1f;
	float _thrust_min = 0.1f;
	float _thrust_max = 0.9f;
	float _servo_angle_limit_deg = 22.5f;
	int32_t _max_iter = 15;
	float _lr_thrust = 0.01f;
	float _lr_servo = 0.4f;
	int32_t _is_write = 1;
	int32_t _is_pub_ctl = 1;
	int32_t _write_time = 50; // ms
	float _servo_scale = 1.0f;
	int32_t _ctl_all_rate = 100; // Hz

	vehicle_torque_setpoint_s _torque_sp;
	vehicle_thrust_setpoint_s _thrust_sp;
	hover_thrust_estimate_s _hover_thrust;
	uORB::Subscription _thrust_sub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Subscription _torque_sub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Subscription _hover_thrust_sub{ORB_ID(hover_thrust_estimate)};
	uORB::Publication<my_actuator_motors_s> _my_actuator_motors_pub{ORB_ID(my_actuator_motors)};

	AllocationCalculation _allocator;
};

