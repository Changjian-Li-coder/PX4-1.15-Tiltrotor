#pragma once

#include <px4_platform_common/px4_config.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/my_actuator_motors.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/my_actuator_control.h>


#include <parameters/param.h>

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>
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

	matrix::Matrix<float, 8, 8> _W;
	float sqrt2_2 = 0.7071f;

	// 参数句柄
	param_t _param_my_l = PARAM_INVALID;
	param_t _param_my_k = PARAM_INVALID;
	param_t _param_my_thrust_min = PARAM_INVALID;
	param_t _param_my_thrust_max = PARAM_INVALID;
	param_t _param_my_ser_ang_lim = PARAM_INVALID;
	param_t _param_my_is_write = PARAM_INVALID;
	param_t _param_my_is_pub_ctl = PARAM_INVALID;
	param_t _param_my_write_time = PARAM_INVALID;
	param_t _param_my_servo_scale = PARAM_INVALID;
	param_t _param_my_w_xy = PARAM_INVALID;
	param_t _param_my_w_z = PARAM_INVALID;
	param_t _param_my_ctl_all_rate = PARAM_INVALID;
	param_t _param_my_ctl_ser_rate = PARAM_INVALID;
	param_t _param_my_servo_lp_alpha = PARAM_INVALID;
	param_t _param_my_servo_delta_min = PARAM_INVALID;
	param_t _param_my_servo_delta_max = PARAM_INVALID;
	param_t _param_my_re_channel = PARAM_INVALID;
	param_t _param_my_is_log = PARAM_INVALID;


	// 参数缓存值
	float _L = 0.18f;  // 轴距365mm
	float _kf = 0.01f;
	float _tau_roll_max = _L * 2.f * sqrt2_2;
	float _tau_pitch_max = _L * 2.f * sqrt2_2;
	float _tau_yaw_max = _L * 4.f;
	float _thrust_min = 0.1f;
	float _thrust_max = 0.9f;
	float _servo_angle_limit_deg = 22.5f;
	int32_t _is_write = 1;
	int32_t _is_pub_ctl = 1;
	int32_t _write_time = 5; // ms
	float _servo_scale = 1.0f;
	int32_t _ctl_all_rate = 100; // Hz
	int32_t _ctl_ser_rate = 50; // Hz
	float _servo_lp_alpha = 0.8f;
	float _servo_delta_min = -0.15f;
	float _servo_delta_max = 0.15f;
	int32_t _re_channel = 0;
	int32_t _is_log = 0;


	vehicle_torque_setpoint_s _torque_sp;
	vehicle_thrust_setpoint_s _thrust_sp;

	debug_array_s _debug_array;
	my_actuator_control_s _my_actuator_control_sp;
	vehicle_rates_setpoint_s _rate_sp;

	uORB::Subscription _thrust_sub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Subscription _torque_sub{ORB_ID(vehicle_torque_setpoint)};
	//接收上位机数据
	uORB::Subscription _debug_array_sub{ORB_ID(debug_array)};
	uORB::Subscription _my_ctl_sub{ORB_ID(my_actuator_control)};
	uORB::Subscription _rate_sub{ORB_ID(vehicle_rates_setpoint)};

	uORB::Publication<my_actuator_motors_s> _my_actuator_motors_pub{ORB_ID(my_actuator_motors)};

	AllocationCalculation _allocator;
};
