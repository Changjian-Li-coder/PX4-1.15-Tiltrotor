#pragma once

#include <px4_platform_common/px4_config.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
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



	float sqrt2_2 = 0.7071f;
	float _rad_to_deg = 57.2957795f;


	// 参数句柄
	param_t _param_my_l = PARAM_INVALID;
	param_t _param_my_k = PARAM_INVALID;
	param_t _param_my_lambda_thrust = PARAM_INVALID;
	param_t _param_my_lambda_servo = PARAM_INVALID;
	param_t _param_my_thrust_min = PARAM_INVALID;
	param_t _param_my_thrust_max = PARAM_INVALID;
	param_t _param_my_ser_ang_lim = PARAM_INVALID;
	param_t _param_my_max_iter = PARAM_INVALID;
	param_t _param_my_learning_rate = PARAM_INVALID;

	// 参数缓存值
	float _L = 0.18f;  // 轴距365mm
	float _kf = 0.1f;
	float _tau_roll_max = _L * 4.f * sqrt2_2;
	float _tau_pitch_max = _L * 4.f * sqrt2_2;
	float _tau_yaw_max = _L * 4.f;
	float _lambda_thrust = 0.001f;
	float _lambda_servo = 0.5f;
	float _thrust_min = 0.1f;
	float _thrust_max = 0.9f;
	float _servo_angle_limit_deg = 22.5f;
	int32_t _max_iter = 10;
	float _learning_rate = 0.01f;

	vehicle_torque_setpoint_s _torque_sp;
	vehicle_thrust_setpoint_s _thrust_sp;
	uORB::Subscription _thrust_sub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Subscription _torque_sub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};

	AllocationCalculation _allocator;
};

