#pragma once

#include <px4_platform_common/px4_config.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

#include <matrix/matrix/math.hpp>

class MyControlAllocator
{
public:
	static int main(int argc, char *argv[]);

private:
	static int start();
	static int stop();
	static int status();
	static void usage();
	static int task_main(int argc, char *argv[]);
	static int run();

	static int uart_init(const char *uart_name);
	static int set_uart_baudrate(int fd, unsigned baud);
	static void initialize_matrices();
	static void get_control_matrix(const vehicle_thrust_setpoint_s &thrust, const vehicle_torque_setpoint_s &torque, matrix::Vector<float, 6> &control_matrix);
	static void normalization_thrust(matrix::Vector<float, 4> &motor_thrusts);
	static void normalization_servo_angle(matrix::Vector<float, 4> &servo_angles);
	static void allocation_calculation(const matrix::Vector<float, 6> &control_matrix, matrix::Vector<float, 4> &motor_thrusts, matrix::Vector<float, 4> &servo_angles);
	static void write_to_uart(const matrix::Vector<float, 4> &servo_angles, int fd);
	static void publish_actuator_motors(const matrix::Vector<float, 4> &motor_throttle);

	static bool _task_should_exit;
	static bool _is_running;
	static int _task_handle;
	static const char *_device_name; // 声明，不初始化
	static const unsigned _baudrate = 9600;

	static matrix::Matrix<float, 6, 8> _matrix_A; // 声明为静态成员
	static matrix::Matrix<float, 8, 6> _matrix_mix; // 声明为静态成员
	static matrix::Vector<float, 6> _matrix_ctl; // 声明为静态成员
	static matrix::Vector<float, 8> _matrix_b; // 声明为静态成员
	static matrix::Vector<float, 4> _matrix_F; // 声明为静态成员
	static matrix::Vector<float, 4> _matrix_a; // 声明为静态成员

	static constexpr float _L = 0.5f;
	static constexpr float _kf = 0.1f;
	static constexpr float _k = 0.7071f;
	static constexpr float _rad_to_deg = 57.2957795f;

	static vehicle_torque_setpoint_s _torque_sp; // 声明为静态成员
	static vehicle_thrust_setpoint_s _thrust_sp; // 声明为静态成员
	static uORB::Subscription _thrust_sub; // 声明为静态成员
	static uORB::Subscription _torque_sub; // 声明为静态成员
	static uORB::Publication<actuator_motors_s> _actuator_motors_pub; // 声明为静态成员，无需类内初始化

};

