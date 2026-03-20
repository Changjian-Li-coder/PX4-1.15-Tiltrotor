#include "my_control_allocator.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <drivers/drv_hrt.h>

#include <matrix/PseudoInverse.hpp>

#include <cerrno>
#include <cmath>
#include <float.h>
#include <cstring>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

const char *MyControlAllocator::_device_name = "/dev/ttyS2"; // 在类外初始化
uORB::Publication<actuator_motors_s> MyControlAllocator::_actuator_motors_pub{ORB_ID(actuator_motors)}; // 初始化静态成员
vehicle_torque_setpoint_s MyControlAllocator::_torque_sp{}; // 初始化静态成员
vehicle_thrust_setpoint_s MyControlAllocator::_thrust_sp{}; // 初始化静态成员
uORB::Subscription MyControlAllocator::_thrust_sub{ORB_ID(vehicle_thrust_setpoint)}; // 初始化静态成员
uORB::Subscription MyControlAllocator::_torque_sub{ORB_ID(vehicle_torque_setpoint)}; // 初始化静态成员
matrix::Matrix<float, 6, 8> MyControlAllocator::_matrix_A; // 初始化静态成员
matrix::Matrix<float, 8, 6> MyControlAllocator::_matrix_mix; // 初始化静态成员
matrix::Vector<float, 6> MyControlAllocator::_matrix_ctl; // 初始化静态成员
matrix::Vector<float, 8> MyControlAllocator::_matrix_b; // 初始化静态成员
matrix::Vector<float, 4> MyControlAllocator::_matrix_F; // 初始化静态成员
matrix::Vector<float, 4> MyControlAllocator::_matrix_a; // 初始化静态成员
bool MyControlAllocator::_task_should_exit = false; // 初始化静态成员
bool MyControlAllocator::_is_running = false; // 初始化静态成员
int MyControlAllocator::_task_handle = -1; // 初始化静态成员
int MyControlAllocator::uart_init(const char *uart_name)
{
	const int serial_fd = ::open(uart_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (serial_fd < 0) {
		PX4_ERR("open %s failed (%d)", uart_name, errno);
		return -errno;
	}

	return serial_fd;
}

int MyControlAllocator::set_uart_baudrate(int fd, unsigned baud)
{
	int speed = 0;

	switch (baud) {
	case 9600: speed = B9600; break;
	case 19200: speed = B19200; break;
	case 38400: speed = B38400; break;
	case 57600: speed = B57600; break;
	case 115200: speed = B115200; break;
	case 230400: speed = B230400; break;
	case 460800: speed = B460800; break;
	default:
		PX4_ERR("unsupported baudrate %u", baud);
		return -EINVAL;
	}

	struct termios uart_config {};

	if (tcgetattr(fd, &uart_config) != 0) {
		PX4_ERR("tcgetattr failed");
		return -errno;
	}

	cfmakeraw(&uart_config);
	uart_config.c_cflag |= (CLOCAL | CREAD);
	uart_config.c_cflag &= ~(CSTOPB | PARENB);
	uart_config.c_iflag &= ~(IXON | IXOFF | IXANY);

	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		PX4_ERR("cfsetspeed failed");
		return -errno;
	}

	if (tcsetattr(fd, TCSANOW, &uart_config) < 0) {
		PX4_ERR("tcsetattr failed");
		return -errno;
	}

	return 0;
}

void MyControlAllocator::initialize_matrices()
{
	_matrix_A(0, 0) = 0.f;    _matrix_A(0, 1) = -_k;      _matrix_A(0, 2) = 0.f;    _matrix_A(0, 3) =  _k;
	_matrix_A(0, 4) = 0.f;    _matrix_A(0, 5) =  _k;      _matrix_A(0, 6) = 0.f;    _matrix_A(0, 7) = -_k;

	_matrix_A(1, 0) = 0.f;    _matrix_A(1, 1) =  _k;      _matrix_A(1, 2) = 0.f;    _matrix_A(1, 3) = -_k;
	_matrix_A(1, 4) = 0.f;    _matrix_A(1, 5) =  _k;      _matrix_A(1, 6) = 0.f;    _matrix_A(1, 7) = -_k;

	_matrix_A(2, 0) = -1.f;   _matrix_A(2, 1) = 0.f;      _matrix_A(2, 2) = -1.f;   _matrix_A(2, 3) = 0.f;
	_matrix_A(2, 4) = -1.f;   _matrix_A(2, 5) = 0.f;      _matrix_A(2, 6) = -1.f;   _matrix_A(2, 7) = 0.f;

	_matrix_A(3, 0) = -_k * _L; _matrix_A(3, 1) =  _k * _kf; _matrix_A(3, 2) =  _k * _L; _matrix_A(3, 3) = -_k * _kf;
	_matrix_A(3, 4) =  _k * _L; _matrix_A(3, 5) =  _k * _kf; _matrix_A(3, 6) = -_k * _L; _matrix_A(3, 7) = -_k * _kf;

	_matrix_A(4, 0) =  _k * _L; _matrix_A(4, 1) = -_k * _kf; _matrix_A(4, 2) = -_k * _L; _matrix_A(4, 3) =  _k * _kf;
	_matrix_A(4, 4) =  _k * _L; _matrix_A(4, 5) =  _k * _kf; _matrix_A(4, 6) = -_k * _L; _matrix_A(4, 7) = -_k * _kf;

	_matrix_A(5, 0) =  _k;    _matrix_A(5, 1) =  _L;      _matrix_A(5, 2) =  _k;    _matrix_A(5, 3) =  _L;
	_matrix_A(5, 4) = -_k;    _matrix_A(5, 5) =  _L;      _matrix_A(5, 6) = -_k;    _matrix_A(5, 7) =  _L;

	if (!matrix::geninv(_matrix_A, _matrix_mix)) {
		PX4_ERR("failed to compute pseudo-inverse for matrix_A");
	}
}

void MyControlAllocator::get_control_matrix(const vehicle_thrust_setpoint_s &thrust, const vehicle_torque_setpoint_s &torque,
			matrix::Vector<float, 6> &control_matrix)
{
	control_matrix(0) = thrust.xyz[0];
	control_matrix(1) = thrust.xyz[1];
	control_matrix(2) = thrust.xyz[2];
	control_matrix(3) = torque.xyz[0];
	control_matrix(4) = torque.xyz[1];
	control_matrix(5) = torque.xyz[2];
}

void MyControlAllocator::allocation_calculation(const matrix::Vector<float, 6> &control_matrix, matrix::Vector<float, 4> &motor_thrusts, matrix::Vector<float, 4> &servo_angles)
{

	// Compute the mixed control inputs
	matrix::Vector<float, 8> mixed_controls = _matrix_mix * control_matrix;

	for (int i = 0; i < 4; ++i) {
		const float bx = mixed_controls(2 * i);
		const float by = mixed_controls(2 * i + 1);
		motor_thrusts(i) = sqrtf(bx * bx + by * by);
		servo_angles(i) = atan2f(by, bx);
	}
}

void MyControlAllocator::normalization_thrust(matrix::Vector<float, 4> &motor_thrusts)
{
	// Normalize motor thrusts to prevent overflow
	float max_thrust = 0.0f;
	for (int i = 0; i < 4; ++i) {
		max_thrust = fmaxf(max_thrust, motor_thrusts(i));
	}
	if (max_thrust > 1.0f) {
		motor_thrusts /= max_thrust;
	}
}

void MyControlAllocator::normalization_servo_angle(matrix::Vector<float, 4> &servo_angles)
{
	// Normalize servo angles to prevent overflow (threshold can be adjusted)
	float max_abs = 0.0f;
	for (int i = 0; i < 4; ++i) {
		max_abs = fmaxf(max_abs, fabsf(servo_angles(i)));
	}
	if (max_abs > 1.0f) {
		servo_angles /= max_abs;
	}
}

void MyControlAllocator::write_to_uart(const matrix::Vector<float, 4> &servo_angles, int fd)
{
	if (fd < 0) {
		return;
	}

	uint8_t servo_cmd[19] = {0x55,0x55,0x11,0x03,0x04,0xF4,0x01,0x01,0x00,0x00,0x02,0x00,0x00,0x03,0x00,0x00,0x04,0x00,0x00};

	for (int i = 0; i < 4; ++i) {
		float deg = servo_angles(i) * _rad_to_deg;
		if (deg < 0.f) { deg = 0.f; }
		if (deg > 240.f) { deg = 240.f; }

		int quant = static_cast<int>(lroundf(deg / 240.f * 1000.f));
		if (quant < 0) { quant = 0; }
		if (quant > 1000) { quant = 1000; }

		const uint8_t low_byte = quant & 0xFF;
		const uint8_t high_byte = (quant >> 8) & 0xFF;

		servo_cmd[8 + 3 * i] = high_byte;
		servo_cmd[8 + 3 * i + 1] = low_byte;
	}

	const ssize_t written = ::write(fd, servo_cmd, sizeof(servo_cmd));
	if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
		PX4_WARN("uart write failed: %d", errno);
	}
}

void MyControlAllocator::publish_actuator_motors(const matrix::Vector<float, 4> &motor_throttle)
{
	actuator_motors_s msg{};
	msg.timestamp = hrt_absolute_time();
	msg.timestamp_sample = msg.timestamp;

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
		msg.control[i] = NAN;
	}

	for (int i = 0; i < 4; ++i) {
		msg.control[i] = motor_throttle(i);
	}

	_actuator_motors_pub.publish(msg);
}

int MyControlAllocator::run()
{
	const int fd = uart_init(_device_name);

	if (set_uart_baudrate(fd, _baudrate) != 0) {
		::close(fd);
		return -1;
	}

	initialize_matrices();

	while (!_task_should_exit) {
		if (_thrust_sub.updated()) {
			_thrust_sub.copy(&_thrust_sp);
		}

		if (_torque_sub.updated()) {
			_torque_sub.copy(&_torque_sp);
		}

		get_control_matrix(_thrust_sp, _torque_sp, _matrix_ctl);
		allocation_calculation(_matrix_ctl, _matrix_F, _matrix_a);
		normalization_thrust(_matrix_F);
		normalization_servo_angle(_matrix_a);
		publish_actuator_motors(_matrix_F);
		write_to_uart(_matrix_a, fd);

		usleep(100000);
	}

	::close(fd);
	return 0;
}

int MyControlAllocator::task_main(int argc, char *argv[])
{
	(void)argc;
	(void)argv;
	return run();
}

void MyControlAllocator::usage()
{
	PX4_INFO("usage: my_control_allocator {start|stop|status}");
}

int MyControlAllocator::start()
{
	if (_is_running) {
		PX4_INFO("already running");
		return 0;
	}

	_task_should_exit = false;
	_task_handle = px4_task_spawn_cmd("my_control_allocator",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT,
					  8192,
					  task_main,
					  nullptr);

	if (_task_handle < 0) {
		PX4_ERR("task start failed (%d)", errno);
		return -errno;
	}

	_is_running = true;
	PX4_INFO("started");
	return 0;
}

int MyControlAllocator::stop()
{
	if (!_is_running) {
		PX4_INFO("not running");
		return 0;
	}

	_task_should_exit = true;
	_is_running = false;
	PX4_INFO("stopping...");
	return 0;
}

int MyControlAllocator::status()
{
	PX4_INFO("%s", _is_running ? "running" : "stopped");
	return 0;
}

int MyControlAllocator::main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return -EINVAL;
	}

	if (!strcmp(argv[1], "start")) {
		return start();
	}

	if (!strcmp(argv[1], "stop")) {
		return stop();
	}

	if (!strcmp(argv[1], "status")) {
		return status();
	}

	usage();
	return -EINVAL;
}

extern "C" __EXPORT int my_control_allocator_main(int argc, char *argv[])
{
	return MyControlAllocator::main(argc, argv);
}
