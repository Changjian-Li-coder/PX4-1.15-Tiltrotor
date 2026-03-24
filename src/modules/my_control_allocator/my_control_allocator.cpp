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

MyControlAllocator *MyControlAllocator::_instance = nullptr;

void MyControlAllocator::parameters_init()
{
	_param_my_l = param_find("MY_L");
	_param_my_k = param_find("MY_K");
	_param_my_lambda_thrust = param_find("MY_LAMBDA_THRUST");
	_param_my_lambda_servo = param_find("MY_LAMBDA_SERVO");
	_param_my_thrust_min = param_find("MY_THRUST_MIN");
	_param_my_thrust_max = param_find("MY_THRUST_MAX");
	_param_my_ser_ang_lim = param_find("MY_SER_ANG_LIM");
	_param_my_max_iter = param_find("MY_MAX_ITER");
	_param_my_learning_rate = param_find("MY_LEARNING_RATE");
}

void MyControlAllocator::update_tau_max()
{
	_tau_roll_max = _L * 4.f * sqrt2_2;
	_tau_pitch_max = _L * 4.f * sqrt2_2;
	_tau_yaw_max = _L * 4.f;
}

void MyControlAllocator::parameters_update()
{
	float l = _L;
	float k = _kf;
	float lambda_thrust = _lambda_thrust;
	float lambda_servo = _lambda_servo;
	float thrust_min = _thrust_min;
	float thrust_max = _thrust_max;
	float servo_angle_limit_deg = _servo_angle_limit_deg;
	int32_t max_iter = _max_iter;
	float learning_rate = _learning_rate;

	if (_param_my_l != PARAM_INVALID) {
		(void)param_get(_param_my_l, &l);
	}

	if (_param_my_k != PARAM_INVALID) {
		(void)param_get(_param_my_k, &k);
	}

	if (_param_my_lambda_thrust != PARAM_INVALID) {
		(void)param_get(_param_my_lambda_thrust, &lambda_thrust);
	}

	if (_param_my_lambda_servo != PARAM_INVALID) {
		(void)param_get(_param_my_lambda_servo, &lambda_servo);
	}

	if (_param_my_thrust_min != PARAM_INVALID) {
		(void)param_get(_param_my_thrust_min, &thrust_min);
	}

	if (_param_my_thrust_max != PARAM_INVALID) {
		(void)param_get(_param_my_thrust_max, &thrust_max);
	}

	if (_param_my_ser_ang_lim != PARAM_INVALID) {
		(void)param_get(_param_my_ser_ang_lim, &servo_angle_limit_deg);
	}

	if (_param_my_max_iter != PARAM_INVALID) {
		(void)param_get(_param_my_max_iter, &max_iter);
	}

	if (_param_my_learning_rate != PARAM_INVALID) {
		(void)param_get(_param_my_learning_rate, &learning_rate);
	}

	const bool geometry_changed = (fabsf(l - _L) > 1e-6f) || (fabsf(k - _kf) > 1e-6f);

	_L = l;
	_kf = k;
	_lambda_thrust = lambda_thrust;
	_lambda_servo = lambda_servo;
	_thrust_min = thrust_min;
	_thrust_max = thrust_max;
	_servo_angle_limit_deg = servo_angle_limit_deg;
	_max_iter = max_iter;
	_learning_rate = learning_rate;

	_allocator.set_parameters(_lambda_thrust, _lambda_servo, _thrust_min, _thrust_max,
				      _servo_angle_limit_deg, _max_iter, _learning_rate);

	if (geometry_changed) {
		initialize_matrices();
	}
}

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

void MyControlAllocator::init()
{
	parameters_init();
	parameters_update();

	for (int i = 0; i < 4; ++i) {
		_x_out(2 * i) = _thrust_min; // 给一个微小的初始推力，避免梯度为0
		_x_out(2 * i + 1) = 0.0f; // 舵机回中
	}

	update_tau_max();
	initialize_matrices();
}
void MyControlAllocator::initialize_matrices()
{
	_matrix_A(0, 0) = 0.f;    _matrix_A(0, 1) = -sqrt2_2;      _matrix_A(0, 2) = 0.f;    _matrix_A(0, 3) =  sqrt2_2;
	_matrix_A(0, 4) = 0.f;    _matrix_A(0, 5) =  sqrt2_2;      _matrix_A(0, 6) = 0.f;    _matrix_A(0, 7) = -sqrt2_2;

	_matrix_A(1, 0) = 0.f;    _matrix_A(1, 1) =  sqrt2_2;      _matrix_A(1, 2) = 0.f;    _matrix_A(1, 3) = -sqrt2_2;
	_matrix_A(1, 4) = 0.f;    _matrix_A(1, 5) =  sqrt2_2;      _matrix_A(1, 6) = 0.f;    _matrix_A(1, 7) = -sqrt2_2;

	_matrix_A(2, 0) = -1.f;   _matrix_A(2, 1) = 0.f;      _matrix_A(2, 2) = -1.f;   _matrix_A(2, 3) = 0.f;
	_matrix_A(2, 4) = -1.f;   _matrix_A(2, 5) = 0.f;      _matrix_A(2, 6) = -1.f;   _matrix_A(2, 7) = 0.f;

	_matrix_A(3, 0) = -sqrt2_2 * _L; _matrix_A(3, 1) = -sqrt2_2 * _kf; _matrix_A(3, 2) =  sqrt2_2 * _L; _matrix_A(3, 3) =  sqrt2_2 * _kf;
	_matrix_A(3, 4) =  sqrt2_2 * _L; _matrix_A(3, 5) =  sqrt2_2 * _kf; _matrix_A(3, 6) = -sqrt2_2 * _L; _matrix_A(3, 7) = -sqrt2_2 * _kf;

	_matrix_A(4, 0) =  sqrt2_2 * _L; _matrix_A(4, 1) =  sqrt2_2 * _kf; _matrix_A(4, 2) = -sqrt2_2 * _L; _matrix_A(4, 3) = -sqrt2_2 * _kf;
	_matrix_A(4, 4) =  sqrt2_2 * _L; _matrix_A(4, 5) =  sqrt2_2 * _kf; _matrix_A(4, 6) = -sqrt2_2 * _L; _matrix_A(4, 7) = -sqrt2_2 * _kf;

	_matrix_A(5, 0) =  -_kf;    _matrix_A(5, 1) =  _L;      _matrix_A(5, 2) =  -_kf;    _matrix_A(5, 3) =  _L;
	_matrix_A(5, 4) =   _kf;    _matrix_A(5, 5) =  _L;      _matrix_A(5, 6) =   _kf;    _matrix_A(5, 7) =  _L;

	if (!matrix::geninv(_matrix_A, _matrix_mix)) {
		PX4_ERR("failed to compute pseudo-inverse for matrix_A");
	}
}

void MyControlAllocator::get_control_matrix(const vehicle_thrust_setpoint_s &thrust, const vehicle_torque_setpoint_s &torque,
			matrix::Vector<float, 6> &control_matrix)
{
	control_matrix(0) = thrust.xyz[0] * 4;
	control_matrix(1) = thrust.xyz[1] * 4;
	control_matrix(2) = thrust.xyz[2] * 4;
	control_matrix(3) = torque.xyz[0] * _tau_roll_max;
	control_matrix(4) = torque.xyz[1] * _tau_pitch_max;
	control_matrix(5) = torque.xyz[2] * _tau_yaw_max;
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
void MyControlAllocator::log_data_at_2hz()
{
    static hrt_abstime last_log_time = 0;
    hrt_abstime now = hrt_absolute_time();

    if (now - last_log_time >= 500000) {
        last_log_time = now;

        PX4_INFO("vehicle_torque_setpoint: [%.2f, %.2f, %.2f]",
                 (double)_torque_sp.xyz[0],
                 (double)_torque_sp.xyz[1],
                 (double)_torque_sp.xyz[2]);

        PX4_INFO("vehicle_thrust_setpoint: [%.2f, %.2f, %.2f]",
                 (double)_thrust_sp.xyz[0],
                 (double)_thrust_sp.xyz[1],
                 (double)_thrust_sp.xyz[2]);

        PX4_INFO("actuator_motors: [%.2f, %.2f, %.2f, %.2f]",
                 (double)_matrix_T(0),
                 (double)_matrix_T(1),
                 (double)_matrix_T(2),
                 (double)_matrix_T(3));
	PX4_INFO("angle_servos: [%.2f, %.2f, %.2f, %.2f]",
                 (double)_matrix_a(0),
                 (double)_matrix_a(1),
                 (double)_matrix_a(2),
                 (double)_matrix_a(3));
    }
}

int MyControlAllocator::run()
{
	const int fd = uart_init(_device_name);

	if (fd < 0) {
		return fd;
	}

	if (set_uart_baudrate(fd, _baudrate) != 0) {
		::close(fd);
		return -1;
	}

	if (!_is_initialized) {
		init();
		_is_initialized = true;
	}

	while (!_task_should_exit) {

		if (_thrust_sub.updated()) {
			_thrust_sub.copy(&_thrust_sp);
		}

		if (_torque_sub.updated()) {
			_torque_sub.copy(&_torque_sp);
		}

		get_control_matrix(_thrust_sp, _torque_sp, _matrix_U);
		_allocator.solve_allocation(_matrix_U, _matrix_A, _x_out);
		for (int i = 0; i < 4; ++i) {
			_matrix_T(i) = _x_out(2 * i);
			_matrix_a(i) = _x_out(2 * i + 1);
		}
		//normalization_thrust(_matrix_T);
		//normalization_servo_angle(_matrix_a);
		//write_to_uart(_matrix_a, fd);
		log_data_at_2hz();

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
    if (_instance->_is_running) {
        PX4_INFO("already running");
        return 0;
    }

    _task_should_exit = false;
    _task_handle = px4_task_spawn_cmd("my_control_allocator",
                                      SCHED_DEFAULT,
                                      SCHED_PRIORITY_DEFAULT,
                                      8192,
                                      [](int argc, char *argv[]) { return _instance->task_main(argc, argv); },
                                      nullptr);

    if (_instance->_task_handle < 0) {
        PX4_ERR("task start failed (%d)", errno);
        delete _instance;
        _instance = nullptr;
        return -errno;
    }

    _instance->_is_running = true;
    PX4_INFO("started");
    return 0;
}

int MyControlAllocator::stop()
{
    if (_instance && !_instance->_is_running) {
        PX4_INFO("not running");
        return 0;
    }

    _instance->_task_should_exit = true;
    _instance->_is_running = false;
    PX4_INFO("stopping...");
    return 0;
}

int MyControlAllocator::status()
{
    PX4_INFO("%s", _instance->_is_running ? "running" : "stopped");
    return 0;
}

int MyControlAllocator::main(int argc, char *argv[])
{
    if (argc < 2) {
		PX4_INFO("usage: my_control_allocator {start|stop|status}");
        return -EINVAL;
    }

    if (!strcmp(argv[1], "start")) {
		if (_instance == nullptr) {
			_instance = new MyControlAllocator();
		}

		const int ret = _instance->start();

		if (ret != 0) {
			delete _instance;
			_instance = nullptr;
		}

		return ret;
    }

    if (!strcmp(argv[1], "stop")) {
		if (_instance == nullptr) {
			PX4_INFO("not running");
			return 0;
		}

        return _instance->stop();
    }

    if (!strcmp(argv[1], "status")) {
		if (_instance == nullptr) {
			PX4_INFO("stopped");
			return 0;
		}

        return _instance->status();
    }

	PX4_INFO("usage: my_control_allocator {start|stop|status}");
    return -EINVAL;
}

extern "C" __EXPORT int my_control_allocator_main(int argc, char *argv[])
{
    return MyControlAllocator::main(argc, argv);
}
