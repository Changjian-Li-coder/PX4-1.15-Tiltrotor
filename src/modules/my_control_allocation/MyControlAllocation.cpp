#include "MyControl.hpp"

#include <fcntl.h>
#include <math.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

MyControlAllocation::MyControlAllocation(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")),
	_vtol(vtol)
{
	_rate_param = param_find("SER_CTL_RATE");
	parameters_updated();
	// 如果有发布者，需要在这里初始化，例如：
	// _actuator_controls_pub.advertise();
}

MyControlAllocation::~MyControlAllocation()
{
	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}

	perf_free(_loop_perf);
}

bool MyControlAllocation::init()
{
	if (!_thrust_sub.registerCallback()) {
		PX4_ERR("thrust_sub callback registration failed");
		_thrust_sub.unregisterCallback();
		return false;
	}

	if (!_torque_sub.registerCallback()) {
		PX4_ERR("torque_sub callback registration failed");
		_torque_sub.unregisterCallback();
		return false;
	}

	initialize_matrices();

	if (!_matrix_initialized) {
		_torque_sub.unregisterCallback();
		_thrust_sub.unregisterCallback();
		return false;
	}

	_fd = uart_init(_device_name);

	if (_fd < 0) {
		_torque_sub.unregisterCallback();
		_thrust_sub.unregisterCallback();
		return false;
	}

	return true;
}

void MyControlAllocation::parameters_updated()
{
	if (_rate_param != PARAM_INVALID) {
		param_get(_rate_param, &_rate_hz);
	}

	if (_rate_hz < 1) {
		_rate_hz = 1;
	}

	_min_update_interval_us = 1000000 / static_cast<hrt_abstime>(_rate_hz);
}

void MyControlAllocation::initialize_matrices()
{
	_matrix_A(0, 0) = 0.f;    _matrix_A(0, 1) = -k;     _matrix_A(0, 2) = 0.f;    _matrix_A(0, 3) =  k;
	_matrix_A(0, 4) = 0.f;    _matrix_A(0, 5) =  k;     _matrix_A(0, 6) = 0.f;    _matrix_A(0, 7) = -k;

	_matrix_A(1, 0) = 0.f;    _matrix_A(1, 1) =  k;     _matrix_A(1, 2) = 0.f;    _matrix_A(1, 3) = -k;
	_matrix_A(1, 4) = 0.f;    _matrix_A(1, 5) =  k;     _matrix_A(1, 6) = 0.f;    _matrix_A(1, 7) = -k;

	_matrix_A(2, 0) = -1.f;   _matrix_A(2, 1) = 0.f;    _matrix_A(2, 2) = -1.f;   _matrix_A(2, 3) = 0.f;
	_matrix_A(2, 4) = -1.f;   _matrix_A(2, 5) = 0.f;    _matrix_A(2, 6) = -1.f;   _matrix_A(2, 7) = 0.f;

	_matrix_A(3, 0) = -k * L; _matrix_A(3, 1) =  k * kf; _matrix_A(3, 2) =  k * L; _matrix_A(3, 3) = -k * kf;
	_matrix_A(3, 4) =  k * L; _matrix_A(3, 5) =  k * kf; _matrix_A(3, 6) = -k * L; _matrix_A(3, 7) = -k * kf;

	_matrix_A(4, 0) =  k * L; _matrix_A(4, 1) = -k * kf; _matrix_A(4, 2) = -k * L; _matrix_A(4, 3) =  k * kf;
	_matrix_A(4, 4) =  k * L; _matrix_A(4, 5) =  k * kf; _matrix_A(4, 6) = -k * L; _matrix_A(4, 7) = -k * kf;

	_matrix_A(5, 0) =  k;     _matrix_A(5, 1) =  L;      _matrix_A(5, 2) =  k;     _matrix_A(5, 3) =  L;
	_matrix_A(5, 4) = -k;     _matrix_A(5, 5) =  L;      _matrix_A(5, 6) = -k;     _matrix_A(5, 7) =  L;

	_matrix_initialized = matrix::geninv(_matrix_A, _matrix_mix);

	if (!_matrix_initialized) {
		PX4_ERR("failed to compute pseudo-inverse for matrix_A");
	}
}

int MyControlAllocation::uart_init(const char *uart_name)
{
	const int serial_fd = ::open(uart_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (serial_fd < 0) {
		PX4_ERR("open %s failed (%d)", uart_name, errno);
		return -errno;
	}

	return serial_fd;
}

void MyControlAllocation::get_control_matrix(const vehicle_thrust_setpoint_s &thrust,
		const vehicle_torque_setpoint_s &torque,
		matrix::Vector<float, 6> &control_matrix)
{
	control_matrix(0) = thrust.xyz[0];
	control_matrix(1) = thrust.xyz[1];
	control_matrix(2) = thrust.xyz[2];
	control_matrix(3) = torque.xyz[0];
	control_matrix(4) = torque.xyz[1];
	control_matrix(5) = torque.xyz[2];
}

void MyControlAllocation::allocation_calculation(const matrix::Vector<float, 6> &control_matrix,
		matrix::Vector<float, 4> &motor_thrusts,
		matrix::Vector<float, 4> &servo_angles)
{
	_matrix_b = _matrix_mix * control_matrix;

	for (int i = 0; i < 4; ++i) {
		const float bx = _matrix_b(2 * i);
		const float by = _matrix_b(2 * i + 1);
		motor_thrusts(i) = sqrtf(bx * bx + by * by);
		servo_angles(i) = atan2f(by, bx);
	}
	// 归一化 servo_angles：以最大绝对值为基准，防止数值溢出（阈值可调整）
	{
		float max_abs = 0.0f;
		for (int i = 0; i < 4; ++i) {
			max_abs = fmaxf(max_abs, fabsf(servo_angles(i)));
		}
		if (max_abs > 1.0f) {
			servo_angles /= max_abs;
		}
	}
}

void MyControlAllocation::write_to_uart(const matrix::Vector<float, 4> &servo_angles, int fd)
{
	if (fd < 0) {
		return;
	}

	uint8_t servo_control_cmd[] = {0x55, 0x55, 0x0B, 0x03, 0x02, 0xF4, 0x01, 0x01, 0x00, 0x00, 0x02, 0x00, 0x00};

	for (int i = 0; i < 2; ++i) {
		float deg = servo_angles(i) * RAD_TO_DEG;

		if (deg < 0.0f) {
			deg = 0.0f;
		}

		if (deg > 240.0f) {
			deg = 240.0f;
		}

		int quant = static_cast<int>(lroundf(deg / 240.0f * 1000.0f));

		if (quant < 0) {
			quant = 0;
		}

		if (quant > 1000) {
			quant = 1000;
		}

		servo_control_cmd[8 + 3 * i] = static_cast<uint8_t>((quant >> 8) & 0xFF);
		servo_control_cmd[8 + 3 * i + 1] = static_cast<uint8_t>(quant & 0xFF);
	}

	const size_t len = sizeof(servo_control_cmd);
	const ssize_t written = ::write(fd, servo_control_cmd, len);
	PX4_INFO("control_cmd: ID:%02X:, angle1: %d %d, ID%02X:, angle2: %d %d", servo_control_cmd[7], servo_control_cmd[8],servo_control_cmd[9],servo_control_cmd[10],servo_control_cmd[11],servo_control_cmd[12]);
	if (written < 0) {
		if (errno != EAGAIN && errno != EWOULDBLOCK) {
			PX4_WARN("uart write failed: %d", errno);
		}

	} else if (written != static_cast<ssize_t>(len)) {
		PX4_WARN("uart write truncated: %d/%d", static_cast<int>(written), static_cast<int>(len));
	}
}

void MyControlAllocation::Run()
{
	if (should_exit()) {
		_thrust_sub.unregisterCallback();
		_torque_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}
	PX4_INFO("my_control_allocation is running, waiting for updates...");
	perf_begin(_loop_perf);

	const bool thrust_updated = _thrust_sub.update(&_vehicle_thrust_setpoint);
	const bool torque_updated = _torque_sub.update(&_vehicle_torque_setpoint);
	const hrt_abstime now = hrt_absolute_time();

	if (_matrix_initialized && _fd >= 0 && (thrust_updated || torque_updated)
	    && (now - _last_uart_write >= _min_update_interval_us)) {
		get_control_matrix(_vehicle_thrust_setpoint, _vehicle_torque_setpoint, _matrix_ctl);
		allocation_calculation(_matrix_ctl, _motor_thrusts, _servo_angles);
		write_to_uart(_servo_angles, _fd);
		_last_uart_write = now;
	}

	perf_end(_loop_perf);
}

int MyControlAllocation::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1 && strcmp(argv[1], "vtol") == 0) {
		vtol = true;
	}

	MyControl *instance = new MyControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MyControlAllocation::custom_command(int argc, char *argv[])
{
	(void)argc;
	(void)argv;
	return print_usage("unknown command");
}

int MyControlAllocation::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module reads vehicle thrust and torque setpoints,
computes tilt servo commands using a predefined control allocation matrix,
and sends the result through UART.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("my_control_allocation", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int my_control_allocation_main(int argc, char *argv[])
{
	return MyControl::main(argc, argv);
}
