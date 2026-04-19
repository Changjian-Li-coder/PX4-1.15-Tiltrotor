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
	_param_my_thrust_min = param_find("MY_THRUST_MIN");
	_param_my_thrust_max = param_find("MY_THRUST_MAX");
	_param_my_ser_ang_lim = param_find("MY_SER_ANG_LIM");
	_param_my_is_write = param_find("MY_IS_WRITE");
	_param_my_is_pub_ctl = param_find("MY_IS_PUB_CTL");
	_param_my_write_time = param_find("MY_WRITE_TIME");
	_param_my_servo_scale = param_find("MY_SERVO_SCALE");
	_param_my_w_xy = param_find("MY_W_XY");
	_param_my_w_z = param_find("MY_W_Z");
	_param_my_ctl_all_rate = param_find("MY_CTL_ALL_RATE");
	_param_my_ctl_ser_rate = param_find("MY_CTL_SER_RATE");
	_param_my_servo_lp_alpha = param_find("MY_SER_LP_A");
	_param_my_servo_delta_min = param_find("MY_SER_DEL_MIN");
	_param_my_servo_delta_max = param_find("MY_SER_DEL_MAX");
	_param_my_re_channel = param_find("MY_RE_CHANNEL");
	_param_my_is_log = param_find("MY_IS_LOG");
}

void MyControlAllocator::update_tau_max()
{
	_tau_roll_max = 2.f * sqrt2_2 * _L;
	_tau_pitch_max = 2.f * sqrt2_2 * _L;
	_tau_yaw_max = 4.f * _L;
}

void MyControlAllocator::parameters_update()
{
	float l = _L;
	float k = _kf;
	float thrust_min = _thrust_min;
	float thrust_max = _thrust_max;
	float servo_angle_limit_deg = _servo_angle_limit_deg;
	int32_t is_write = 0;
	int32_t is_pub_ctl = 0;
	int32_t write_time = 500;
	float servo_scale = 1.0f;
	float w_xy = 1.0f;
	float w_z = 1.0f;
	int32_t ctl_all_rate = 100;
	int32_t ctl_ser_rate = 50;
	float servo_lp_alpha = 0.15f;
	float servo_delta_min = 0.01f;
	float servo_delta_max = 0.15f;
	int32_t re_channel = 0;
	int32_t is_log = 0;

	if (_param_my_l != PARAM_INVALID) {
		(void)param_get(_param_my_l, &l);
	}
	if (_param_my_k != PARAM_INVALID) {
		(void)param_get(_param_my_k, &k);
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
	if ( _param_my_is_write != PARAM_INVALID) {
		(void)param_get(_param_my_is_write, &is_write);
	}
	if ( _param_my_is_pub_ctl != PARAM_INVALID) {
		(void)param_get(_param_my_is_pub_ctl, &is_pub_ctl);
	}
	if ( _param_my_write_time != PARAM_INVALID) {
		(void)param_get(_param_my_write_time, &write_time);
	}
	if ( _param_my_servo_scale != PARAM_INVALID) {
		(void)param_get(_param_my_servo_scale, &servo_scale);
	}
	if ( _param_my_w_xy != PARAM_INVALID) {
		(void)param_get(_param_my_w_xy, &w_xy);
	}
	if ( _param_my_w_z != PARAM_INVALID) {
		(void)param_get(_param_my_w_z, &w_z);
	}
	if ( _param_my_ctl_all_rate != PARAM_INVALID) {
		(void)param_get(_param_my_ctl_all_rate, &ctl_all_rate);
	}
	if ( _param_my_ctl_ser_rate != PARAM_INVALID) {
		(void)param_get(_param_my_ctl_ser_rate, &ctl_ser_rate);
	}
	if ( _param_my_servo_lp_alpha != PARAM_INVALID) {
		(void)param_get(_param_my_servo_lp_alpha, &servo_lp_alpha);
	}
	if ( _param_my_servo_delta_min != PARAM_INVALID) {
		(void)param_get(_param_my_servo_delta_min, &servo_delta_min);
	}
	if ( _param_my_servo_delta_max != PARAM_INVALID) {
		(void)param_get(_param_my_servo_delta_max, &servo_delta_max);
	}
	if ( _param_my_re_channel != PARAM_INVALID) {
		(void)param_get(_param_my_re_channel, &re_channel);
	}
	if ( _param_my_is_log != PARAM_INVALID) {
		(void)param_get(_param_my_is_log, &is_log);
	}

	const bool geometry_changed = (fabsf(l - _L) > 1e-6f) || (fabsf(k - _kf) > 1e-6f);

	_L = l;
	_kf = k;
	_thrust_min = thrust_min;
	_thrust_max = thrust_max;
	_servo_angle_limit_deg = servo_angle_limit_deg;
	_is_write = is_write;
	_is_pub_ctl = is_pub_ctl;
	_write_time = write_time;
	_servo_scale = servo_scale;
	_ctl_all_rate = ctl_all_rate;
	_ctl_ser_rate = ctl_ser_rate;
	_servo_lp_alpha = servo_lp_alpha;
	_servo_delta_min = servo_delta_min;
	_servo_delta_max = servo_delta_max;
	_re_channel = re_channel;
	_is_log = is_log;


	if (_re_channel == 3) {
		_servo_scale = 1.0;
		_servo_delta_max = 0.10;
		_servo_delta_min = -0.10;
		_write_time = 4;
	}
	_W.setZero();
	_W(0,0) = w_xy;
	_W(1,1) = w_z;
	_W(2,2) = w_xy;
	_W(3,3) = w_z;
	_W(4,4) = w_xy;
	_W(5,5) = w_z;
	_W(6,6) = w_xy;
	_W(7,7) = w_z;
	_allocator.set_parameters(_servo_angle_limit_deg,_W);


	if (geometry_changed) {
		initialize_matrices();
		update_tau_max();
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

	_matrix_A(1, 0) = 0.f;    _matrix_A(1, 1) = -sqrt2_2;      _matrix_A(1, 2) = 0.f;    _matrix_A(1, 3) =  sqrt2_2;
	_matrix_A(1, 4) = 0.f;    _matrix_A(1, 5) = -sqrt2_2;      _matrix_A(1, 6) = 0.f;    _matrix_A(1, 7) =  sqrt2_2;

	_matrix_A(2, 0) = 1.f;   _matrix_A(2, 1) = 0.f;      _matrix_A(2, 2) = 1.f;   _matrix_A(2, 3) = 0.f;
	_matrix_A(2, 4) = 1.f;   _matrix_A(2, 5) = 0.f;      _matrix_A(2, 6) = 1.f;   _matrix_A(2, 7) = 0.f;

	_matrix_A(3, 0) = -sqrt2_2 * _L; _matrix_A(3, 1) =  sqrt2_2 * _kf; _matrix_A(3, 2) =  sqrt2_2 * _L; _matrix_A(3, 3) = -sqrt2_2 * _kf;
	_matrix_A(3, 4) =  sqrt2_2 * _L; _matrix_A(3, 5) =  sqrt2_2 * _kf; _matrix_A(3, 6) = -sqrt2_2 * _L; _matrix_A(3, 7) = -sqrt2_2 * _kf;

	_matrix_A(4, 0) = -sqrt2_2 * _L; _matrix_A(4, 1) =  sqrt2_2 * _kf; _matrix_A(4, 2) =  sqrt2_2 * _L; _matrix_A(4, 3) = -sqrt2_2 * _kf;
	_matrix_A(4, 4) = -sqrt2_2 * _L; _matrix_A(4, 5) = -sqrt2_2 * _kf; _matrix_A(4, 6) =  sqrt2_2 * _L; _matrix_A(4, 7) =  sqrt2_2 * _kf;

	_matrix_A(5, 0) =  -_kf;    _matrix_A(5, 1) = -_L;      _matrix_A(5, 2) = -_kf;    _matrix_A(5, 3) = -_L;
	_matrix_A(5, 4) =   _kf;    _matrix_A(5, 5) = -_L;      _matrix_A(5, 6) =  _kf;    _matrix_A(5, 7) = -_L;

	_matrix_a.setZero();
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

void MyControlAllocator::write_to_uart(const matrix::Vector<float, 4> &servo_angles, int fd)
{
    if (fd < 0) {
        return;
    }

    uint8_t servo_cmd[19] = {0x55,0x55,0x11,0x03,0x04,0xF4,0x01,0x01,0x00,0x00,0x02,0x00,0x00,0x03,0x00,0x00,0x04,0x00,0x00};

    // Convert _write_time to hexadecimal and assign to servo_cmd[5] and servo_cmd[6]
    uint16_t write_time_hex = static_cast<uint16_t>(_write_time);
    servo_cmd[5] = write_time_hex & 0xFF; // Low byte
    servo_cmd[6] = (write_time_hex >> 8) & 0xFF; // High byte

    for (int i = 0; i < 4; ++i) {
	int quant = (int)((servo_angles(i) / 120.0f) * 500.0f) + 500; // 将 -120~120 的舵机角度映射到 0~1000 的量化值 / 120
        const uint8_t low_byte = quant & 0xFF;
        const uint8_t high_byte = (quant >> 8) & 0xFF;

        servo_cmd[8 + 3 * i] = low_byte;
        servo_cmd[8 + 3 * i + 1] = high_byte;
    }

    const ssize_t written = ::write(fd, servo_cmd, sizeof(servo_cmd));
    if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        PX4_WARN("uart write failed: %d", errno);
    }
}

void MyControlAllocator::publish_actuator_motors(const matrix::Vector<float, 4> &motor_throttle)
{
	my_actuator_motors_s msg{};
	msg.timestamp = hrt_absolute_time();
	msg.timestamp_sample = msg.timestamp;

	for (int i = 0; i < 4; ++i) {
		msg.control[i] = motor_throttle(i);
	}
	for (int i = 4; i < my_actuator_motors_s::NUM_CONTROLS; ++i) {
		msg.control[i] = NAN;
	}

	_my_actuator_motors_pub.publish(msg);
}
void MyControlAllocator::log_data_at_2hz()
{
    static hrt_abstime last_log_time = 0;
    hrt_abstime now = hrt_absolute_time();

    if (now - last_log_time >= 500000) {
        last_log_time = now;
	PX4_INFO("=== Control Input ===");
        PX4_INFO("vehicle_thrust_setpoint: [%.4f, %.4f, %.4f]",
                 (double)_matrix_U(0),
                 (double)_matrix_U(1),
                 (double)_matrix_U(2));

	PX4_INFO("vehicle_torque_setpoint: [%.4f, %.4f, %.4f]",
                 (double)_matrix_U(3),
                 (double)_matrix_U(4),
                 (double)_matrix_U(5));

	PX4_INFO("===Allocation_Calculation Output===");
	PX4_INFO("x_out_motor: [%.4f, %.4f, %.4f, %.4f]",
		(double)_x_out(0),
		(double)_x_out(1),
		(double)_x_out(2),
		(double)_x_out(3));
	PX4_INFO("x_out_servo: [%.4f, %.4f, %.4f, %.4f]",
		(double)_x_out(4),
		(double)_x_out(5),
		(double)_x_out(6),
		(double)_x_out(7));
	PX4_INFO("===Publish Output===");
        PX4_INFO("motors_throttle: [%.4f, %.4f, %.4f, %.4f]",
                 (double)_matrix_T(0),
                 (double)_matrix_T(1),
                 (double)_matrix_T(2),
                 (double)_matrix_T(3));
	PX4_INFO("servos_angle: [%.4f, %.4f, %.4f, %.4f]",
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

	hrt_abstime last_time = hrt_absolute_time();
	hrt_abstime last_write_time = hrt_absolute_time();
	hrt_abstime last_param_update_time = hrt_absolute_time();

	const hrt_abstime interval = 1000000/ _ctl_all_rate; // 10 ms (100 Hz)
	const hrt_abstime write_interval = 1000000/ _ctl_ser_rate; // 50 ms (20 Hz)
	const hrt_abstime param_update_interval = 3000000; // 3 second (0.33 Hz)

	while (!_task_should_exit) {

		// 不需要接收thrust和torque的更新，控制量直接由上位机发送过来，解算交给上位机完成，这里只需要接收上位机发送的控制量即可
		if (_thrust_sub.updated()) {
			_thrust_sub.copy(&_thrust_sp);
		}

		if (_torque_sub.updated()) {
			_torque_sub.copy(&_torque_sp);
		}

		if (_my_ctl_sub.updated()) {
			_my_ctl_sub.copy(&_my_actuator_control_sp);
		}
		if (_debug_array_sub.updated()) {
			_debug_array_sub.copy(&_debug_array);
		}
		if (_rate_sub.updated()) {
			_rate_sub.copy(&_rate_sp);
		}

		if (_re_channel == 0){
			for (int i = 0; i < 6; ++i){
				_matrix_U(i) = _debug_array.data[i];
			}
		}
		else if (_re_channel == 1){
			for (int i =0; i < 6; ++i){
				_matrix_U(i) = _my_actuator_control_sp.controls[i];
			}
		}
		else if (_re_channel == 2){
			_matrix_U(0) = 0;
			_matrix_U(1) = 0;
			_matrix_U(2) = _rate_sp.thrust_body[2];
			_matrix_U(3) = _rate_sp.roll;
			_matrix_U(4) = _rate_sp.pitch;
			_matrix_U(5) = _rate_sp.yaw;
		}
		else if(_re_channel == 3){
			get_control_matrix(_thrust_sp, _torque_sp, _matrix_U);
			_matrix_U(3) = _matrix_U(3) / _tau_roll_max;
			_matrix_U(4) = _matrix_U(4) / _tau_pitch_max;
			_matrix_U(5) = _matrix_U(5) / _tau_yaw_max;
		}
		else{
			get_control_matrix(_thrust_sp, _torque_sp, _matrix_U);
		}

		// _x_out.setZero();
		_allocator.solve_allocation(_matrix_U, _matrix_A, _x_out);

		for (int j = 0; j < 4; ++j) {
			_matrix_T(j) = _x_out(j) / _thrust_max; // 归一化到0~1的油门值
			float target_a = _x_out(j + 4) * _servo_scale; // 应用舵机缩放系数

			float delta_angle = math::constrain(target_a - _matrix_a(j), _servo_delta_min, _servo_delta_max);
			// 舵机角度解算值的平滑处理 (低通滤波/指数移动平均)
			// alpha 取值范围 (0, 1]，值越小越平滑且响应越慢，1.0 表示无滤波。可根据运行频率调整
			_matrix_a(j) = (1.0f - _servo_lp_alpha) * _matrix_a(j) + _servo_lp_alpha * (delta_angle + _matrix_a(j));
			// _matrix_a(j) = 0;
		}

		if (_is_log) {
			log_data_at_2hz();
		}

		hrt_abstime now = hrt_absolute_time();
		if (now - last_param_update_time >= param_update_interval) {
			last_param_update_time = now;
			parameters_update();
		}
		if (now - last_time >= interval && _is_pub_ctl) {
			last_time = now;
			publish_actuator_motors(_matrix_T);
		}
		if (now - last_write_time >= write_interval && _is_write) {
			last_write_time = now;
			write_to_uart(_matrix_a, fd);
		}
		hrt_abstime sleep_time = interval - (now - last_time);
		if (sleep_time > 0) {
			px4_usleep(sleep_time); // PX4高精度休眠，不占用CPU
		}
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
