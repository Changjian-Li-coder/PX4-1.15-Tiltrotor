/*
 * 简单串口读写模块：在 NuttX 上打开指定串口，定时发送一段文本并读取回传。
 * 用法：my_control start [-d <device>] [-b <baud>] [-m <msg>] [-r <rate_hz>]
 *       my_control stop
 *       my_control status
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/getopt.h>

#include <parameters/param.h>
#include <drivers/drv_hrt.h>

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>

#include <matrix/matrix/math.hpp>
#include <matrix/PseudoInverse.hpp>

#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/servo_control.h>

extern "C" __EXPORT int my_control_main(int argc, char *argv[]);

static int  uart_init(const char *uart_name);
static int  set_uart_baudrate(const int fd, unsigned int baud);
static void usage(void);
static int  task_main(int argc, char *argv[]);

static bool     _task_should_exit = true;
static bool     _is_running = false;
static int      _task_handle = -1;
static char     _device_name[32] = "/dev/ttyS2"; // 使用USART4串口
static unsigned _baudrate = 9600; // 默认波特率
// 不使用外部参数，保持简单的 start/stop/status 功能


static matrix::Matrix<float, 6, 8> matrix_A; // 控制分配矩阵
static matrix::Matrix<float, 8, 6> matrix_mix; // 伪逆矩阵
static matrix::Vector<float, 6> matrix_ctl; // 控制输入矩阵
static matrix::Vector<float, 8> matrix_b; // 四个转子 xy 轴的推力分量
static matrix::Vector<float, 4> matrix_F; // 四个电机产生的推力
static matrix::Vector<float, 4> matrix_a; // 四个舵机倾转的角度

static float L = 0.5; // 机臂长度
static float kf = 0.1; // 推力系数
static float k = 0.7071; // 45度的正弦余弦值
static constexpr float RAD_TO_DEG = 57.2957795f;
static void initialize_matrices()
{
	matrix_A(0, 0) = 0.f;    matrix_A(0, 1) = -k;     matrix_A(0, 2) = 0.f;    matrix_A(0, 3) =  k;
	matrix_A(0, 4) = 0.f;    matrix_A(0, 5) =  k;     matrix_A(0, 6) = 0.f;    matrix_A(0, 7) = -k;

	matrix_A(1, 0) = 0.f;    matrix_A(1, 1) =  k;     matrix_A(1, 2) = 0.f;    matrix_A(1, 3) = -k;
	matrix_A(1, 4) = 0.f;    matrix_A(1, 5) =  k;     matrix_A(1, 6) = 0.f;    matrix_A(1, 7) = -k;

	matrix_A(2, 0) = -1.f;   matrix_A(2, 1) = 0.f;    matrix_A(2, 2) = -1.f;   matrix_A(2, 3) = 0.f;
	matrix_A(2, 4) = -1.f;   matrix_A(2, 5) = 0.f;    matrix_A(2, 6) = -1.f;   matrix_A(2, 7) = 0.f;

	matrix_A(3, 0) = -k * L; matrix_A(3, 1) =  k * kf; matrix_A(3, 2) =  k * L; matrix_A(3, 3) = -k * kf;
	matrix_A(3, 4) =  k * L; matrix_A(3, 5) =  k * kf; matrix_A(3, 6) = -k * L; matrix_A(3, 7) = -k * kf;

	matrix_A(4, 0) =  k * L; matrix_A(4, 1) = -k * kf; matrix_A(4, 2) = -k * L; matrix_A(4, 3) =  k * kf;
	matrix_A(4, 4) =  k * L; matrix_A(4, 5) =  k * kf; matrix_A(4, 6) = -k * L; matrix_A(4, 7) = -k * kf;

	matrix_A(5, 0) =  k;     matrix_A(5, 1) =  L;     matrix_A(5, 2) =  k;     matrix_A(5, 3) =  L;
	matrix_A(5, 4) = -k;     matrix_A(5, 5) =  L;     matrix_A(5, 6) = -k;     matrix_A(5, 7) =  L;

	if (!matrix::geninv(matrix_A, matrix_mix)) {
		PX4_ERR("failed to compute pseudo-inverse for matrix_A");
	}
}
static void usage(void)
{
	PX4_INFO("usage: my_control {start|stop|status}");
}

static int set_uart_baudrate(const int fd, unsigned int baud)
{
	int speed = 0;

	switch (baud) {
	case 9600:   speed = B9600;   break;
	case 19200:  speed = B19200;  break;
	case 38400:  speed = B38400;  break;
	case 57600:  speed = B57600;  break;
	case 115200: speed = B115200; break;
	case 230400: speed = B230400; break;
	case 460800: speed = B460800; break;
	default:
		PX4_ERR("unsupported baudrate %u", baud);
		return -EINVAL;
	}

	struct termios uart_config; // 终端设备的配置结构体

	if (tcgetattr(fd, &uart_config) != 0) { //主要用于获取终端设备（包括串口）的当前配置属性
		PX4_ERR("tcgetattr failed");
		return -errno;
	}

	/* 原始模式，8N1，无流控 */
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

static int uart_init(const char *uart_name)// 打开串口设备
{
	int serial_fd = open(uart_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (serial_fd < 0) {
		PX4_ERR("open %s failed (%d)", uart_name, errno);
		return -errno;
	}

	return serial_fd;
}

static int task_main(int argc, char *argv[])
{
	int fd = uart_init(_device_name);
	initialize_matrices();

	if (fd < 0) {
		return -1;
	}

	if (set_uart_baudrate(fd, _baudrate) != 0) {
		close(fd);
		return -1;
	}


	uORB::Subscription _thrust_sub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Subscription _torque_sub{ORB_ID(vehicle_torque_setpoint)};


	vehicle_torque_setpoint_s vehicle_torque_setpoint;
	vehicle_thrust_setpoint_s vehicle_thrust_setpoint;

	while (!_task_should_exit) {
		if(_thrust_sub.updated()) {
			_thrust_sub.copy(&vehicle_thrust_setpoint);
		}
		if(_torque_sub.updated()) {
			_torque_sub.copy(&vehicle_torque_setpoint);
		}

		matrix_ctl(0) = vehicle_thrust_setpoint.xyz[0];
		matrix_ctl(1) = vehicle_thrust_setpoint.xyz[1];
		matrix_ctl(2) = vehicle_thrust_setpoint.xyz[2];
		matrix_ctl(3) = vehicle_torque_setpoint.xyz[0];
		matrix_ctl(4) = vehicle_torque_setpoint.xyz[1];
		matrix_ctl(5) = vehicle_torque_setpoint.xyz[2];

		// 计算分配后的执行器输入
		matrix_b = matrix_mix * matrix_ctl;
		for(int i = 0; i < 4; ++i) {
			const float bx = matrix_b(2 * i);
			const float by = matrix_b(2 * i + 1);
			matrix_F(i) = sqrtf(bx * bx + by * by);
			matrix_a(i) = atan2f(by, bx); // 弧度
		}
		// 归一化 matrix_a：以最大绝对值为基准，防止数值溢出（阈值可调整）
		{
			float max_abs = 0.0f;
			for (int i = 0; i < 4; ++i) {
				max_abs = fmaxf(max_abs, fabsf(matrix_a(i)));
			}
			if (max_abs > 1.0f) {
				matrix_a /= max_abs;
			}
		}

		// uint8_t servo_control_cmd[18] = {0x55,0x55,0x11,0x03,0x04,0xF4,0x01,0x01,0x00,0x00,0x02,0x00,0x00,0x03,0x00,0x00,0x04,0x00,0x00}; // 示例命令格式，需根据实际协议调整
		uint8_t servo_control_cmd[] = {0x55,0x55,0x0B,0x03,0x02,0xF4,0x01,0x01,0x00,0x00,0x02,0x00,0x00}; // 示例命令格式，需根据实际协议调整
		for (int i = 0; i < 2; ++i) {
			float deg = matrix_a(i) * RAD_TO_DEG; // 转度
			// 如果需要把负角映射到 0~240 或其他规则，在此修改；目前截断到 [0,240]
			if (deg < 0.0f) deg = 0.0f;
			if (deg > 240.0f) deg = 240.0f;
			int quant = (int)lroundf(deg / 240.0f * 1000.0f); // 0..1000
			if (quant < 0) quant = 0;
			if (quant > 1000) quant = 1000;

			uint8_t low_byte  = quant & 0xFF;         // 低 8 位
			uint8_t high_byte = (quant >> 8) & 0xFF;  // 高 8 位 (0..3)

			// 存放高/低字节（用于实际发送）
			servo_control_cmd[8 + 3*i]     = high_byte;
			servo_control_cmd[8 + 3*i + 1] = low_byte;
		}
		len = sizeof(servo_control_cmd);
		ssize_t written = write(fd, servo_control_cmd, len); // （根据实际协议调整长度）
		PX4_INFO("control_cmd: ID:%02X:, angle1: %d %d, ID%02X:, angle2: %d %d", servo_control_cmd[7], servo_control_cmd[8],servo_control_cmd[9],servo_control_cmd[10],servo_control_cmd[11],servo_control_cmd[12]);
		if (written < 0) {
			if (errno != EAGAIN && errno != EWOULDBLOCK) {
				PX4_WARN("uart write failed: %d", errno);
			}
		}
		usleep(1000000 / 10);
	}
	close(fd);
	return 0;
}

extern "C" __EXPORT int my_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return -EINVAL;
	}

	const char *verb = argv[1];

	if (!strcmp(verb, "start")) {
		if (_is_running) {
			PX4_INFO("already running");
			return 0;
		}

		// 直接启动任务（不解析外部参数），使用文件顶部的默认设备和波特率
		_task_should_exit = false;
		_task_handle = px4_task_spawn_cmd("my_control",
						  SCHED_DEFAULT,
						  SCHED_PRIORITY_DEFAULT,
						  8192,
						  task_main,
						  NULL);

		if (_task_handle < 0) {
			PX4_ERR("task start failed (%d)", errno);
			return -errno;
		}

		_is_running = true;
		PX4_INFO("started");
		return 0;
	}

	if (!strcmp(verb, "stop")) {
		if (!_is_running) {
			PX4_INFO("not running");
			return 0;
		}

		_task_should_exit = true;
		_is_running = false;
		PX4_INFO("stopping...");
		return 0;
	}

	if (!strcmp(verb, "status")) {
		PX4_INFO("%s", _is_running ? "running" : "stopped");
		return 0;
	}

	usage();
	return -EINVAL;
}
