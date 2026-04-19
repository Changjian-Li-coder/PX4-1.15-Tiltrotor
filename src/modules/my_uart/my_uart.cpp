/*
 * 简单串口读写模块：在 NuttX 上打开指定串口，定时发送一段文本并读取回传。
 * 用法：my_uart start [-d <device>] [-b <baud>] [-m <msg>] [-r <rate_hz>]
 *       my_uart stop
 *       my_uart status
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

#include <uORB/uORB.h>
#include <uORB/topics/my_uart_rx.h>
#include <uORB/topics/my_uart_tx.h>
#include <uORB/topics/my_uart_test.h>

extern "C" __EXPORT int my_uart_main(int argc, char *argv[]);

static int  uart_init(const char *uart_name);
static int  set_uart_baudrate(const int fd, unsigned int baud);
static void usage(void);
static int  task_main(int argc, char *argv[]);

static bool     _task_should_exit = true;
static bool     _is_running = false;
static int      _task_handle = -1;
static char     _device_name[32] = "/dev/ttyS2"; // 使用USART4串口
static unsigned _baudrate = 9600; // 默认波特率
static char     _tx_msg[64] = "UART Testing\r\n";
static unsigned _tx_rate_hz = 10; // 查看舵机角度默认频率

static const uint8_t _servo_angle_query_cmd[] = {0x55, 0x55, 0x05, 0x15, 0x02, 0x01, 0x02};

static void usage(void)
{
    PX4_INFO("usage: my_uart {start|stop|status} [-d device] [-b baud] [-m msg] [-r rate_hz]");
    PX4_INFO("defaults: -d %s -b %u -m '%s' -r %u", _device_name, _baudrate, _tx_msg, _tx_rate_hz);
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
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

    if (serial_fd < 0) {
        PX4_ERR("open %s failed (%d)", uart_name, errno);
        return -errno;
    }

    return serial_fd;
}

static int task_main(int argc, char *argv[])
{
    int fd = uart_init(_device_name);

    if (fd < 0) {
        return -1;
    }

    if (set_uart_baudrate(fd, _baudrate) != 0) {
        close(fd);
        return -1;
    }

    PX4_INFO("UART open on %s @ %u", _device_name, _baudrate);

    const int timeout_ms = 50; // poll 超时时间
    uint64_t next_tx = hrt_absolute_time();

    const int test_sub_fd = orb_subscribe(ORB_ID(my_uart_test));
    struct pollfd fds[1];
    memset(fds, 0, sizeof(fds));

    if (test_sub_fd >= 0) {
        fds[0].fd = test_sub_fd; // 仅监视 uORB 订阅的文件描述符
        fds[0].events = POLLIN;
    }

    orb_advert_t rx_pub = NULL;
    struct my_uart_test_s _servo_move_cmd = {};
    struct my_uart_rx_s rx_msg = {};
    uint8_t rx_buf[64] = {};

    int32_t tx_rate_param = _tx_rate_hz;
    const param_t my_uart_rate_handle = param_find("MY_UART_RATE");

    if (my_uart_rate_handle != PARAM_INVALID) {
        param_get(my_uart_rate_handle, &tx_rate_param);
    }

    if (tx_rate_param <= 0) {
        tx_rate_param = 1;
    }

    uint64_t tx_interval_us = 1000000u / (uint32_t)tx_rate_param;

    while (!_task_should_exit) {
        int pret = poll(fds, 1, timeout_ms);

        if (pret > 0 && test_sub_fd >= 0 && (fds[0].revents & POLLIN)) {
            bool updated = false;

            if (orb_check(test_sub_fd, &updated) == PX4_OK && updated) {
                orb_copy(ORB_ID(my_uart_test), test_sub_fd, &_servo_move_cmd);

                size_t len = _servo_move_cmd.data_length;

                if (len > sizeof(_servo_move_cmd.data)) {
                    len = sizeof(_servo_move_cmd.data);
                }

                if (len > 0) {
                    // 通过串口发送接收到的数据
                    ssize_t ret = write(fd, _servo_move_cmd.data, len);
                    if (ret > 0) {
                        PX4_INFO("Len: (%d), Content (hex): %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X", (int)len,
                                 _servo_move_cmd.data[0], _servo_move_cmd.data[1], _servo_move_cmd.data[2], _servo_move_cmd.data[3], _servo_move_cmd.data[4], _servo_move_cmd.data[5], _servo_move_cmd.data[6],
                                 _servo_move_cmd.data[7], _servo_move_cmd.data[8], _servo_move_cmd.data[9], _servo_move_cmd.data[10], _servo_move_cmd.data[11], _servo_move_cmd.data[12]);
                    }

                    if (ret < 0) {
                        PX4_ERR("UART write failed (%d)", errno);
                    }
                }
            }
        }
	// 定时发送查询舵机角度的命令
	const uint64_t now = hrt_absolute_time();

        if (now >= next_tx) {
            ssize_t ret = write(fd, _servo_angle_query_cmd, sizeof(_servo_angle_query_cmd));
	    if (ret < 0) {
		PX4_ERR("UART write failed (%d)", errno);
	    }
            // 接收部分代码
            ssize_t rret = read(fd, rx_buf, sizeof(rx_buf));

            if (rret > 0) {
                rx_msg.timestamp = hrt_absolute_time();

                size_t copy_len = (size_t)rret;

                if (copy_len > sizeof(rx_msg.data)) {
                    copy_len = sizeof(rx_msg.data);
                }

                memset(rx_msg.data, 0, sizeof(rx_msg.data));
                memcpy(rx_msg.data, rx_buf, copy_len);
                rx_msg.data_length = (uint16_t)copy_len;

                PX4_INFO("RX Message Data: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X",
                         rx_msg.data[0], rx_msg.data[1], rx_msg.data[2], rx_msg.data[3], rx_msg.data[4],
                         rx_msg.data[5], rx_msg.data[6], rx_msg.data[7], rx_msg.data[8], rx_msg.data[9],
                         rx_msg.data[10]);

                if (rx_pub == NULL) {
                    rx_pub = orb_advertise(ORB_ID(my_uart_rx), &rx_msg);
                } else {
                    orb_publish(ORB_ID(my_uart_rx), rx_pub, &rx_msg);
                }
            } else if (rret < 0) {
                PX4_ERR("UART read failed (%d)", errno);
            }

            next_tx += tx_interval_us;
        }

        if (my_uart_rate_handle != PARAM_INVALID) {
            param_get(my_uart_rate_handle, &tx_rate_param);
        }

        if (tx_rate_param <= 0) {
            tx_rate_param = 1;
        }

        tx_interval_us = 1000000u / (uint32_t)tx_rate_param;
    }

    if (test_sub_fd >= 0) {
        orb_unsubscribe(test_sub_fd);
    }

    if (rx_pub != NULL) {
        orb_unadvertise(rx_pub);
    }

    close(fd);
    PX4_INFO("UART task exiting");
    return 0;
}

extern "C" __EXPORT int my_uart_main(int argc, char *argv[])
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

        int32_t param_baud = 0;
        param_get(param_find("SERV_UART_BAUD"), &param_baud);

        if (param_baud > 0) {
            _baudrate = (unsigned)param_baud;
        }

        int ch;
        int myoptind = 1;
        const char *myoptarg = NULL;

        while ((ch = px4_getopt(argc, argv, "d:b:m:r:", &myoptind, &myoptarg)) != EOF) {
            switch (ch) {
            case 'd':
                strncpy(_device_name, myoptarg, sizeof(_device_name) - 1);
                _device_name[sizeof(_device_name) - 1] = '\0';
                break;

            case 'b':
                _baudrate = (unsigned)strtoul(myoptarg, NULL, 10);
                break;

            case 'm':
                strncpy(_tx_msg, myoptarg, sizeof(_tx_msg) - 1);
                _tx_msg[sizeof(_tx_msg) - 1] = '\0';
                break;

            case 'r':
                _tx_rate_hz = (unsigned)strtoul(myoptarg, NULL, 10);
                if (_tx_rate_hz == 0) {
                    _tx_rate_hz = 1;
                }
                break;

            default:
                usage();
                return -EINVAL;
            }
        }

        _task_should_exit = false;
        _task_handle = px4_task_spawn_cmd("my_uart",
                                          SCHED_DEFAULT,
                                          SCHED_PRIORITY_DEFAULT,
                                          1500,
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
        PX4_INFO("dev=%s baud=%u msg='%s' rate=%u Hz", _device_name, _baudrate, _tx_msg, _tx_rate_hz);
        return 0;
    }

    usage();
    return -EINVAL;
}
