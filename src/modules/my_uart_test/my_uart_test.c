// Simple publisher module for testing: publish `my_uart_test` periodically.

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>

#include <uORB/uORB.h>
#include <uORB/topics/my_uart_test.h>

__EXPORT int my_uart_test_main(int argc, char *argv[]);

static bool _task_should_exit = true;
static bool _is_running = false;
static int _task_handle = -1;
int32_t rate_hz_param = 10;
static void usage(const char *prog)
{
    PX4_INFO("usage: %s {start|stop|status}", prog);
}

// 舵机运动控制参数
#define STEP_SIZE 5          // 每次角度变化步长（越小越丝滑）
#define MIN_ANGLE 0          // 最小角度
#define MAX_ANGLE 1000       // 最大角度
#define SERVO1_ID 1          // 舵机1 ID
#define SERVO2_ID 2          // 舵机2 ID

static int task_main(int argc, char *argv[])
{
    struct my_uart_test_s pub_msg = {};
    orb_advert_t test_pub = NULL;

    const uint64_t interval_us = 1000000u / rate_hz_param;
    uint64_t next = hrt_absolute_time();

    // 基础指令帧（前7字节固定）
    uint8_t hex_data[] = {0x55, 0x55, 0x0B, 0x03, 0x02, 0xC8, 0x00,
                          SERVO1_ID, 0x00, 0x00,  // 舵机1 ID+角度低8+高8
                          SERVO2_ID, 0x00, 0x00}; // 舵机2 ID+角度低8+高8

    // 角度控制变量
    int current_angle = MIN_ANGLE;
    int direction = STEP_SIZE;  // 角度变化方向（正为增加，负为减小）

    while (!_task_should_exit) {
        pub_msg.timestamp = hrt_absolute_time();

        // 更新当前角度（往返逻辑）
        current_angle += direction;
        // 到达最大值，开始减小
        if (current_angle >= MAX_ANGLE) {
            current_angle = MAX_ANGLE;
            direction = -STEP_SIZE;
        }
        // 到达最小值，开始增加
        else if (current_angle <= MIN_ANGLE) {
            current_angle = MIN_ANGLE;
            direction = STEP_SIZE;
        }

        // 将十进制角度转换为高低8位（1000的十六进制是0x03E8，对应低8位0xE8，高8位0x03）
        uint8_t angle_low = current_angle & 0xFF;    // 低8位
        uint8_t angle_high = (current_angle >> 8) & 0xFF; // 高8位

        // 更新舵机1角度
        hex_data[7] = SERVO1_ID;
        hex_data[8] = angle_low;
        hex_data[9] = angle_high;

        // 更新舵机2角度（可根据需求调整，这里和舵机1同步）
        hex_data[10] = SERVO2_ID;
        hex_data[11] = angle_low;
        hex_data[12] = angle_high;

        // 拷贝数据到发布结构体
        size_t sl = sizeof(hex_data);
        memset(pub_msg.data, 0, sizeof(pub_msg.data));
        memcpy(pub_msg.data, hex_data, sl);
        pub_msg.data_length = (uint16_t)sl;

        // 发布消息
        if (test_pub == NULL) {
            test_pub = orb_advertise(ORB_ID(my_uart_test), &pub_msg);
        } else {
            orb_publish(ORB_ID(my_uart_test), test_pub, &pub_msg);
        }

        // 精准延时，保证运动平滑
        next += interval_us;
        const uint64_t now = hrt_absolute_time();
        if (next > now) {
            px4_usleep(next - now);
        } else {
            next = now + interval_us;
        }
    }

    if (test_pub != NULL) {
        orb_unadvertise(test_pub);
    }

    return 0;
}

int my_uart_test_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage(argv[0]);
        return -EINVAL;
    }

    const char *verb = argv[1];

    if (!strcmp(verb, "start")) {
        if (_is_running) {
            PX4_INFO("already running");
            return 0;
        }

        _task_should_exit = false;
        _task_handle = px4_task_spawn_cmd("my_uart_test",
                                          SCHED_DEFAULT,
                                          SCHED_PRIORITY_DEFAULT,
                                          2000,  // 稍微增大栈空间
                                          task_main,
                                          NULL);

        if (_task_handle < 0) {
            PX4_ERR("task start failed (%d)", errno);
            return -errno;
        }

        _is_running = true;
        PX4_INFO("my_uart_test started");
        return 0;
    }

    if (!strcmp(verb, "stop")) {
        if (!_is_running) {
            PX4_INFO("not running");
            return 0;
        }

        _task_should_exit = true;
        _is_running = false;
        PX4_INFO("my_uart_test stopping...");
        // 等待任务退出
        px4_task_exit(_task_handle);
        _task_handle = -1;
        return 0;
    }

    if (!strcmp(verb, "status")) {
        PX4_INFO("%s", _is_running ? "running" : "stopped");
        return 0;
    }

    usage(argv[0]);
    return -EINVAL;
}
