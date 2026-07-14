/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file my_control_params.c
 *
 * Parameters for my_control module
 */

#include <parameters/param.h>

/**
 * 机臂长度
 *
 * 定义四旋翼机臂的长度，单位为米。
 *
 * @unit m

 * @group My Control Allocator
 */
PARAM_DEFINE_FLOAT(MY_L, 0.182);

/**
 * 推力-反扭矩比例系数
 *
 * 定义推力与反扭矩之间的比例关系。
 *
 * @group My Control Allocator
 */
PARAM_DEFINE_FLOAT(MY_K, 0.01);


/**
 * 推力限幅的最小值
 *
 * 定义推力的最小限制值，单位为N。
 *
 * @group My Control Allocator
 */
PARAM_DEFINE_FLOAT(MY_THRUST_MIN, 0.1);

/**
 * 推力限幅的最大值
 *
 * 定义推力的最大限制值,单位为N。
 *
 * @group My Control Allocator
 */
PARAM_DEFINE_FLOAT(MY_THRUST_MAX, 20);

/**
 * 舵机角度限幅
 *
 * 定义舵机角度的最大限制值，单位为度。
 *
 * @unit deg
 * @group My Control Allocator
 */
PARAM_DEFINE_FLOAT(MY_SER_ANG_LIM, 22.5);


/**
 * 是否通过串口输出
 *
 * 定义是否通过串口输出。
 *
 * @group My Control Allocator
 */
PARAM_DEFINE_INT32(MY_IS_WRITE, 1);

/**
 * 是否publish actuator_controls
 *
 * 定义是否publish actuator_controls。
 *
 * @group My Control Allocator
 */
PARAM_DEFINE_INT32(MY_IS_PUB_CTL, 1);

/**
 * 舵机控制时间
 *
 * 定义舵机达到指定角度的时间。
 *
 * @unit ms
 * @group My Control Allocator
 */
PARAM_DEFINE_INT32(MY_WRITE_TIME, 5);

/**
 * 舵机角度解算结果缩放系数
 *
 * 定义舵机角度解算结果的缩放系数。
 *
 * @group My Control Allocator
 */
PARAM_DEFINE_FLOAT(MY_SERVO_SCALE, 1);

/**
 * XY平面的加权矩阵系数
 *
 * 定义XY平面的权重系数
 *
 * @group My Control Allocator
 */
PARAM_DEFINE_FLOAT(MY_W_XY, 1.0);

/**
 * Z方向的加权矩阵系数
 *
 * 定义Z方向的权重系数
 *
 * @group My Control Allocator
 */
PARAM_DEFINE_FLOAT(MY_W_Z, 1.0);

/**
 * control allocator的运行频率
 *
 * 定义control allocator的运行频率
 * @unit Hz
 * @group My Control Allocator
 */
PARAM_DEFINE_INT32(MY_CTL_ALL_RATE, 100);

/**
 * control servo的运行频率
 *
 * 定义control servo的频率
 * @unit Hz
 * @group My Control Allocator
 */
PARAM_DEFINE_INT32(MY_CTL_SER_RATE, 100);

/**
 * 舵机角度低通滤波比例系数
 *
 * alpha 取值范围 (0, 1]，值越小越平滑且响应越慢，1.0 表示无滤波。
 *
 * @group My Control Allocator
 */
PARAM_DEFINE_FLOAT(MY_SER_LP_A, 0.8);

/**
 * 舵机角度变化最小值限幅
 *
 * 舵机角度变化最小值限幅。

 * @group My Control Allocator
 */
PARAM_DEFINE_FLOAT(MY_SER_DEL_MIN, -0.8);

/**
 * 舵机角度变化最大值限幅
 *
 * 舵机角度变化最大值限幅。

 * @group My Control Allocator
 */
PARAM_DEFINE_FLOAT(MY_SER_DEL_MAX, 0.8);

/**
 * 上位机数据来源
 *
 * 0：DebugValue/send;1:actuator_control;2:setpoint_raw/attitude;else:PX4-PID

 * @group My Control Allocator
 */
PARAM_DEFINE_INT32(MY_RE_CHANNEL, 1);

/**
 * 是否输出log_data_at_2hz
 *
 * @group My Control Allocator
 */
PARAM_DEFINE_INT32(MY_IS_LOG, 0);

/**
 * 测试：舵机角度
 *
 * @group My Control Allocator
 */
PARAM_DEFINE_INT32(MY_SER_ANGLE, 0);

/**
 * 参数更新时间间隔
 *
 * @unit ms
 * @group My Control Allocator
 */
PARAM_DEFINE_INT32(MY_PARAM_UPDATE, 3000);
