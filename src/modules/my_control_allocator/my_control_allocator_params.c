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
 * my_control 周期发送频率
 *
 * 控制 my_control 模块发送查询命令的频率。
 *
 * @unit Hz
 * @min 1
 * @max 200
 * @increment 1
 * @group My UART
 */
//PARAM_DEFINE_INT32(SER_CTL_RATE, 10);

/**
 * 机臂长度
 *
 * 定义四旋翼机臂的长度，单位为米。
 *
 * @unit m
 * @min 0.1
 * @max 0.5
 * @group My Control Allocator
 */
PARAM_DEFINE_FLOAT(MY_L, 0.182);

/**
 * 推力-反扭矩比例系数
 *
 * 定义推力与反扭矩之间的比例关系。
 *

 * @min 0.001
 * @max 0.1
 * @group My Control Allocator
 */
PARAM_DEFINE_FLOAT(MY_K, 0.01);

/**
 * 推力惩罚系数
 *
 * 定义优化过程中推力的惩罚权重。
 *

 * @min 0.0
 * @max 10.0
 * @group My Control Allocator
 */
PARAM_DEFINE_FLOAT(MY_LAMBDA_THRUST, 0.001);

/**
 * 舵机角度惩罚系数
 *
 * 定义优化过程中舵机角度的惩罚权重。
 *

 * @min 0.0
 * @max 10.0
 * @group My Control Allocator
 */
PARAM_DEFINE_FLOAT(MY_LAMBDA_SERVO, 0.5);

/**
 * 推力限幅的最小值
 *
 * 定义推力的最小限制值，单位为无量纲比例。
 *

 * @min 0.0
 * @max 1.0
 * @group My Control Allocator
 */
PARAM_DEFINE_FLOAT(MY_THRUST_MIN, 0.1);

/**
 * 推力限幅的最大值
 *
 * 定义推力的最大限制值，单位为无量纲比例。
 *

 * @min 0.0
 * @max 1.0
 * @group My Control Allocator
 */
PARAM_DEFINE_FLOAT(MY_THRUST_MAX, 0.9);

/**
 * 舵机角度限幅
 *
 * 定义舵机角度的最大限制值，单位为度。
 *
 * @unit deg
 * @min 0.0
 * @max 45.0
 * @group My Control Allocator
 */
PARAM_DEFINE_FLOAT(MY_SER_ANG_LIM, 22.5);

/**
 * 最大迭代次数
 *
 * 定义优化算法的最大迭代次数。
 *

 * @min 1
 * @max 100
 * @group My Control Allocator
 */
PARAM_DEFINE_INT32(MY_MAX_ITER, 10);

/**
 * 梯度下降步长
 *
 * 定义优化算法中梯度下降的步长。
 *

 * @min 0.001
 * @max 1.0
 * @group My Control Allocator
 */
PARAM_DEFINE_FLOAT(MY_LEARNING_RATE, 0.01);
