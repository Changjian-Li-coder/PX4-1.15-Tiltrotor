#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <matrix/PseudoInverse.hpp>
#include <parameters/param.h>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>

class MyControlAllocation : public ModuleBase<MyControlAllocation>, public ModuleParams, public px4::WorkItem
{
	public:
		MyControlAllocation(bool vtol = false);
		~MyControlAllocation() override;
		static int task_spawn(int argc, char *argv[]);
		static int custom_command(int argc, char *argv[]);
		static int print_usage(const char *reason = nullptr);

		bool init();

	private:
		void Run() override;

		int uart_init(const char *uart_name);
		void initialize_matrices();
		void parameters_updated();
		void get_control_matrix(const vehicle_thrust_setpoint_s &thrust, const vehicle_torque_setpoint_s &torque,
						matrix::Vector<float, 6> &control_matrix);
		void allocation_calculation(const matrix::Vector<float, 6> &control_matrix,
						   matrix::Vector<float, 4> &motor_thrusts,
						   matrix::Vector<float, 4> &servo_angles);
		void write_to_uart(const matrix::Vector<float, 4> &servo_angles, int fd);

		uORB::SubscriptionCallbackWorkItem _thrust_sub{this, ORB_ID(vehicle_thrust_setpoint)};
		uORB::SubscriptionCallbackWorkItem _torque_sub{this, ORB_ID(vehicle_torque_setpoint)};

		matrix::Matrix<float, 6, 8> _matrix_A{};
		matrix::Matrix<float, 8, 6> _matrix_mix{};
		matrix::Vector<float, 6> _matrix_ctl{};
		matrix::Vector<float, 8> _matrix_b{};
		matrix::Vector<float, 4> _motor_thrusts{};
		matrix::Vector<float, 4> _servo_angles{};

		vehicle_torque_setpoint_s _vehicle_torque_setpoint{};
		vehicle_thrust_setpoint_s _vehicle_thrust_setpoint{};

		perf_counter_t _loop_perf;
		int _fd{-1};
		bool _vtol{false};
		bool _matrix_initialized{false};
		param_t _rate_param{PARAM_INVALID};
		int32_t _rate_hz{10};
		hrt_abstime _min_update_interval_us{100000};
		hrt_abstime _last_uart_write{0};

		static constexpr float L = 0.5f;
		static constexpr float kf = 0.1f;
		static constexpr float k = 0.70710678f;
		static constexpr float RAD_TO_DEG = 57.2957795f;

		const char *_device_name{"/dev/ttyS1"};
};
