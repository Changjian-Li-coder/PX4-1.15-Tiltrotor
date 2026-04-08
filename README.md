V2.0.0--2026年4月8日更新

V2版本面向上位机NMPC控制发布控制指令

修改内容：
1.自定义了MyActuatorControl.msg
2.修改了mavlink_receiver.cpp/hpp，添加了对mavros/actuator_control话题消息的处理
3.修改了my_control_allocator.cpp/hpp，删减了订阅vehicle_thrust/torque_setpoints，增加了两种订阅上位机话题debug_array和actuator_control
