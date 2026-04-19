# Servo control message
uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample		# the timestamp the data this control response is based on was sampled

# 帧头：0x55 0x55
# 数据长度：data_length
uint8 data_length
# 控制指令：0x03
# 控制舵机数量：num_servo
uint8 num_servo
# 时间低八位time_low与时间高八位time_high，单位为毫秒
uint8 time_low
uint8 time_high
# 舵机id号：servo_id_1
uint8 servo_id_1
# 角度控制低八位angle_low_1与角度控制高八位angle_high_1，取值范围0~1000，对应角度0~240度
uint8 angle_low_1
uint8 angle_high_1
# 舵机id号：servo_id_2
uint8 servo_id_2
# 角度控制低八位angle_low_2与角度控制高八位angle_high_2，取值范围0~1000，对应角度0~240度
uint8 angle_low_2
uint8 angle_high_2
# 舵机id号：servo_id_3
uint8 servo_id_3
# 角度控制低八位angle_low_3与角度控制高八位angle_high_3，取值范围0~1000，对应角度0~240度
uint8 angle_low_3
uint8 angle_high_3
# 舵机id号：servo_id_4
uint8 servo_id_4
# 角度控制低八位angle_low_4与角度控制高八位angle_high_4，取值范围0~1000，对应角度0~240度
uint8 angle_low_4
uint8 angle_high_4


