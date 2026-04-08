# PX4 1.15版本中，不再支持actuator_control mavros话题
uint64 timestamp
uint8 group          # 0=主控制组
float32[8] controls  # 8路控制量
