px4_add_module(
    MODULE modules__my_control_allocation
    MAIN my_control_allocation
    SRCS
        MyControlAllocation.cpp
    MODULE_CONFIG
        module.yaml
    DEPENDS
        px4_work_queue
)
