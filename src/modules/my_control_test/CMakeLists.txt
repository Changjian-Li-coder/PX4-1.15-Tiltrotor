set(MODULE_CFLAGS)
px4_add_module(
    MODULE modules__my_control_test
    MAIN my_control_test
    SRCS
        my_control_test.cpp
    MODULE_CONFIG
        module.yaml
)
