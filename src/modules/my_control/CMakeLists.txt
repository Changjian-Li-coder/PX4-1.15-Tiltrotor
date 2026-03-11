set(MODULE_CFLAGS)
px4_add_module(
    MODULE modules__my_control
    MAIN my_control
    SRCS
        my_control.cpp
    MODULE_CONFIG
        module.yaml
)
