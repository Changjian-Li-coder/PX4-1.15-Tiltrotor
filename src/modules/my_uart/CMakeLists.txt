set(MODULE_CFLAGS)
px4_add_module(
    MODULE modules__my_uart
    MAIN my_uart
    SRCS
        my_uart.cpp
    MODULE_CONFIG
        module.yaml
    DEPENDS
        parameters
)
