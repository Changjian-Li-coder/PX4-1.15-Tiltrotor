set(MODULE_CFLAGS)
px4_add_module(
    MODULE modules__my_uart_test
    MAIN my_uart_test
    SRCS my_uart_test.c

    )
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
