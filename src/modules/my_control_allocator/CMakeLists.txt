set(MODULE_CFLAGS)
px4_add_module(
    MODULE modules__my_control_allocator
    MAIN my_control_allocator
    SRCS
        my_control_allocator.hpp
        my_control_allocator.cpp
)
