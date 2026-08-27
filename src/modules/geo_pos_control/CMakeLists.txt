px4_add_module(
	MODULE modules__geo_pos_control
	MAIN geo_pos_control
	SRCS geo_pos_control.cpp geo_pos_control.hpp
	DEPENDS px4_work_queue
)
