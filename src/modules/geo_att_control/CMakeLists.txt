px4_add_module(
	MODULE modules__geo_att_control
	MAIN geo_att_control
	SRCS geo_att_control.cpp geo_att_control.hpp
	DEPENDS px4_work_queue
)
