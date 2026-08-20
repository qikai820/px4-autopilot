px4_add_module(
	MODULE modules__uorb_udp_bridge
	MAIN uorb_udp_bridge
	STACK_MAIN 1800
	SRCS
		uorb_udp_bridge.cpp
	DEPENDS
		px4_work_queue
)
