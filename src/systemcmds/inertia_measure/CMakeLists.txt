px4_add_module(
	MODULE systemcmds__inertia_measure
	MAIN inertia_measure
	STACK_MAIN 4096
	SRCS
		inertia_measure.cpp
	)
