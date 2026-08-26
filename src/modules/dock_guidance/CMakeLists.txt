############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
############################################################################

px4_add_module(
	MODULE modules__dock_guidance
	MAIN dock_guidance
	SRCS
		dock_guidance_main.cpp
		DockGuidance.cpp
		DockGuidance.hpp
	DEPENDS
		mathlib
		px4_work_queue
	)
