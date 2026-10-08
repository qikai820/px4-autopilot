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
		DockGuidanceDynamic.cpp
		DockGuidanceRelative.cpp
		RelativeMotion.hpp
		RelativeResponse.hpp
		RelativeGuidance.hpp
		DockGuidance.hpp
		CameraHistory.hpp
		PredictiveCorrection.hpp
		RecoveryHeading.hpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		mathlib
		px4_work_queue
	)
