/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "DockGuidance.hpp"

extern "C" __EXPORT int dock_guidance_main(int argc, char *argv[])
{
	return DockGuidance::main(argc, argv);
}
