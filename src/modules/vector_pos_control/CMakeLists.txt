############################################################################
#
#   Copyright (c) 2015-2020 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

add_subdirectory(VectorGoto)
add_subdirectory(VectorPosition)
add_subdirectory(VectorTakeoff)

px4_add_module(
	MODULE modules__vector_pos_control
	MAIN vector_pos_control
	COMPILE_FLAGS
	SRCS
		VectorPositionControl.cpp
		VectorPositionControl.hpp
	DEPENDS
		VectorGoto
		VectorPosition
		VectorTakeoff
		controllib
		geo
		SlewRate
	)

# The complete copied wrappers retain the project's optional Dock hooks.
set(_dock_config ${CMAKE_CURRENT_SOURCE_DIR}/control_params.yaml)
get_property(_configs GLOBAL PROPERTY PX4_MODULE_CONFIG_FILES)
if(NOT CONFIG_MODULES_DOCK_GUIDANCE AND NOT _dock_config IN_LIST _configs)
 set_property(GLOBAL APPEND PROPERTY PX4_MODULE_CONFIG_FILES ${_dock_config})
endif()
