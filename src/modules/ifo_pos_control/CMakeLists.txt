############################################################################
#
#   Copyright (c) 2024 IFODRONE Project. All rights reserved.
#
############################################################################

add_subdirectory(PositionControl)
add_subdirectory(Takeoff)

px4_add_module(
	MODULE modules__ifo_pos_control
	MAIN ifo_pos_control
	COMPILE_FLAGS
	SRCS
		IfodronePositionControl.cpp
		IfodronePositionControl.hpp
	DEPENDS
		IfoPositionControl
		IfoTakeoff
		controllib
		geo
		mathlib
		hysteresis
	)
