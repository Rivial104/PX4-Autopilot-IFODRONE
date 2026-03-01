############################################################################
#
#   Copyright (c) 2024 IFODRONE Project. All rights reserved.
#
############################################################################

px4_add_module(
	MODULE modules__ifo_pos_control
	MAIN ifo_pos_control
	SRCS
		IfoPositionControl.cpp
		IfoPositionControl.hpp
	DEPENDS
		mathlib
)
