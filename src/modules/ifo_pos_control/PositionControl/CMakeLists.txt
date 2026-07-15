############################################################################
#
#   Copyright (c) 2024 IFODRONE Project. All rights reserved.
#
############################################################################

px4_add_library(IfoPositionControl
	ControlMath.cpp
	ControlMath.hpp
	PositionControl.cpp
	PositionControl.hpp
)
target_include_directories(IfoPositionControl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

px4_add_unit_gtest(SRC IfodroneControlMathTest.cpp LINKLIBS IfoPositionControl)
