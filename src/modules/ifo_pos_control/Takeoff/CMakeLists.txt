############################################################################
#
#   Copyright (c) 2024 IFODRONE Project. All rights reserved.
#
############################################################################

px4_add_library(IfoTakeoff
	Takeoff.cpp
	Takeoff.hpp
)
target_include_directories(IfoTakeoff PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(IfoTakeoff PUBLIC hysteresis)
