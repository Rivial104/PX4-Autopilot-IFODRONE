
if(NOT TARGET AttitudeControl)
	add_subdirectory(
		${PX4_SOURCE_DIR}/src/modules/mc_att_control/AttitudeControl
		${CMAKE_CURRENT_BINARY_DIR}/AttitudeControl
	)
endif()

px4_add_module(
	MODULE modules__ifo_att_control
	MAIN ifo_att_control
	SRCS
		IfodroneAttitudeControl.cpp
		IfodroneAttitudeControl.hpp
	DEPENDS
		AttitudeControl
		px4_work_queue
	)
