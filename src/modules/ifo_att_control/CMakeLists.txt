
px4_add_module(
	MODULE modules__ifo_att_control
	MAIN ifo_att_control
	SRCS
		IfoAttitudeControl.cpp
		IfoAttitudeControl.hpp
		${PX4_SOURCE_DIR}/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp
	INCLUDES
		${PX4_SOURCE_DIR}/src/modules/mc_att_control/AttitudeControl
	DEPENDS
		mathlib
		px4_work_queue
	)
