

px4_add_module(
	MODULE modules__ifo_att_control
	MAIN ifo_att_control
	SRCS
		IfodroneAttitudeControl.cpp
		IfodroneAttitudeControl.hpp
	DEPENDS
		px4_work_queue
	)
