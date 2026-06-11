px4_add_module(
	MODULE modules__ifo_rate_control
	MAIN ifo_rate_control
	SRCS
		IfodroneRateControl.cpp
		IfodroneRateControl.hpp
	DEPENDS
		px4_work_queue
	)
