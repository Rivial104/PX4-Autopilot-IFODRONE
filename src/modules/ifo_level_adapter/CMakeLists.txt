px4_add_module(
	MODULE modules__ifo_level_adapter
	MAIN ifo_level_adapter
	SRCS
		IfodroneLevelAdapter.cpp
		IfodroneLevelAdapter.hpp
	DEPENDS
		px4_work_queue
	)
