px4_add_library(IfoGotoControl
	GotoControl.cpp
	GotoControl.hpp
)
target_include_directories(IfoGotoControl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
