px4_add_module(
	MODULE drivers__jointcan
	MAIN joican
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
		#-DDEBUG_BUILD
	SRCS
		joican_raw_can_driver.cpp
		joican_raw_can_driver.hpp
		joican.cpp
		joican.hpp
		mit_protocol.hpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		mixer_module
		px4_work_queue
	)
