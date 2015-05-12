MODULE = uDC_module

ifeq ($(TEST),blinking_led)
	PRJ_CSRC += test/blinking_led.c
endif

ifeq ($(TEST),hardware_test)
	PACKAGES += led
	PRJ_CPPSRC += test/hardware_test.cpp
endif

ifeq ($(TEST),)
	PACKAGES += led
	PRJ_CPPSRC += main.cpp
endif

PRJ_INC = .

include $(R2P_ROOT)/core/r2p.mk

