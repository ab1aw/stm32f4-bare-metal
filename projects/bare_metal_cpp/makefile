TARGET = main
SRCS = 
CPP_SRCS = main.cpp startup/handlers_cm.cpp startup/stack.cpp startup/startup.cpp

LINKER_SCRIPT = ../../flash/stm32f407.ld

# Generate debug info
DEBUG = 1

# Choose processor
CDEFS  = -DSTM32F407xx
# Enable FPU
#CDEFS += -D__VFP_FP__

include ../armf4.mk

clean-all : clean
	rm -fr startup/*.[od]