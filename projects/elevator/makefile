TARGET = elevator
SRCS = 
CPP_SRCS =  elevator.cpp  main.cpp  motor.cpp

LINKER_SCRIPT = ../../flash/stm32f407.ld

# Generate debug info
DEBUG = 1

# Choose processor
CDEFS  = -DSTM32F407xx
# Enable FPU
#CDEFS += -D__VFP_FP__

INCLUDES    += -I ../StateMachines/tinyfsm/include

CPPFLAGS    += -std=c++11

CPPFLAGS    += -Wextra
CPPFLAGS    += -Wctor-dtor-privacy
CPPFLAGS    += -Wcast-align -Wpointer-arith -Wredundant-decls
CPPFLAGS    += -Wshadow -Wcast-qual -Wcast-align -pedantic

LDFLAGS     += -fno-exceptions
LDFLAGS     += -fno-rtti

include ../armf4.mk