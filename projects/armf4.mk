CMSIS = ../../libs/CMSIS_5

SRCS += ../../include/system_stm32f4xx.c

#OBJS = $(SRCS:.c=.o)
#CPP_OBJS = $(CPP_SRCS:.cpp=.o)

OBJS = $(addprefix ,$(notdir $(SRCS:.c=.o)))
CPP_OBJS = $(addprefix ,$(notdir $(CPP_SRCS:.cpp=.o)))

vpath %.c $(sort $(dir $(SRCS)))
vpath %.cpp $(sort $(dir $(CPP_SRCS)))

INCLUDES += -I.
INCLUDES += -I../../include
INCLUDES += -I$(CMSIS)/CMSIS/Core/Include

CFLAGS += $(CDEFS)

CFLAGS += -mcpu=cortex-m4 -mthumb # processor setup
CFLAGS += -O0 # optimization is off

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2 # generate debug info
# generate dependency info
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"
endif

CFLAGS += -fno-common
CFLAGS += -Wall # turn on warnings
CFLAGS += -pedantic # more warnings
CFLAGS += -Wsign-compare
CFLAGS += -Wcast-align
CFLAGS += -Wconversion # neg int const implicitly converted to uint
CFLAGS += -fsingle-precision-constant
CFLAGS += -fomit-frame-pointer # do not use fp if not needed
CFLAGS += -ffunction-sections -fdata-sections
#CFLAGS += --std=c99

# Chooses the relevant FPU option
#CFLAGS += -mfloat-abi=soft # No FP
CFLAGS += -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -lang-c-c++-comments # Soft FP
#CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16 # Hard FP

LDFLAGS += -mfloat-abi=softfp -mfpu=fpv4-sp-d16 # Soft FP
#LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16 # Hard FP

LDFLAGS += -march=armv7e-m # processor setup
LDFLAGS += -nostartfiles # no start files are used
#LDFLAGS += --specs=nano.specs
LDFLAGS += --specs=nosys.specs
LDFLAGS += -Wl,--gc-sections # linker garbage collector
LDFLAGS += -Wl,-Map=$(TARGET).map #generate map file
LDFLAGS += -T$(LINKER_SCRIPT)
LDFLAGS += $(LIBS)
#LDFLAGS += -lc -lstdc++_s  -lnosys
LDFLAGS += -lc -lnosys

CROSS_COMPILE = arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
OBJDUMP = $(CROSS_COMPILE)objdump
OBJCOPY = $(CROSS_COMPILE)objcopy
SIZE = $(CROSS_COMPILE)size
DBG = $(CROSS_COMPILE)gdb
CPP = $(CROSS_COMPILE)g++

all: clean $(SRCS) build size
	@echo "Successfully finished..."

build: $(TARGET).elf $(TARGET).bin $(TARGET).lst

$(TARGET).elf: $(OBJS) $(CPP_OBJS)
	@echo -e "\n\nBuilding" $@
	@$(CC) -v $(OBJS) $(CPP_OBJS) $(LDFLAGS) -o $@

%.o: %.c
	@echo -e "\n\nBuilding" $@
	@$(CC) -v $(CFLAGS) $(INCLUDES) -c $< -o $@

%.o: %.cpp
	@echo -e "\n\nBuilding" $@
	@$(CPP) -v $(CFLAGS) $(INCLUDES) -c $< -o $@

%.o: %.s
	@echo -e "\n\nBuilding" $@
	@$(CC) $(CFLAGS) -c $< -o $@

%.hex: %.elf
	@echo -e "\n\nBuilding" $@
	@$(OBJCOPY) -O ihex $< $@

%.bin: %.elf
	@echo -e "\n\nBuilding" $@
	@$(OBJCOPY) -O binary $< $@

%.lst: %.elf
	@echo -e "\n\nBuilding" $@
	@$(OBJDUMP) -x -S $(TARGET).elf > $@

size: $(TARGET).elf
	@$(SIZE) $(TARGET).elf

disass: $(TARGET).elf
	@$(OBJDUMP) -d $(TARGET).elf

disass-all: $(TARGET).elf
	@$(OBJDUMP) -D $(TARGET).elf

debug:
	@$(DBG) --eval-command="target extended-remote :4242" \
	 $(TARGET).elf

burn:
	@st-flash write $(TARGET).bin 0x8000000

flash:
	st-flash write $(TARGET).bin 0x8000000 || \
	openocd -f interface/stlink-v2.cfg \
		-f target/stm32f4x_stlink.cfg \
		-c "init" \
		-c "reset init" \
		-c "flash probe 0" \
		-c "flash info 0" \
		-c "flash write_image erase $(TARGET).bin 0x08000000" \
		-c "reset run" -c "shutdown"

clean:
	@echo "Cleaning..."
	@rm -f $(TARGET).elf
	@rm -f $(TARGET).bin
	@rm -f $(TARGET).map
	@rm -f $(TARGET).hex
	@rm -f $(TARGET).lst
	@rm -f *.o
	@rm -f *.d

.PHONY: all build size clean burn disass disass-all
