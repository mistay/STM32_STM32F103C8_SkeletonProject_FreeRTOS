# Compile the project

# Uncomment the appropriate device type and startup file
#DEVICE_TYPE = STM32F10X_LD
#STARTUP_FILE = stm32f10x_ld
#DEVICE_TYPE = STM32F10X_MD
#y#
#STARTUP_FILE = stm32f10x_md
DEVICE_TYPE = STM32F10X_HD
STARTUP_FILE = stm32f10x_hd
#y#
#DEVICE_TYPE = STM32F10X_LD_VL
#y#
#DEVICE_TYPE = STM32F10X_LD_VL
#STARTUP_FILE = stm32f10x_ld_vl
#DEVICE_TYPE = STM32F10X_MD_VL
#STARTUP_FILE = stm32f10x_md_vl
#DEVICE_TYPE = STM32F10X_HD_VL
#STARTUP_FILE = stm32f10x_hd_vl
#DEVICE_TYPE = STM32F10X_XL
#STARTUP_FILE = stm32f10x_xl
#DEVICE_TYPE = STM32F10X_CL
#STARTUP_FILE = stm32f10x_cl
# 
# STM32F103C8
# Memories
# 64 or 128 Kbytes of Flash memory
# 20 Kbytes of SRAM
# 
# - Low-density devices are STM32F101xx, STM32F102xx and STM32F103xx microcontrollers
#   where the Flash memory density ranges between 16 and 32 Kbytes.
# 
# - Low-density value line devices are STM32F100xx microcontrollers where the Flash
#   memory density ranges between 16 and 32 Kbytes.
# 
# - Medium-density devices are STM32F101xx, STM32F102xx and STM32F103xx microcontrollers
#   where the Flash memory density ranges between 64 and 128 Kbytes.
# 
# - Medium-density value line devices are STM32F100xx microcontrollers where the 
#   Flash memory density ranges between 64 and 128 Kbytes.   
# 
# - High-density devices are STM32F101xx and STM32F103xx microcontrollers where
#   the Flash memory density ranges between 256 and 512 Kbytes.
# 
# - High-density value line devices are STM32F100xx microcontrollers where the 
#   Flash memory density ranges between 256 and 512 Kbytes.   
# 
# - XL-density devices are STM32F101xx and STM32F103xx microcontrollers where
#   the Flash memory density ranges between 512 and 1024 Kbytes.
# 
# - Connectivity line devices are STM32F105xx and STM32F107xx microcontrollers.
#
# Set the external clock frequency
HSE_VALUE = 8000000L

# Enable debug compilation
#DEBUG = 1
#y#
DEBUG = 1
#y#
# [OPTIONAL] Set the serial details for bootloading
STM32LDR_PORT = /dev/ttyUSB0
STM32LDR_BAUD = 115200
# [OPTIONAL] Comment out to disable bootloader verification
STM32LDR_VERIFY = -v

# [OPTIONAL] Uncomment to use the firmware library
FWLIB = lib/STM32F10x_StdPeriph_Driver/libstm32fw.a
# [OPTIONAL] Uncomment to enable peripheral drivers
#             (instead of direct register access)
USE_STDPERIPH_DRIVER = -DUSE_STDPERIPH_DRIVER
# [OPTIONAL] Uncomment to use the USB library
#USBLIB = lib/STM32_USB-FS-Device_Driver/libstm32usb.a
BAR = bar/libbar.a
LIBENC28J60 = lib/libenc28j60/libenc28j60.a

# [OPTIONAL] Uncomment to link to maths library libm
#LIBM = -lm

export DEBUG
export MESSAGES

TARGET_ARCH = -mcpu=cortex-m3 -mthumb

INCLUDE_DIRS = -I . -I lib/STM32F10x_StdPeriph_Driver/inc\
 -I lib/STM32F10x_StdPeriph_Driver -I lib/CMSIS_CM3\
 -I lib/STM32_USB-FS-Device_Driver/inc \
 -I lib/libenc28j60 \
 -I bar/ \
 -I FreeRTOS/include/ \
 -I FreeRTOS/portable/GCC/ARM_CM3/ \


LIBRARY_DIRS = -L lib/STM32F10x_StdPeriph_Driver/\
 -L lib/STM32_USB-FS-Device_Driver 

DEFINES = -D$(DEVICE_TYPE) -DHSE_VALUE=$(HSE_VALUE) $(USE_STDPERIPH_DRIVER)

export DEFINES

COMPILE_OPTS = $(WARNINGS) $(TARGET_OPTS) $(MESSAGES) $(INCLUDE_DIRS) $(DEFINES)
WARNINGS = -Wall -W -Wshadow -Wcast-qual -Wwrite-strings -Winline

ifdef DEBUG
 TARGET_OPTS = -O0 -g3
 DEBUG_MACRO = -DDEBUG
else
 TARGET_OPTS = -O2 $(F_INLINE) $(F_INLINE_ONCE) $(F_UNROLL_LOOPS)
 F_INLINE = -finline
 F_INLINE_ONCE = -finline-functions-called-once
 #F_UNROLL_LOOPS = -funroll-loops
endif

CC = arm-none-eabi-gcc
CFLAGS = -std=gnu99 $(COMPILE_OPTS)

AS = $(CC) -x assembler-with-cpp -c $(TARGET_ARCH)
ASFLAGS = $(COMPILE_OPTS)

LD = $(CC)
LDFLAGS = -Wl,--gc-sections,-Map=$(MAIN_MAP),-cref -T stm32.ld $(INCLUDE_DIRS)\
 $(LIBRARY_DIRS) $(LIBM)

AR = arm-none-eabi-ar
ARFLAGS = cr

OBJCOPY = arm-none-eabi-objcopy
OBJCOPYFLAGS = -O binary

STARTUP_OBJ = lib/CMSIS_CM3/startup/gcc/startup_$(STARTUP_FILE).o

MAIN_OUT = main.elf
MAIN_MAP = $(MAIN_OUT:%.elf=%.map)
MAIN_BIN = $(MAIN_OUT:%.elf=%.bin)

MAIN_OBJS = $(sort \
 $(patsubst %.cpp,%.o,$(wildcard *.cpp)) \
 $(patsubst %.cc,%.o,$(wildcard *.cc)) \
 $(patsubst %.c,%.o,$(wildcard *.c)) \
 $(patsubst %.s,%.o,$(wildcard *.s)) \
 $(patsubst %.c,%.o,$(wildcard lib/CMSIS_CM3/*.c)) \
 $(STARTUP_OBJ))

# all

.PHONY: all
all: $(MAIN_BIN)


# main

$(MAIN_OUT): $(MAIN_OBJS) $(FWLIB) $(USBLIB) $(BAR) $(LIBENC28J60) FreeRTOS/*.o FreeRTOS/portable/GCC/ARM_CM3/*.o FreeRTOS/portable/MemMang/heap_2.o #FreeRTOS/portable/GCC/ARM_CM3/*.o #FreeRTOS/*.o #FreeRTOS/Source/portable/GCC/ARM_CM3/port.c 
	$(LD) $(LDFLAGS) $(TARGET_ARCH) $^ -o $@

$(MAIN_OBJS): $(wildcard *.h) $(wildcard lib/STM32F10x_StdPeriph_Driver/*.h)\
 $(wildcard lib/STM32F10x_StdPeriph_Driver/inc/*.h)\
 $(wildcard lib/CMSIS_CM3/*.h)\
 $(wildcard lib/STM32_USB-FS-Device_Driver/inc/*.h)

$(MAIN_BIN): $(MAIN_OUT)
	$(OBJCOPY) $(OBJCOPYFLAGS) $< $@


# libenc28j60

.PHONY: libenc28j60
libenc28j60: $(LIBENC28J60)

$(LIBENC28J60): $(wildcard lib/libenc28j60/*.h)
	@cd lib/libenc28j60 && $(MAKE)

# bar

.PHONY: bar
bar: $(BAR)

$(BAR): $(wildcard bar/*.h)
	@cd bar && $(MAKE) 
	

# fwlib

.PHONY: fwlib
fwlib: $(FWLIB)

$(FWLIB): $(wildcard lib/STM32F10x_StdPeriph_Driver/*.h)\
 $(wildcard lib/STM32F10x_StdPeriph_Driver/inc/*.h)
	@cd lib/STM32F10x_StdPeriph_Driver && $(MAKE)

# usblib

.PHONY: usblib
usblib: $(USBLIB)

$(USBLIB): $(wildcard lib/STM32_USB-FS-Device_Driver/inc*.h)
	@cd lib/STM32_USB-FS-Device_Driver && $(MAKE)

# flash

.PHONY: flash
flash: flash-elf
#flash: flash-bin

.PHONY: flash-elf
flash-elf: all
	@cp $(MAIN_OUT) jtag/flash.elf
	@cd jtag && openocd -f flash-elf.cfg
	@rm jtag/flash.elf

.PHONY: flash-bin
flash-bin: all
	@cp $(MAIN_BIN) jtag/flash.bin
	@cd jtag && openocd -f flash-bin.cfg
	@rm jtag/flash.bin

.PHONY: upload
upload: all
	@python jtag/stm32loader.py -p $(STM32LDR_PORT) -b $(STM32LDR_BAUD) \
	-e $(STM32LDR_VERIFY) -w main.bin


# clean

.PHONY: clean
clean:
	-rm -f $(MAIN_OBJS) $(MAIN_OUT) $(MAIN_MAP) $(MAIN_BIN)
	-rm -f lib/CMSIS_CM3/startup/gcc/*.o
	-rm -f jtag/flash.elf jtag/flash.bin
	@cd lib/STM32F10x_StdPeriph_Driver && $(MAKE) clean
	@cd lib/STM32_USB-FS-Device_Driver && $(MAKE) clean
	@cd lib/libenc28j60 && $(MAKE) clean
