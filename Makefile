PROJ_NAME=logician

######################################################################
#                         SETUP SOURCES                              #
######################################################################

LIBRARY_DIR = ./lib

SRC_DIRS  = src
SRC_DIRS += $(LIBRARY_DIR)/dblib/src
SRC_DIRS += $(LIBRARY_DIR)/StdPeriph/src
SRC_DIRS += $(LIBRARY_DIR)/USB_OTG/src
SRC_DIRS += $(LIBRARY_DIR)/USB_Device/Core/src
SRC_DIRS += $(LIBRARY_DIR)/USB_Device/Class/cdc/src

vpath %.c $(SRC_DIRS)
vpath %.a lib

SRCS  = $(wildcard src/*.c)
SRCS += discovery.c usart.c scheduler.c systime.c
SRCS += usbd_cdc_core.c usbd_core.c usbd_ioreq.c usbd_req.c
SRCS += usb_core.c usb_otg.c usb_dcd_int.c usb_dcd.c

# Standard peripheral sources
#SRCS  += stm32f4xx_adc.c
#SRCS  += stm32f4xx_can.c
#SRCS  += stm32f4xx_crc.c
#SRCS  += stm32f4xx_cryp.c
#SRCS  += stm32f4xx_dac.c
#SRCS  += stm32f4xx_dbgmcu.c
#SRCS  += stm32f4xx_dcmi.c
#SRCS  += stm32f4xx_dma.c
SRCS  += stm32f4xx_exti.c
#SRCS  += stm32f4xx_flash.c
#SRCS  += stm32f4xx_fsmc.c
#SRCS  += stm32f4xx_hash.c
SRCS  += stm32f4xx_gpio.c
#SRCS  += stm32f4xx_i2c.c
#SRCS  += stm32f4xx_iwdg.c
#SRCS  += stm32f4xx_pwr.c
SRCS  += stm32f4xx_rcc.c
#SRCS  += stm32f4xx_rng.c
#SRCS  += stm32f4xx_rtc.c
#SRCS  += stm32f4xx_sdio.c
#SRCS  += stm32f4xx_spi.c
#SRCS  += stm32f4xx_syscfg.c
#SRCS  += stm32f4xx_tim.c
SRCS  += stm32f4xx_usart.c
#SRCS  += stm32f4xx_wwdg.c
SRCS  += misc.c

SRCS += system_stm32f4xx.c

SRCS += $(LIBRARY_DIR)/startup_stm32f4xx.s

INC_DIRS = inc
INC_DIRS += conf
INC_DIRS += $(LIBRARY_DIR)
INC_DIRS += $(LIBRARY_DIR)/stm32
INC_DIRS += $(LIBRARY_DIR)/cmsis
INC_DIRS += $(LIBRARY_DIR)/dblib/inc
INC_DIRS += $(LIBRARY_DIR)/StdPeriph/inc
INC_DIRS += $(LIBRARY_DIR)/USB_OTG/inc
INC_DIRS += $(LIBRARY_DIR)/USB_Device/Core/inc
INC_DIRS += $(LIBRARY_DIR)/USB_Device/Class/cdc/inc


DIST_BUILD_DIR = build/dist
DEBUG_BUILD_DIR = build/debug

######################################################################
#                         SETUP TOOLS                                #
######################################################################

TOOLS_DIR = $(HOME)/development/mac-arm-toolchain/bin

CC      = $(TOOLS_DIR)/arm-none-eabi-gcc
OBJCOPY = $(TOOLS_DIR)/arm-none-eabi-objcopy
GDB     = $(TOOLS_DIR)/arm-none-eabi-gdb

## Preprocessor options
FLOAT_TYPE = hard

INCLUDE = $(addprefix -I,$(INC_DIRS))

DEFS    = -DUSE_STDPERIPH_DRIVER
DEFS   += -DSYSTIME_COUNTS_PER_MS=10
DEFS   += -DHSE_VALUE=8000000
# if you use the following option, you must implement the function 
#    assert_failed(uint8_t* file, uint32_t line)
# because it is conditionally used in the library
# DEFS   += -DUSE_FULL_ASSERT

## Compiler options
CFLAGS  = -std=gnu99
CFLAGS += -Wall -Wextra -Warray-bounds -Wno-unused-parameter 
CFLAGS += -Wno-unused-variable -Wno-strict-aliasing
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

## Defines

LFLAGS  = -Tstm32_flash.ld

######################################################################
#                         SETUP TARGETS                              #
######################################################################

.PHONY: $(PROJ_NAME) debug status ocd

$(PROJ_NAME): CFLAGS += -O2
$(PROJ_NAME): BUILD_DIR = $(DIST_BUILD_DIR)
$(PROJ_NAME): $(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	@mkdir -p $(BUILD_DIR)
	$(CC) $(INCLUDE) $(DEFS) $(CFLAGS) $(LFLAGS) $^ -o $(BUILD_DIR)/$@
	$(OBJCOPY) -O ihex $(BUILD_DIR)/$(PROJ_NAME).elf   $(BUILD_DIR)/$(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(BUILD_DIR)/$(PROJ_NAME).elf $(BUILD_DIR)/$(PROJ_NAME).bin

clean:
	rm -f $(DIST_BUILD_DIR)/*.o
	rm -f $(DIST_BUILD_DIR)/*.elf
	rm -f $(DIST_BUILD_DIR)/*.bin
	rm -f $(DIST_BUILD_DIR)/*.hex
	rm -f $(DEBUG_BUILD_DIR)/*.o
	rm -f $(DEBUG_BUILD_DIR)/*.elf
	rm -f $(DEBUG_BUILD_DIR)/*.bin
	rm -f $(DEBUG_BUILD_DIR)/*.hex

upload: CFLAGS += -O2
upload: BUILD_DIR = $(DIST_BUILD_DIR)
upload: $(PROJ_NAME).elf
	@echo "reset halt\nflash write_image erase $(DIST_BUILD_DIR)/$(PROJ_NAME).bin 0x08000000 bin\nreset run\nexit\n" | nc localhost 4444

status:
	@echo 'flash probe 0\nexit\n' | nc localhost 4444

debug: CFLAGS += -O0
debug: CFLAGS += -ggdb
debug: BUILD_DIR = $(DEBUG_BUILD_DIR)
debug: $(PROJ_NAME).elf
	@echo "reset halt\nflash write_image erase $(DEBUG_BUILD_DIR)/$(PROJ_NAME).bin 0x08000000 bin\nreset run\nexit\n" | nc localhost 4444
	$(GDB) $(DEBUG_BUILD_DIR)/$(PROJ_NAME).elf

ocd:
	openocd -f stm32f4discovery.cfg

