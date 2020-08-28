OUTPUT_FILENAME :itc_sdk11

PROJECT_NAME := itc_polyU
PROJECT_PATH = $(HOME)/workspace/itcx_sdk11
target: $(PROJECT_PATH)
SDK_PATH = $(HOME)/nRF5_SDK_11.0.0_89a8197
#SDK_PATH = $(HOME)/nRF51_SDK_9.0.0_2e23562
export OUTPUT_FILENAME

#MAKEFILE_NAME := $(CURDIR)/$(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST))
MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) ) 

TEMPLATE_PATH = $(SDK_PATH)/components/toolchain/gcc
ifeq ($(OS),Windows_NT)
include $(TEMPLATE_PATH)/Makefile.windows
else
include $(TEMPLATE_PATH)/Makefile.posix
endif

MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO := 
else
NO_ECHO := @
endif

# Toolchain commands
CC       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc"
AS       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as"
AR       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar" -r
LD       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld"
NM       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm"
OBJDUMP  		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump"
OBJCOPY  		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy"
SIZE    		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size"

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#source common to all targets
C_SOURCE_FILES += \
$(abspath $(SDK_PATH)/components/libraries/timer/app_timer.c) \
$(abspath $(SDK_PATH)/components/libraries/button/app_button.c) \
$(abspath $(SDK_PATH)/components/libraries/util/app_error.c) \
$(abspath $(SDK_PATH)/components/libraries/fifo/app_fifo.c) \
$(abspath $(SDK_PATH)/components/libraries/trace/app_trace.c) \
$(abspath $(SDK_PATH)/components/libraries/bootloader_dfu/bootloader_util.c) \
$(abspath $(SDK_PATH)/components/libraries/bootloader_dfu/dfu_app_handler.c) \
$(abspath $(SDK_PATH)/components/libraries/util/nrf_assert.c) \
$(abspath $(SDK_PATH)/components/drivers_nrf/delay/nrf_delay.c) \
$(abspath $(SDK_PATH)/components/drivers_nrf/common/nrf_drv_common.c) \
$(abspath $(SDK_PATH)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c) \
$(abspath $(SDK_PATH)/components/drivers_nrf/ble_flash/ble_flash.c) \
$(abspath $(SDK_PATH)/components/ble/common/ble_advdata.c) \
$(abspath $(SDK_PATH)/components/ble/common/ble_conn_params.c) \
$(abspath $(SDK_PATH)/components/ble/ble_services/ble_dfu/ble_dfu.c) \
$(abspath $(SDK_PATH)/components/ble/ble_services/ble_dis/ble_dis.c) \
$(abspath $(SDK_PATH)/components/ble/ble_services/ble_hrs/ble_hrs.c) \
$(abspath $(SDK_PATH)/components/ble/ble_services/ble_bas/ble_bas.c) \
$(abspath $(SDK_PATH)/components/ble/common/ble_srv_common.c) \
$(abspath $(SDK_PATH)/components/toolchain/system_nrf51.c) \
$(abspath $(SDK_PATH)/components/ble/device_manager/device_manager_peripheral.c) \
$(abspath $(SDK_PATH)/components/softdevice/common/softdevice_handler/softdevice_handler.c) \
$(abspath $(SDK_PATH)/components/drivers_nrf/pstorage/pstorage.c) \
$(abspath $(SDK_PATH)/components/libraries/fstorage/fstorage.c) \
$(abspath $(SDK_PATH)/components/ble/ble_advertising/ble_advertising.c) \
$(abspath $(SDK_PATH)/components/libraries/uart/retarget.c) \
$(abspath $(SDK_PATH)/components/ble/ble_radio_notification/ble_radio_notification.c) \
$(abspath $(SDK_PATH)/components/ble/ble_services/ble_nus/ble_nus.c) \
$(abspath $(PROJECT_PATH)/Application/adc.c) \
$(abspath $(PROJECT_PATH)/Application/ble_controller_serv.c) \
$(abspath $(PROJECT_PATH)/Application/ble_sensor_serv.c) \
$(abspath $(PROJECT_PATH)/Application/bluetooth.c) \
$(abspath $(PROJECT_PATH)/Application/button.c) \
$(abspath $(PROJECT_PATH)/Application/flash.c) \
$(abspath $(PROJECT_PATH)/Application/init.c) \
$(abspath $(PROJECT_PATH)/Application/nrf_pwm.c) \
$(abspath $(PROJECT_PATH)/Application/pm.c) \
$(abspath $(PROJECT_PATH)/Application/pwm.c) \
$(abspath $(PROJECT_PATH)/Application/timers.c) \
$(abspath $(PROJECT_PATH)/Application/safety_mode.c) \
$(abspath $(PROJECT_PATH)/Application/ble_uart.c) \
$(abspath $(PROJECT_PATH)/Application/main.c) \
$(abspath $(PROJECT_PATH)/Application/mpu6050.c) \
$(abspath $(PROJECT_PATH)/Application/api_mpu6050.c) \
$(abspath $(PROJECT_PATH)/Application/bmp180.c) \
$(abspath $(PROJECT_PATH)/Application/ap3216c.c) \
$(abspath $(PROJECT_PATH)/Application/door.c) \
$(abspath $(PROJECT_PATH)/Application/twi_hw_master.c) \
$(abspath $(PROJECT_PATH)/Application/quaternion.c) \
$(abspath $(PROJECT_PATH)/Application/sensor_processing_lib.c) \
$(abspath $(PROJECT_PATH)/Application/vector_3d.c) \
$(abspath $(SDK_PATH)/components/drivers_nrf/uart/nrf_drv_uart.c) \
$(abspath $(SDK_PATH)/components/drivers_nrf/uart/app_uart_fifo.c) \
$(abspath $(PROJECT_PATH)/Application/uart.c) \
$(abspath $(SDK_PATH)/components/drivers_nrf/adc/nrf_drv_adc.c) \

#$(abspath $(SDK_PATH)/components/drivers_nrf/hal/nrf_adc.c) \

#assembly files common to all targets 

ASM_SOURCE_FILES  = $(abspath $(SDK_PATH)/components/toolchain/gcc/gcc_startup_nrf51.s)

#includes common to all targets

INC_PATHS = -I$(PROJECT_PATH)/config/ble_app_proximity_s130_pca10028
INC_PATHS += -I$(PROJECT_PATH)/config
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/config
INC_PATHS += -I$(SDK_PATH)/components/toolchain/CMSIS/Include
INC_PATHS += -I$(SDK_PATH)/components/softdevice/s130/headers
INC_PATHS += -I$(SDK_PATH)/components/softdevice/s130/headers/nrf51
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/nrf_soc_nosd
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/delay
INC_PATHS += -I$(SDK_PATH)/components/libraries/bootloader_dfu
INC_PATHS += -I$(SDK_PATH)/components/ble/ble_services/ble_dfu
INC_PATHS += -I$(SDK_PATH)/components/libraries/fifo
INC_PATHS += -I$(SDK_PATH)/components/libraries/util
INC_PATHS += -I$(SDK_PATH)/components/libraries/uart
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/pstorage/config
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/pstorage
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/ble_flash
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/uart
INC_PATHS += -I$(SDK_PATH)/components/ble/common
INC_PATHS += -I$(SDK_PATH)/components/libraries/sensorsim
INC_PATHS += -I$(SDK_PATH)/components/ble/device_manager
INC_PATHS += -I$(SDK_PATH)/components/ble/ble_services/ble_dis
INC_PATHS += -I$(SDK_PATH)/components/device
INC_PATHS += -I$(SDK_PATH)/components/libraries/button
INC_PATHS += -I$(SDK_PATH)/components/libraries/timer
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/gpiote
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/hal
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/pstorage
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/ble_flash
INC_PATHS += -I$(SDK_PATH)/components/toolchain/gcc
INC_PATHS += -I$(SDK_PATH)/components/toolchain
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/common
INC_PATHS += -I$(SDK_PATH)/components/ble/ble_advertising
INC_PATHS += -I$(SDK_PATH)/components/libraries/trace
INC_PATHS += -I$(SDK_PATH)/components/ble/ble_services/ble_bas
INC_PATHS += -I$(SDK_PATH)/components/ble/ble_radio_notification
INC_PATHS += -I$(SDK_PATH)/components/softdevice/common/softdevice_handler
INC_PATHS += -I$(SDK_PATH)/components/ble/ble_services/ble_dfu
INC_PATHS += -I$(SDK_PATH)/components/ble/ble_services/ble_hrs
INC_PATHS += -I$(SDK_PATH)/components/libraries/fstorage
INC_PATHS += -I$(SDK_PATH)/components/libraries/fstorage/config
INC_PATHS += -I$(SDK_PATH)/components/libraries/fds
INC_PATHS += -I$(SDK_PATH)/components/libraries/fds/config
INC_PATHS += -I$(SDK_PATH)/components/libraries/experimental_section_vars
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/timer
INC_PATHS += -I$(SDK_PATH)/components/ble/ble_services/ble_bas
INC_PATHS += -I$(SDK_PATH)/components/ble/ble_services/ble_nus
INC_PATHS += -I$(PROJECT_PATH)/Application
INC_PATHS += -I$(SDK_PATH)/examples/bsp
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/adc

#INC_PATHS += -I$(SDK_PATH)/external/segger_rtt


OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )



#flags common to all targets
CFLAGS  = -DSOFTDEVICE_PRESENT
CFLAGS += -DNRF51
CFLAGS += -DS130
CFLAGS += -DBOARD_ITC_POLYU
#CFLAGS += -DBOARD_PCA10031_OHTCOM
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -Os
CFLAGS += -mfloat-abi=soft
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums
#FLAGS += -flto -fno-builtin
#CFLAGS += --short-enums
#CFLAGS += -Wno-unused-but-set-variable
CFLAGS += -Wno-unused-variable
CFLAGS += -Wno-unused-function



# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys
LDFLAGS += -u _printf_float

# Assembler flags
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DBOARD_ITC_POLYU
#ASMFLAGS += -DBOARD_PCA10031_OHTCOM
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DNRF51
ASMFLAGS += -DS130
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
#ASMFLAGS += -DBSP_DEFINES_ONLY
#ASMFLAGS += -D__HEAP_SIZE=16 -D__STACK_SIZE=1024
#ASMFLAGS += -DBSP_SIMPLE
#ASMFLAGS += -D_RTE_


#default target - first one defined
default: clean nrf51422_xxac_s130

#building all targets
all: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e nrf51422_xxac_s130

#target for printing all targets
help:
	@echo following targets are available:
	@echo 	nrf51422_xxac_s130
	@echo 	flash_softdevice


C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

nrf51422_xxac_s130: OUTPUT_FILENAME := postbox_sdk11
nrf51422_xxac_s130: LINKER_SCRIPT=linker_gcc_nrf51qfaa.ld
nrf51422_xxac_s130: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS)-lm -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	@echo arm-none-eabi-gcc $(CFLAGS) $(INC_PATHS) -c -o $@ $<	
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<


# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out


## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize: genbin genhex echosize genpack

genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
genhex: 
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

echosize:
	-@echo ""
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ""

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o





JLINK_OPTS = -device nrf51422_xxac -if swd -speed 4000
JLINK_ROOT := /opt/SEGGER/JLink
SOFTDEVICE_PATH := $(SDK_PATH)/components/softdevice/s130/hex
SOFTDEVICE_FILE := s130_nrf51_2.0.0_softdevice.hex
GDB_PORT_NUMBER = 2331

ifeq ($(USE_SOFTDEVICE),)
	FLASH_START_ADDRESS = 0
else
	FLASH_START_ADDRESS = 0x1B000
endif

ifneq ($(OS),Windows_NT)
	UNAME_S := $(shell uname -s)
	ifeq ($(UNAME_S),Linux)
		export LD_LIBRARY_PATH := $(JLINK_ROOT):$(LD_LIBRARY_PATH)
	endif
	ifeq ($(UNAME_S),Darwin)
		export DYLD_LIBRARY_PATH := $(JLINK_ROOT):$(DYLD_LIBRARY_PATH)
	endif
endif

JLINK := $(JLINK_ROOT)/JLinkExe $(JLINK_OPTS)
JLINKGDBSERVER := $(JLINK_ROOT)/JLinkGDBServer

genpack: combine genzip
	@echo GEN Package
	rm $(BUILD_DIRECTORIES)/*.o
	rm $(BUILD_DIRECTORIES)/*.map
	rm $(BUILD_DIRECTORIES)/*.out
	#rm $(BUILD_DIRECTORIES)/*.bin

combine:
	@echo Merging app and SoftDevice and App and Valid params
	srec_cat $(SDK_PATH)/bootloader/boot_dfu_dual_bank.hex --intel $(SDK_PATH)/components/softdevice/s130/hex/s130_nrf51_2.0.0_softdevice.hex --intel -o $(OUTPUT_BINARY_DIRECTORY)/comb_bl_sd.hex --intel
	srec_cat $(OUTPUT_BINARY_DIRECTORY)/comb_bl_sd.hex --intel $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex --intel -o $(OUTPUT_BINARY_DIRECTORY)/comb_bl_sd_$(OUTPUT_FILENAME).hex --intel
	srec_cat $(OUTPUT_BINARY_DIRECTORY)/comb_bl_sd_$(OUTPUT_FILENAME).hex --intel $(SDK_PATH)/bootloader/app_valid_setting_apply.hex --intel -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)_combined.hex --intel
	rm $(BUILD_DIRECTORIES)/comb_bl*.hex	

genzip:
	@echo Package zip
	nrfutil dfu genpkg $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).zip --application $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex --application-version 0xffff --dev-revision 0xff --dev-type 0xff --sd-req 0xfffe


serial_send:
	nrfutil dfu serial -pkg $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).zip -p /dev/ttyUSB0  -b 115200 --flowcontrol


## Program device
flash: erase-all flash-softdevice flash.jlink
	$(JLINK) flash.jlink
flash.jlink:
	printf "loadbin $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex 0x1B000\nr\ng\nexit\n" > flash.jlink
#	printf "loadbin $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin $(FLASH_START_ADDR)\nr\ng\nexit\n" > flash.jlink
	

rm-flash.jlink:
	-rm -rf flash.jlink
	
#hexfile=$(SOFTDEVICE_PATH)/$(SOFTDEVICE_FILE)
#binoutput=S130_nRF51822_3.0.0-1.beta_softdevice_mainpart.bin
flash-softdevice: flash-softdevice.jlink #flash-softdevice.jlink stopdebug
	$(OBJCOPY) -Iihex -Obinary SOFTDEVICE_FILE $(OUTPUT_BINARY_DIRECTORY)/s130_nrf51_2.0.0_softdevice.bin
	$(JLINK) flash-softdevice.jlink

flash-softdevice.jlink:
	echo "w4 4001e504 1\nloadbin \"$(OUTPUT_BINARY_DIRECTORY)/s130_nrf51_2.0.0_softdevice.bin\" 0x00000\nr\ng\nexit\n" > flash-softdevice.jlink



## Program device
flash-combined: combine erase-all flash-combined.jlink
	$(JLINK) flash-combined.jlink
flash-combined.jlink:
	~/nrfjprog/nrfjprog --eraseall 
	~/nrfjprog/nrfjprog --program $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)_combined.hex
	~/nrfjprog/nrfjprog --pinreset



#flash-softdevice: erase-all flash-softdevice.jlink
#ifndef SOFTDEVICE
#	$(error "You need to set the SOFTDEVICE command-line parameter to a path (without spaces) to the softdevice hex-file")
#endif

	# Convert from hex to binary. Split original hex in two to avoid huge (>250 MB) binary file with just 0s. 
#	$(OBJCOPY) -Iihex -Obinary --remove-section .sec3 $(SOFTDEVICE) $(SOFTDEVICE_OUTPUT:.hex=_mainpart.bin)
#	$(OBJCOPY) -Iihex -Obinary --remove-section .sec1 --remove-section .sec2 $(SOFTDEVICE) $(SOFTDEVICE_OUTPUT:.hex=_uicr.bin)
#	$(JLINK) flash-softdevice.jlink

#flash-softdevice.jlink:
	# Do magic. Write to NVMC to enable erase, do erase all and erase UICR, reset, enable writing, load mainpart bin, load uicr bin. Reset.
	# Resetting in between is needed to disable the protections. 
#	echo "w4 4001e504 1\nloadbin \"$(SOFTDEVICE_OUTPUT:.hex=_mainpart.bin)\" 0\nloadbin \"$(SOFTDEVICE_OUTPUT:.hex=_uicr.bin)\" 0x10001000\nr\ng\nexit\n" > flash-softdevice.jlink





recover: recover.jlink erase-all.jlink pin-reset.jlink
	$(JLINK) recover.jlink
	$(JLINK) erase-all.jlink
	$(JLINK) pin-reset.jlink

recover.jlink:
	echo "si 0\nt0\nsleep 1\ntck1\nsleep 1\nt1\nsleep 2\nt0\nsleep 2\nt1\nsleep 2\nt0\nsleep 2\nt1\nsleep 2\nt0\nsleep 2\nt1\nsleep 2\nt0\nsleep 2\nt1\nsleep 2\nt0\nsleep 2\nt1\nsleep 2\nt0\nsleep 2\nt1\nsleep 2\ntck0\nsleep 100\nsi 1\nr\nexit\n" > $(OUTPUT_BINARY_DIRECTORY)/recover.jlink

pin-reset.jlink:
	echo "device nrf51422_xxaa\nw4 4001e504 2\nw4 40000544 1\nr\nexit\n" > pin-reset.jlink

erase-all: erase-all.jlink
	$(JLINK) erase-all.jlink

erase-all.jlink:
#	echo "w4 4001e504 2\nw4 4001e50c 1\nw4 4001e514 1\nr\nexit\n" > erase-all.jlink
	echo "device nrf51822\nw4 4001e504 2\nw4 4001e50c 1\nw4 4001e514 1\nsleep 100\nr\nexit\n" >  erase-all.jlink
startdebug: stopdebug debug.jlink .gdbinit
	$(JLINKGDBSERVER) -if SWD -device nRF51422_xxAC -speed 4000 #-if swd -speed 1000 -port $(GDB_PORT_NUMBER) &
	sleep 1
	$(GDB) $(ELF)

stopdebug:
	-killall $(JLINKGDBSERVER)

.gdbinit:
	echo "target remote localhost:$(GDB_PORT_NUMBER)\nmonitor flash download = 1\nmonitor flash device = nrf51822\nbreak main\nmon reset\n" > .gdbinit

debug.jlink:
	echo "device nrf51422_xxac" > debug.jlink
	
.PHONY: flash flash-softdevice erase-all startdebug stopdebug		
	
