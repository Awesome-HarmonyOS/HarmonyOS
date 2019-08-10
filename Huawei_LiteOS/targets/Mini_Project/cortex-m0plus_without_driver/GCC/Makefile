##########################################################################################################################
# cortex-m0_without_driver GCC compiler Makefile
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
# ------------------------------------------------
include config.mk
include prune.mk
######################################
# target
######################################
TARGET = Huawei_LiteOS
######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


USE_OTA := no
OTA_IS_NEED_DTLS := no
ifeq ($(USE_FOTA), yes)
	USE_OTA = yes
endif

ifeq ($(USE_OTA), yes)
    ifneq ($(OTA_PACK_CHECKSUM), NO_CHECKSUM)
	OTA_IS_NEED_DTLS = yes
    endif
endif


#######################################
# binaries
#######################################
PREFIX    = arm-none-eabi-
CC        = $(PREFIX)gcc
AS        = $(PREFIX)gcc -x assembler-with-cpp
OBJCOPY   = $(PREFIX)objcopy
OBJDUMP   = $(PREFIX)objdump
AR        = $(PREFIX)ar
SZ        = $(PREFIX)size
LD        = $(PREFIX)ld
HEX       = $(OBJCOPY) -O ihex
BIN       = $(OBJCOPY) -O binary -S


PROJECTBASE = $(PWD)
override PROJECTBASE    := $(abspath $(PROJECTBASE))
TOP_DIR = $(PROJECTBASE)/../../../..
TARGET_DIR = $(PROJECTBASE)/..

#######################################
# paths
#######################################
# firmware library path
PERIFLIB_PATH =

# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
KERNEL_SRC =  \
        ${wildcard $(TOP_DIR)/kernel/*.c} \
        ${wildcard $(TOP_DIR)/kernel/base/core/*.c} \
        ${wildcard $(TOP_DIR)/kernel/base/ipc/*.c} \
        ${wildcard $(TOP_DIR)/kernel/base/mem/bestfit_little/*.c} \
        ${wildcard $(TOP_DIR)/kernel/base/mem/membox/*.c} \
        ${wildcard $(TOP_DIR)/kernel/base/mem/common/*.c} \
        ${wildcard $(TOP_DIR)/kernel/base/misc/*.c} \
        ${wildcard $(TOP_DIR)/kernel/base/om/*.c} \
        ${wildcard $(TOP_DIR)/kernel/extended/tickless/*.c}
        C_SOURCES += $(KERNEL_SRC)

CMSIS_SRC =  \
        ${wildcard $(TOP_DIR)/osdepends/liteos/cmsis/*.c}
        C_SOURCES += $(CMSIS_SRC)

ARCH_SRC =  \
        ${wildcard $(TOP_DIR)/arch/arm/arm-m/src/*.c} \
        ${wildcard $(TOP_DIR)/arch/arm/arm-m/cortex-m0/*.c}
        C_SOURCES += $(ARCH_SRC)
		
ifeq ($(WITH_LWIP), yes)
LWIP_SRC =  \
        ${wildcard $(TOP_DIR)/components/net/lwip/lwip-2.0.3/src/api/*.c} \
        ${wildcard $(TOP_DIR)/components/net/lwip/lwip-2.0.3/src/core/ipv4/*.c} \
		${wildcard $(TOP_DIR)/components/net/lwip/lwip-2.0.3/src/core/ipv6/*.c} \
        ${wildcard $(TOP_DIR)/components/net/lwip/lwip-2.0.3/src/core/*.c} \
        ${wildcard $(TOP_DIR)/components/net/lwip/lwip-2.0.3/src/netif/ethernet.c}
        C_SOURCES += $(LWIP_SRC)

LWIP_PORT_SRC = \
        ${wildcard $(TOP_DIR)/components/net/lwip/lwip_port/OS/*.c}
        C_SOURCES += $(LWIP_PORT_SRC)
endif

ifneq ($(OTA_IS_NEED_DTLS)_$(WITH_DTLS), no_no)
MBEDTLS_SRC = \
        ${wildcard $(TOP_DIR)/components/security/mbedtls/mbedtls-2.6.0/library/*.c}
        C_SOURCES += $(MBEDTLS_SRC)

MBEDTLS_PORT_SRC = \
        ${wildcard $(TOP_DIR)/components/security/mbedtls/mbedtls_port/*.c}
        C_SOURCES += $(MBEDTLS_PORT_SRC)
endif

ifeq ($(WITH_LWM2M), yes)
ER_COAP_SRC = \
        ${wildcard $(TOP_DIR)/components/connectivity/lwm2m/core/er-coap-13/er-coap-13.c}
        C_SOURCES += $(ER_COAP_SRC)

LWM2M_SRC = \
        ${wildcard $(TOP_DIR)/components/connectivity/lwm2m/core/*.c} \
		${wildcard $(TOP_DIR)/components/connectivity/lwm2m/examples/shared/*.c}
        C_SOURCES += $(LWM2M_SRC)
		
ATINY_TINY_SRC = \
        ${wildcard $(TOP_DIR)/components/connectivity/agent_tiny/atiny_lwm2m/*.c}
ifeq ($(USE_FOTA), yes)
	C_SOURCES += $(ATINY_TINY_SRC)
else
	ATINY_TINY_SRC_NO_FOTA = \
			$(filter-out $(TOP_DIR)/components/connectivity/agent_tiny/atiny_lwm2m/atiny_fota_manager.c \
			$(TOP_DIR)/components/connectivity/agent_tiny/atiny_lwm2m/atiny_fota_state.c \
			$(TOP_DIR)/components/connectivity/agent_tiny/atiny_lwm2m/firmware_update.c, $(ATINY_TINY_SRC))
	C_SOURCES += $(ATINY_TINY_SRC_NO_FOTA)
endif
		
AGENT_DEMO_SRC = \
        ${wildcard $(TOP_DIR)/demos/agenttiny_lwm2m/*.c}
        C_SOURCES += $(AGENT_DEMO_SRC)

endif



OS_DEPENDS_SRC = \
        ${wildcard $(TOP_DIR)/osdepends/liteos/*.c}
        C_SOURCES += $(OS_DEPENDS_SRC)

SOCKET_ADAPTER_SRC = \
        ${wildcard $(TOP_DIR)/components/net/sal/*.c}
        C_SOURCES += $(SOCKET_ADAPTER_SRC)

NEWLIB_SRC = \
        ${wildcard $(TOP_DIR)/components/lib/libc/malloc.c}
        C_SOURCES += $(NEWLIB_SRC)
		

ATINY_LOG = \
        ${wildcard $(TOP_DIR)/components/log/*.c}
        C_SOURCES += $(ATINY_LOG)


ifeq ($(WITH_MQTT), yes)
MQTT_SRC = \
        ${wildcard $(TOP_DIR)/components/connectivity/mqtt/MQTTPacket/src/*.c} \
        $(TOP_DIR)/components/connectivity/mqtt/MQTTClient-C/src/MQTTClient.c \
        $(TOP_DIR)/components/connectivity/mqtt/MQTTClient-C/src/liteOS/MQTTliteos.c
        C_SOURCES += $(MQTT_SRC)
ATINY_TINY_MQTT_SRC = \
        ${wildcard $(TOP_DIR)/components/connectivity/agent_tiny/mqtt_client/*.c}
        C_SOURCES += $(ATINY_TINY_MQTT_SRC)
AGENT_DEMO_MQTT_SRC = \
        ${wildcard $(TOP_DIR)/demos/agenttiny_mqtt/*.c}
        C_SOURCES += $(AGENT_DEMO_MQTT_SRC)
endif

ifeq ($(USE_FOTA), yes)
OTA_SRC = \
    ${wildcard  $(TOP_DIR)/components/ota/flag_operate/*.c} \
    ${wildcard  $(TOP_DIR)/components/ota/package/*.c} \
	${wildcard  $(TOP_DIR)/components/ota/utility/*.c}
    C_SOURCES += $(OTA_SRC)
endif


USER_SRC =  \
        $(TARGET_DIR)/Src/main.c \
		$(TARGET_DIR)/Src/user_task.c \
		$(TARGET_DIR)/Src/__hal_simulate.c  
   
        C_SOURCES += $(USER_SRC)



# ASM sources
ASM_SOURCES_s =  \
        ${wildcard $(PROJECTBASE)/startup_LPC51U68.s}

ASM_SOURCES_S =  \
       ${wildcard $(TOP_DIR)/arch/arm/arm-m/cortex-m0/gcc/los_dispatch_gcc.S}


######################################
# firmware library
######################################
PERIFLIB_SOURCES =


#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m0plus
# fpu
#FPU = -mfpu=fpv4-sp-d16
# float-abi
#FLOAT-ABI = -mfloat-abi=hard
# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS =

# C defines
C_DEFS =  \
        -D LWM2M_LITTLE_ENDIAN \
        -D LWM2M_CLIENT_MODE

ifeq ($(WITH_LWIP), yes)
		C_DEFS += -DWITH_LWIP
		C_DEFS += -DLWIP_TIMEVAL_PRIVATE=0 
	ifeq ($(USE_LWIP_TCP), yes)
		C_DEFS += -DLWIP_TCP=1
	else
		C_DEFS += -DLWIP_TCP=0
	endif
endif


ifneq ($(OTA_IS_NEED_DTLS)_$(WITH_DTLS), no_no)
		C_DEFS += -D MBEDTLS_CONFIG_FILE=\"los_mbedtls_config.h\"
	ifeq ($(USE_MBEDTLS_DEBUG_C), yes)
		C_DEFS += -DMBEDTLS_DEBUG_C
	endif

	ifeq ($(USE_MBEDTLS_AES_ROM_TABLES), yes)
		C_DEFS += -DMBEDTLS_AES_ROM_TABLES
	endif
endif
ifeq ($(WITH_DTLS), yes)
		C_DEFS += -DWITH_DTLS
endif



ifeq ($(USE_FOTA), yes)
		C_DEFS += -DCONFIG_FEATURE_FOTA
endif


ifeq ($(LWM2M_BOOTSTRAP), yes)
		C_DEFS += -DLWM2M_BOOTSTRAP
endif


ifeq ($(LWM2M_WITH_LOGS), yes)
		C_DEFS += -DLWM2M_WITH_LOGS
endif

ifeq ($(ATINY_DEBUG), yes)
		C_DEFS += -DATINY_DEBUG
endif




# AS includes
AS_INCLUDES =

# C includes
KERNEL_INC = \
        -I $(TOP_DIR)/kernel/include \
		-I $(TOP_DIR)/kernel/base/include \
		-I $(TOP_DIR)/kernel/extended/include
        C_INCLUDES += $(KERNEL_INC)

INCLUDE_INC = \
        -I $(TOP_DIR)/include
        C_INCLUDES += $(INCLUDE_INC)

LWM2M_ATINY_INC = \
        -I $(TOP_DIR)/components/connectivity/agent_tiny/atiny_lwm2m        
        C_INCLUDES += $(LWM2M_ATINY_INC)

ARCH_INC = \
        -I $(TOP_DIR)/arch/arm/arm-m/include \
	-I $(TOP_DIR)/arch/arm/common/cmsis \
	-I $(TOP_DIR)/arch/arm/arm-m/cortex-m0
        C_INCLUDES += $(ARCH_INC)

LITEOS_CMSIS = \
        -I $(TOP_DIR)/osdepends/liteos/cmsis
        C_INCLUDES += $(LITEOS_CMSIS)
		

LWIP_INC = \
        -I $(TOP_DIR)/components/net/lwip/lwip-2.0.3/src/include
        C_INCLUDES += $(LWIP_INC)

LWIP_PORT_INC = \
        -I $(TOP_DIR)/components/net/lwip/lwip_port \
        -I $(TOP_DIR)/components/net/lwip/lwip_port/OS
        C_INCLUDES += $(LWIP_PORT_INC)


MBEDTLS_INC = \
        -I $(TOP_DIR)/components/security/mbedtls/mbedtls-2.6.0/include
        C_INCLUDES += $(MBEDTLS_INC)

MBEDTLS_PORT_INC = \
        -I $(TOP_DIR)/components/security/mbedtls/mbedtls_port
        C_INCLUDES += $(MBEDTLS_PORT_INC)

ER_COAP_INC = \
        -I $(TOP_DIR)/components/connectivity/lwm2m/core/er-coap-13
        C_INCLUDES += $(ER_COAP_INC)
		
ifeq ($(WITH_LWM2M), yes)
LWM2M_INC = \
        -I $(TOP_DIR)/components/connectivity/lwm2m/core \
        -I $(TOP_DIR)/components/connectivity/lwm2m/examples/shared
        C_INCLUDES += $(LWM2M_INC)

OTA_INC = \
        -I $(TOP_DIR)/components/ota/flag_operate \
        -I $(TOP_DIR)/components/ota/utility
        C_INCLUDES += $(OTA_INC)
endif

ifeq ($(WITH_MQTT), yes)	
MQTT_INC = \
        -I $(TOP_DIR)/components/connectivity/mqtt/MQTTClient-C/src \
        -I $(TOP_DIR)/components/connectivity/mqtt/MQTTClient-C/src/liteOS \
		-I $(TOP_DIR)/components/connectivity/mqtt/MQTTPacket/src
        C_INCLUDES += $(MQTT_INC)
endif

DEMO_INC = \
        -I $(TOP_DIR)/demos
        C_INCLUDES += $(DEMO_INC)


USER_INC = \
        -I $(TARGET_DIR)/Inc \
        -I $(TARGET_DIR)/OS_CONFIG
        C_INCLUDES += $(USER_INC)



# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

CFLAGS += -DCPU_LPC51U68JBD64 -DNDEBUG -Wall -fno-common -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -mthumb -mapcs -std=gnu99
CFLAGS += -mfloat-abi=soft
# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$@"


#######################################
# LDFLAGS
#######################################
# link script

LD_FILE = LPC51U68_flash.ld

LDSCRIPT = $(PROJECTBASE)/$(LD_FILE)

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = -L$(PROJECTBASE)/../Lib
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES_s:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES_s)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES_S:.S=.o)))
vpath %.S $(sort $(dir $(ASM_SOURCES_S)))

$(BUILD_DIR)/%.o: %.c Makefile config.mk prune.mk | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile config.mk prune.mk | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.S Makefile config.mk prune.mk | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile config.mk prune.mk
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir $@

#######################################
# clean up
#######################################
clean:
	-rm -fR .dep $(BUILD_DIR)

#######################################
# dependencies
#######################################
#-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***