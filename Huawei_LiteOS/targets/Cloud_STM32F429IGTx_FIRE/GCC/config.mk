##########################################################################################################################
# Cloud_STM32F429IGTx_FIRE GCC compiler config
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
# ------------------------------------------------
# compile option

#######################################
# use USE_BOOTLOADER
#######################################
USE_BOOTLOADER  := no

#######################################
# use Lwm2m protocol
#######################################
WITH_LWM2M  := yes

#######################################
# use MQTT protocol
#######################################
WITH_MQTT  := no


#######################################
# use ethernet
#######################################
WITH_LWIP  := no

#######################################
# use usart AT command
# (NB_NEUL95_NO_ATINY: nb without agenttiny)
# (NB_NEUL95: nb with agenttiny)
#######################################
WITH_AT_FRAMEWORK := yes
ifeq ($(WITH_AT_FRAMEWORK), yes)
#ESP8266   # SIM900A  # NB_NEUL95  # NB_NEUL95_NO_ATINY
	NETWORK_TYPE := NB_NEUL95
#ONLYONE  #ALL
	AT_COMPILE_ALL := ALL
endif

#######################################
# use mbedtls
#######################################
WITH_DTLS := yes

#######################################
# whether OTA Pack use checksum
#######################################
#SHA256_RSA2048   #SHA256  #NO_CHECKSUM
OTA_PACK_CHECKSUM := SHA256_RSA2048

#######################################
# Firmware Over-The-Air
#######################################
USE_FOTA := yes

#######################################
# Firmware Over-The-Air
#######################################
USE_SOTA := yes

#######################################
# Lwm2m bootstrap program 
#######################################
LWM2M_BOOTSTRAP := yes

#######################################
# Lwm2m bootstrap used 
#######################################
SUPPORT_DTLS_SRV := no

#######################################
# Lwm2m core log
#######################################
LWM2M_WITH_LOGS := yes

#######################################
# Agenttiny log
#######################################
ATINY_DEBUG := yes

#######################################
# File System
#######################################
WITH_FILESYSTEM := yes
ifeq ($(WITH_FILESYSTEM), yes)
#SPIFFS   #FATFS
	FILESYSTEM_TYPE := FATFS
#ONLYONE  #ALL
	IS_COMPILE_ALLFS := ALL
endif

#######################################
# CMockery Test
#######################################
WITH_CMOCKERY_TEST := no
