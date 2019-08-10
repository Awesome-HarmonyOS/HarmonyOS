#*******************************************************************************
#  Copyright (c) 2017 IBM Corp.
#
#  All rights reserved. This program and the accompanying materials
#  are made available under the terms of the Eclipse Public License v1.0
#  and Eclipse Distribution License v1.0 which accompany this distribution.
#
#  The Eclipse Public License is available at
#     http://www.eclipse.org/legal/epl-v10.html
#  and the Eclipse Distribution License is available at
#    http://www.eclipse.org/org/documents/edl-v10.php.
#
#  Contributors:
#     Ian Craggs - initial version
#*******************************************************************************/

# MQTTClient Library - C


file(GLOB SOURCES "*.c" "linux/*.c")

add_library(
  paho-embed-mqtt3cc SHARED
  ${SOURCES}
)
install(TARGETS paho-embed-mqtt3cc DESTINATION /usr/lib)
target_include_directories(paho-embed-mqtt3cc PRIVATE "linux")
target_link_libraries(paho-embed-mqtt3cc paho-embed-mqtt3c)
target_compile_definitions(paho-embed-mqtt3cc PRIVATE
             MQTTCLIENT_PLATFORM_HEADER=MQTTLinux.h MQTTCLIENT_QOS2=1)
