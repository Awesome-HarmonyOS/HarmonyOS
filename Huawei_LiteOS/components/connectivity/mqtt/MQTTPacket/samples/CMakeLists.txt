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

# Samples

include_directories(../src)

add_executable(
  pub0sub1
  pub0sub1.c transport.c
)
target_link_libraries(pub0sub1 paho-embed-mqtt3c)

add_executable(
  pub0sub1_nb
  pub0sub1_nb.c transport.c
)
target_link_libraries(pub0sub1_nb paho-embed-mqtt3c)

add_executable(
  qos0pub
  qos0pub.c transport.c
)
target_link_libraries(qos0pub paho-embed-mqtt3c)
