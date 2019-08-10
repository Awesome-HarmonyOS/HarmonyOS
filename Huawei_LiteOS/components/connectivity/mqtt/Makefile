#*******************************************************************************
#  Copyright (c) 2009, 2014 IBM Corp.
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
#     Xiang Rong - 442039 Add makefile to Embedded C client
#*******************************************************************************/

# Note: on OS X you should install XCode and the associated command-line tools

SHELL = /bin/sh
.PHONY: clean, mkdir, install, uninstall, html 

# assume this is normally run in the main Paho directory
ifndef srcdir
  srcdir = MQTTPacket/src
endif

ifndef blddir
  blddir = build/output
endif

ifndef prefix
	prefix = /usr/local
endif

ifndef exec_prefix
	exec_prefix = ${prefix}
endif

bindir = $(exec_prefix)/bin
includedir = $(prefix)/include
libdir = $(exec_prefix)/lib

SOURCE_FILES_C = $(srcdir)/*.c

HEADERS = $(srcdir)/*.h


SAMPLE_FILES_C = pub0sub1 qos0pub
SYNC_SAMPLES = ${addprefix ${blddir}/samples/,${SAMPLE_FILES_C}}


TEST_FILES_C = test1
SYNC_TESTS = ${addprefix ${blddir}/test/,${TEST_FILES_C}}


# The names of libraries to be built
MQTT_EMBED_LIB_C = paho-embed-mqtt3c


# determine current platform
ifeq ($(OS),Windows_NT)
	OSTYPE = $(OS)
else
	OSTYPE = $(shell uname -s)
	MACHINETYPE = $(shell uname -m)
endif

ifeq ($(OSTYPE),Linux)

CC ?= gcc

ifndef INSTALL
INSTALL = install
endif
INSTALL_PROGRAM = $(INSTALL)
INSTALL_DATA =  $(INSTALL) -m 644

MAJOR_VERSION = 1
MINOR_VERSION = 0
VERSION = ${MAJOR_VERSION}.${MINOR_VERSION}

EMBED_MQTTLIB_C_TARGET = ${blddir}/lib${MQTT_EMBED_LIB_C}.so.${VERSION}


CCFLAGS_SO = -g -fPIC -Os -Wall -fvisibility=hidden -DLINUX_SO
FLAGS_EXE = -I ${srcdir}  -L ${blddir}

LDFLAGS_C = -shared -Wl,-soname,lib$(MQTT_EMBED_LIB_C).so.${MAJOR_VERSION}

all: build
	
build: | mkdir ${EMBED_MQTTLIB_C_TARGET} ${SYNC_SAMPLES} ${SYNC_TESTS}

clean:
	rm -rf ${blddir}/*
	
mkdir:
	-mkdir -p ${blddir}/samples
	-mkdir -p ${blddir}/test

${SYNC_TESTS}: ${blddir}/test/%: ${srcdir}/../test/%.c
	${CC} -g -o ${blddir}/test/${basename ${+F}} $< -l${MQTT_EMBED_LIB_C} ${FLAGS_EXE}


${SYNC_SAMPLES}: ${blddir}/samples/%: ${srcdir}/../samples/%.c ${srcdir}/../samples/transport.o
	${CC} -o $@ $^ -l${MQTT_EMBED_LIB_C} ${FLAGS_EXE}



${EMBED_MQTTLIB_C_TARGET}: ${SOURCE_FILES_C} ${HEADERS_C}
	${CC} ${CCFLAGS_SO} -o $@ ${SOURCE_FILES_C} ${LDFLAGS_C}
	-ln -s lib$(MQTT_EMBED_LIB_C).so.${VERSION}  ${blddir}/lib$(MQTT_EMBED_LIB_C).so.${MAJOR_VERSION}
	-ln -s lib$(MQTT_EMBED_LIB_C).so.${MAJOR_VERSION} ${blddir}/lib$(MQTT_EMBED_LIB_C).so


strip_options:
	$(eval INSTALL_OPTS := -s)

install-strip: build strip_options install

install: build 
	$(INSTALL_DATA) ${INSTALL_OPTS} ${EMBED_MQTTLIB_C_TARGET} $(DESTDIR)${libdir}


	/sbin/ldconfig $(DESTDIR)${libdir}
	ln -s lib$(MQTT_EMBED_LIB_C).so.${MAJOR_VERSION} $(DESTDIR)${libdir}/lib$(MQTT_EMBED_LIB_C).so


uninstall:
	rm $(DESTDIR)${libdir}/lib$(MQTT_EMBED_LIB_C).so.${VERSION}

	/sbin/ldconfig $(DESTDIR)${libdir}
	rm $(DESTDIR)${libdir}/lib$(MQTT_EMBED_LIB_C).so


html:

ARDUINO_LIB_FILES = MQTTClient/src/*.h MQTTClient/src/arduino/*.h $(srcdir)/*
ARDUINO_SAMPLES = MQTTClient/samples/arduino/*
LEGAL_FILES = edl-v10 epl-v10 notice.html about.html CONTRIBUTING.md README.md library.properties

arduino: mkdir
	-mkdir -p ${blddir}/arduino/MQTTClient/examples
	-mkdir -p ${blddir}/arduino/MQTTClient/src
	cp $(ARDUINO_LIB_FILES) ${blddir}/arduino/MQTTClient/src
	cp $(LEGAL_FILES) ${blddir}/arduino/MQTTClient
	cp -R $(ARDUINO_SAMPLES) ${blddir}/arduino/MQTTClient/examples
	cd ${blddir}/arduino && zip -r arduino MQTTClient

endif



ifeq ($(OSTYPE),Darwin)

CC ?= gcc

ifndef INSTALL
INSTALL = install
endif
INSTALL_PROGRAM = $(INSTALL)
INSTALL_DATA =  $(INSTALL) -m 644

MAJOR_VERSION = 1
MINOR_VERSION = 0
VERSION = ${MAJOR_VERSION}.${MINOR_VERSION}

EMBED_MQTTLIB_C_TARGET = ${blddir}/lib${MQTT_EMBED_LIB_C}.so.${VERSION}


CCFLAGS_SO = -g -fPIC -Os -Wall -fvisibility=hidden -Wno-deprecated-declarations -DUSE_NAMED_SEMAPHORES
FLAGS_EXE = -I ${srcdir}  -L ${blddir}

LDFLAGS_C = -shared -Wl,-install_name,lib$(MQTT_EMBED_LIB_C).so.${MAJOR_VERSION}

all: build
	
build: | mkdir ${EMBED_MQTTLIB_C_TARGET} ${SYNC_SAMPLES} ${SYNC_TESTS}

clean:
	rm -rf ${blddir}/*
	
mkdir:
	-mkdir -p ${blddir}/samples
	-mkdir -p ${blddir}/test

${SYNC_TESTS}: ${blddir}/test/%: ${srcdir}/../test/%.c
	${CC} -g -o ${blddir}/test/${basename ${+F}} $< -l${MQTT_EMBED_LIB_C} ${FLAGS_EXE}

${SYNC_SAMPLES}: ${blddir}/samples/%: ${srcdir}/../samples/%.c
	${CC} -o ${blddir}/samples/${basename ${+F}} $< ${FLAGS_EXE} -l${MQTT_EMBED_LIB_C} 

${EMBED_MQTTLIB_C_TARGET}: ${SOURCE_FILES_C} ${HEADERS_C}
	${CC} ${CCFLAGS_SO} -o $@ ${SOURCE_FILES_C} ${LDFLAGS_C}
	-ln -s lib$(MQTT_EMBED_LIB_C).so.${VERSION}  ${blddir}/lib$(MQTT_EMBED_LIB_C).so.${MAJOR_VERSION}
	-ln -s lib$(MQTT_EMBED_LIB_C).so.${MAJOR_VERSION} ${blddir}/lib$(MQTT_EMBED_LIB_C).so


strip_options:
	$(eval INSTALL_OPTS := -s)

install-strip: build strip_options install

install: build 
	$(INSTALL_DATA) ${INSTALL_OPTS} ${EMBED_MQTTLIB_C_TARGET} $(DESTDIR)${libdir}

	/sbin/ldconfig $(DESTDIR)${libdir}
	ln -s lib$(MQTT_EMBED_LIB_C).so.${MAJOR_VERSION} $(DESTDIR)${libdir}/lib$(MQTT_EMBED_LIB_C).so


uninstall:
	rm $(DESTDIR)${libdir}/lib$(MQTT_EMBED_LIB_C).so.${VERSION}
	/sbin/ldconfig $(DESTDIR)${libdir}
	rm $(DESTDIR)${libdir}/lib$(MQTT_EMBED_LIB_C).so


html:


endif
