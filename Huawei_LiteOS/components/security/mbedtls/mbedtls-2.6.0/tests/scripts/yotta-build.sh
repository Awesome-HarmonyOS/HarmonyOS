#!/bin/sh

# yotta-build.sh
#
# This file is part of mbed TLS (https://tls.mbed.org)
#
# Copyright (c) 2015-2016, ARM Limited, All Rights Reserved
#
# Purpose
#
# To run test builds of the yotta module for all supported targets.

set -eu

check_tools()
{
    for TOOL in "$@"; do
        if ! `hash "$TOOL" >/dev/null 2>&1`; then
            echo "$TOOL not found!" >&2
            exit 1
        fi
    done
}

yotta_build()
{
    TARGET=$1

    echo; echo "*** $TARGET (release) ***"
    yt -t $TARGET build

    echo; echo "*** $TARGET (debug) ***"
    yt -t $TARGET build -d
}

# Make sure the tools we need are available.
check_tools "arm-none-eabi-gcc" "armcc" "yotta"

yotta/create-module.sh
cd yotta/module
yt update || true # needs network

if uname -a | grep 'Linux.*x86' >/dev/null; then
    yotta_build x86-linux-native
fi
if uname -a | grep 'Darwin.*x86' >/dev/null; then
    yotta_build x86-osx-native
fi

# armcc build tests.
yotta_build frdm-k64f-armcc
#yotta_build nordic-nrf51822-16k-armcc

# arm-none-eabi-gcc build tests.
yotta_build frdm-k64f-gcc
#yotta_build st-nucleo-f401re-gcc # dirent
#yotta_build stm32f429i-disco-gcc # fails in mbed-hal-st-stm32f4
#yotta_build nordic-nrf51822-16k-gcc # fails in minar-platform
#yotta_build bbc-microbit-classic-gcc # fails in minar-platform
#yotta_build st-stm32f439zi-gcc # fails in mbed-hal-st-stm32f4
#yotta_build st-stm32f429i-disco-gcc # fails in mbed-hal-st-stm32f4
