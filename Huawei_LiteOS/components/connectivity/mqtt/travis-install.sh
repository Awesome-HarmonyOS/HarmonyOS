#!/bin/bash

if [ "$TRAVIS_OS_NAME" == "linux" ]; then
	pwd
	sudo service mosquitto stop
	# Stop any mosquitto instance which may be still running from previous runs
	killall mosquitto
	mosquitto -h
	mosquitto &
fi

if [ "$TRAVIS_OS_NAME" == "osx" ]; then
	pwd
	brew update
	brew install openssl mosquitto
	brew services stop mosquitto
	/usr/local/sbin/mosquitto -h
	/usr/local/sbin/mosquitto &
fi
