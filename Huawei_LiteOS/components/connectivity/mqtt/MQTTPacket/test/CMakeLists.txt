PROJECT(mqttpacket-tests)

IF (WIN32)
  SET(MQTT_TEST_BROKER "tcp://mqtt.iotree.co.uk:1883" CACHE STRING "Hostname of a test MQTT broker to use")
  SET(MQTT_TEST_PROXY "tcp://localhost:1883" CACHE STRING "Hostname of the test proxy to use")
  SET(MQTT_SSL_HOSTNAME "mqtt.iotree.co.uk" CACHE STRING "Hostname of a test SSL MQTT broker to use")
  SET(CERTDIR $ENV{APPVEYOR_BUILD_FOLDER}/test/ssl)
ELSE ()
  SET(MQTT_TEST_BROKER "tcp://localhost:1883" CACHE STRING "Hostname of a test MQTT broker to use")
  SET(MQTT_TEST_PROXY "tcp://localhost:1884" CACHE STRING "Hostname of the test proxy to use")
  SET(MQTT_SSL_HOSTNAME "localhost" CACHE STRING "Hostname of a test SSL MQTT broker to use")
  SET(CERTDIR $ENV{TRAVIS_BUILD_DIR}/test/ssl)
ENDIF ()

include_directories(../src)

ADD_EXECUTABLE(
	test1
	test1.c
)

TARGET_LINK_LIBRARIES(
	test1
	paho-embed-mqtt3c
)

ADD_TEST(
	NAME test1
	COMMAND "test1" "--connection" ${MQTT_TEST_BROKER}
)

SET_TESTS_PROPERTIES(
	test1
	PROPERTIES TIMEOUT 540
)
