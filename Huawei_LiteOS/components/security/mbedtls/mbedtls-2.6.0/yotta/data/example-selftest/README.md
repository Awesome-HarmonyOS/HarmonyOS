# mbed TLS Selftest Example

This application runs the various selftest functions of individual mbed TLS components. It serves as a basic sanity check to verify operation of mbed TLS on your platform. In the future, a wider portion of the mbed TLS test suite will become part of this example application.

## Pre-requisites

To build and run this example you must have:

* A computer with the following software installed:
  * [CMake](http://www.cmake.org/download/).
  * [yotta](https://github.com/ARMmbed/yotta). Please note that **yotta has its own set of dependencies**, listed in the [installation instructions](http://armmbed.github.io/yotta/#installing-on-windows).
  * [Python](https://www.python.org/downloads/).
  * [The ARM GCC toolchain](https://launchpad.net/gcc-arm-embedded).
  * A serial terminal emulator (Like screen, pySerial and cu).
* An [FRDM-K64F](http://developer.mbed.org/platforms/FRDM-K64F/) development board, or another board supported by mbed OS (in which case you'll have to substitute frdm-k64f-gcc with the appropriate target in the instructions below).
* A micro-USB cable.
* If your OS is Windows, please follow the installation instructions [for the serial port driver](https://developer.mbed.org/handbook/Windows-serial-configuration).

## Getting started

1. Connect the FRDM-K64F to the computer with the micro-USB cable, being careful to use the "OpenSDA" connector on the target board.

2. Navigate to the mbedtls directory supplied with your release and open a terminal.

3. Set the yotta target:

    ```
    yotta target frdm-k64f-gcc
    ```

4. Build mbedtls and the examples. This may take a long time if this is your first compilation:

    ```
    $ yotta build
    ```

5. Copy `build/frdm-k64f-gcc/test/mbedtls-test-example-selftest.bin` to your mbed board and wait until the LED next to the USB port stops blinking.

6. Start the serial terminal emulator and connect to the virtual serial port presented by FRDM-K64F. 

	Use the following settings:

	* 115200 baud (not 9600).
	* 8N1.
	* No flow control. 

7. Press the Reset button on the board.

8. The output in the terminal window should look like:

    ```
    {{timeout;40}}
    {{host_test_name;default}}
    {{description;mbed TLS selftest program}}
    {{test_id;MBEDTLS_SELFTEST}}
    {{start}}

      SHA-224 test #1: passed
      SHA-224 test #2: passed
      SHA-224 test #3: passed
      SHA-256 test #1: passed
      SHA-256 test #2: passed
      SHA-256 test #3: passed

        [ ... several lines omitted ... ]

      CTR_DRBG (PR = TRUE) : passed
      CTR_DRBG (PR = FALSE): passed

      HMAC_DRBG (PR = True) : passed
      HMAC_DRBG (PR = False) : passed

      ECP test #1 (constant op_count, base point G): passed
      ECP test #2 (constant op_count, other point): passed

      ENTROPY test: passed

      [ All tests passed ]

    {{success}}
    {{end}}
    ```
