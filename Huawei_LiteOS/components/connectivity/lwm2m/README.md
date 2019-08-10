Wakaama (formerly liblwm2m) is an implementation of the Open Mobile Alliance's LightWeight M2M
protocol (LWM2M).

Developers mailing list: https://dev.eclipse.org/mailman/listinfo/wakaama-dev

## Source Layout

    -+- core                   (the LWM2M engine)
     |    |
     |    +- er-coap-13        (Slightly modified Erbium's CoAP engine from
     |                          http://people.inf.ethz.ch/mkovatsc/erbium.php
     |
     +- tests                  (test cases)
     |
     +- examples
          |
          +- bootstrap_server  (a command-line LWM2M bootstrap server)
          |
          +- client            (a command-line LWM2M client with several test objects)
          |
          +- lightclient       (a very simple command-line LWM2M client with several test objects)
          |
          +- server            (a command-line LWM2M server)
          |
          +- shared            (utility functions for connection handling and command-
                                line interface)


## Compiling

Wakaama is not a library but files to be built with an application.
Wakaama uses CMake >= 3. Look at examples/server/CMakeLists.txt for an
example of how to include it.
Several compilation switches are used:
 - LWM2M_BIG_ENDIAN if your target platform uses big-endian format.
 - LWM2M_LITTLE_ENDIAN if your target platform uses little-endian format.
 - LWM2M_CLIENT_MODE to enable LWM2M Client interfaces.
 - LWM2M_SERVER_MODE to enable LWM2M Server interfaces.
 - LWM2M_BOOTSTRAP_SERVER_MODE to enable LWM2M Bootstrap Server interfaces.
 - LWM2M_BOOTSTRAP to enable LWM2M Bootstrap support in a LWM2M Client.
 - LWM2M_SUPPORT_JSON to enable JSON payload support (implicit when defining LWM2M_SERVER_MODE)
 - LWM2M_OLD_CONTENT_FORMAT_SUPPORT to support the deprecated content format values for TLV and JSON.

Depending on your platform, you need to define LWM2M_BIG_ENDIAN or LWM2M_LITTLE_ENDIAN.
LWM2M_CLIENT_MODE and LWM2M_SERVER_MODE can be defined at the same time.


## Examples

There are some example applications provided to test the server, client and bootstrap capabilities of Wakaama.
The following recipes assume you are on a unix like platform and you have cmake and make installed.

### Server example
 * Create a build directory and change to that.
 * ``cmake [wakaama directory]/examples/server``
 * ``make``
 * ``./lwm2mserver [Options]``

The lwm2mserver listens on UDP port 5683. It features a basic command line
interface. Type 'help' for a list of supported commands.

Options are:
 - -4		Use IPv4 connection. Default: IPv6 connection

### Test client example
 * Create a build directory and change to that.
 * ``cmake [wakaama directory]/examples/client``
 * ``make``
 * ``./lwm2mclient [Options]``

DTLS feature requires the tinydtls submodule. To include it, on the first run,
use the following commands to retrieve the sources:
 * ``git submodule init``
 * ``git submodule update``

You need to install autoconf and automake to build with tinydtls.

Build with tinydtls:
 * Create a build directory and change to that.
 * ``cmake -DDTLS=1 [wakaama directory]/examples/client``
 * ``make``
 * ``./lwm2mclient [Options]``

The lwm2mclient features nine LWM2M objects:
 - Security Object (id: 0)
 - Server Object (id: 1)
 - Access Control Object (id: 2) as a skeleton
 - Device Object (id: 3) containing hard-coded values from the Example LWM2M
 Client of Appendix E of the LWM2M Technical Specification.
 - Connectivity Monitoring Object (id: 4) as a skeleton
 - Firmware Update Object (id: 5) as a skeleton.
 - Location Object (id: 6) as a skeleton.
 - Connectivity Statistics Object (id: 7) as a skeleton.
 - Test Object (id: 31024) with the following description:

                           Multiple
          Object |  ID   | Instances | Mandatory |
           Test  | 31024 |    Yes    |    No     |

           Resources:
                       Supported    Multiple
           Name | ID | Operations | Instances | Mandatory |  Type   | Range |
           test |  1 |    R/W     |    No     |    Yes    | Integer | 0-255 |
           exec |  2 |     E      |    No     |    Yes    |         |       |
           dec  |  3 |    R/W     |    No     |    Yes    |  Float  |       |

The lwm2mclient opens udp port 56830 and tries to register to a LWM2M Server at
127.0.0.1:5683. It features a basic command line interface. Type 'help' for a
list of supported commands.

Options are:
- -n NAME	Set the endpoint name of the Client. Default: testlwm2mclient
- -l PORT	Set the local UDP port of the Client. Default: 56830
- -h HOST	Set the hostname of the LWM2M Server to connect to. Default: localhost
- -p HOST	Set the port of the LWM2M Server to connect to. Default: 5683
- -4		Use IPv4 connection. Default: IPv6 connection
- -t TIME	Set the lifetime of the Client. Default: 300
- -b		Bootstrap requested.
- -c		Change battery level over time.
  
If DTLS feature enable:
- -i Set the device management or bootstrap server PSK identity. If not set use none secure mode
- -s Set the device management or bootstrap server Pre-Shared-Key. If not set use none secure mode

To launch a bootstrap session:
``./lwm2mclient -b``

### Simpler test client example

In the any directory, run the following commands:
 * Create a build directory and change to that.
 * ``cmake [wakaama directory]/examples/lightclient``
 * ``make``
 * ``./lightclient [Options]``

The lightclient is much simpler that the lwm2mclient and features only four
LWM2M objects:
 - Security Object (id: 0)
 - Server Object (id: 1)
 - Device Object (id: 3) containing hard-coded values from the Example LWM2M
 Client of Appendix E of the LWM2M Technical Specification.
 - Test Object (id: 31024) from the lwm2mclient as described above.

The lightclient does not feature any command-line interface.

Options are:
 -  -n NAME	Set the endpoint name of the Client. Default: testlightclient
 - -l PORT	Set the local UDP port of the Client. Default: 56830
 - -4		Use IPv4 connection. Default: IPv6 connection

