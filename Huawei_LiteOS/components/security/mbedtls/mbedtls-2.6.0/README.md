README for mbed TLS
===================

Configuration
-------------

mbed TLS should build out of the box on most systems. Some platform specific options are available in the fully documented configuration file `include/mbedtls/config.h`, which is also the place where features can be selected. This file can be edited manually, or in a more programmatic way using the Perl script `scripts/config.pl` (use `--help` for usage instructions).

Compiler options can be set using conventional environment variables such as `CC` and `CFLAGS` when using the Make and CMake build system (see below).

Compiling
---------

There are currently four active build systems used within mbed TLS releases:

-   yotta
-   Make
-   CMake
-   Microsoft Visual Studio (Visual Studio 6 and Visual Studio 2010)

The main systems used for development are CMake and Make. Those systems are always complete and up-to-date. The others should reflect all changes present in the CMake and Make build system, although features may not be ported there automatically.

Yotta, as a build system, is slightly different from the other build systems:

-   it provides a minimalistic configuration file by default
-   depending on the yotta target, features of mbed OS may be used in examples and tests

The Make and CMake build systems create three libraries: libmbedcrypto, libmbedx509, and libmbedtls. Note that libmbedtls depends on libmbedx509 and libmbedcrypto, and libmbedx509 depends on libmbedcrypto. As a result, some linkers will expect flags to be in a specific order, for example the GNU linker wants `-lmbedtls -lmbedx509 -lmbedcrypto`. Also, when loading shared libraries using dlopen(), you'll need to load libmbedcrypto first, then libmbedx509, before you can load libmbedtls.

### Yotta

[yotta](http://yottabuild.org) is a package manager and build system developed by mbed, and is the build system of mbed OS 16.03. To install it on your platform, please follow the yotta [installation instructions](http://docs.yottabuild.org/#installing).

Once yotta is installed, you can use it to download the latest version of mbed TLS from the yotta registry with:

    yotta install mbedtls

and build it with:

    yotta build

If, on the other hand, you already have a copy of mbed TLS from a source other than the yotta registry, for example from cloning our GitHub repository, or from downloading a tarball of the standalone edition, then you'll first need to generate the yotta module by running:

    yotta/create-module.sh

This should be executed from the root mbed TLS project directory. This will create the yotta module in the `yotta/module` directory within it. You can then change to that directory and build as usual:

    cd yotta/module
    yotta build

In any case, you'll probably want to set the yotta target before building unless it has already been set globally. For more information on using yotta, please consult the [yotta documentation](http://docs.yottabuild.org/).

For more details on the yotta/mbed OS edition of mbed TLS, including example programs, please consult the [Readme at the root of the yotta module](https://github.com/ARMmbed/mbedtls/blob/development/yotta/data/README.md).

### Make

We intentionally only use the minimum of `Make` functionality, as a lot of `Make` features are not supported on all different implementations of Make or on different platforms. As such, the Makefiles sometimes require some manual changes or export statements in order to work for your platform.

In order to build from the source code using Make, just enter at the command line:

    make

In order to run the tests, enter:

    make check

The tests need Perl to be built and run. If you don't have Perl installed, you can skip building the tests with:

    make no_test

You'll still be able to run a much smaller set of tests with:

    programs/test/selftest

In order to build for a Windows platform, you should use `WINDOWS_BUILD=1` if the target is Windows but the build environment is Unix-like (for instance when cross-compiling, or compiling from an MSYS shell), and `WINDOWS=1` if the build environment is a Windows shell (for instance using mingw32-make) (in that case some targets will not be available).

Setting the variable `SHARED` in your environment will build shared libraries in addition to the static libraries. Setting `DEBUG` gives you a debug build. You can override `CFLAGS` and `LDFLAGS` by setting them in your environment or on the make command line; if you do so, essential parts such as `-I` will still be preserved. Warning options may be overridden separately using `WARNING_CFLAGS`.

Depending on your platform, you might run into some issues. Please check the Makefiles in `library/`, `programs/` and `tests/` for options to manually add or remove for specific platforms. You can also check [the mbed TLS Knowledge Base](https://tls.mbed.org/kb) for articles on your platform or issue.

In case you find that you need to do something else as well, please let us know what, so we can add it to the [mbed TLS knowledge base](https://tls.mbed.org/kb).

### CMake

In order to build the source using CMake, just enter at the command line:

    cmake .
    make

In order to run the tests, enter:

    make test

The test suites need Perl to be built. If you don't have Perl installed, you'll want to disable the test suites with:

    cmake -DENABLE_TESTING=Off .

If you disabled the test suites, but kept the programs enabled, you can still run a much smaller set of tests with:

    programs/test/selftest

To configure CMake for building shared libraries, use:

    cmake -DUSE_SHARED_MBEDTLS_LIBRARY=On .

There are many different build modes available within the CMake buildsystem. Most of them are available for gcc and clang, though some are compiler-specific:

-   Release. This generates the default code without any unnecessary information in the binary files.
-   Debug. This generates debug information and disables optimization of the code.
-   Coverage. This generates code coverage information in addition to debug information.
-   ASan. This instruments the code with AddressSanitizer to check for memory errors. (This includes LeakSanitizer, with recent version of gcc and clang.) (With recent version of clang, this mode also instruments the code with UndefinedSanitizer to check for undefined behaviour.)
-   ASanDbg. Same as ASan but slower, with debug information and better stack traces.
-   MemSan. This instruments the code with MemorySanitizer to check for uninitialised memory reads. Experimental, needs recent clang on Linux/x86\_64.
-   MemSanDbg. Same as MemSan but slower, with debug information, better stack traces and origin tracking.
-   Check. This activates the compiler warnings that depend on optimization and treats all warnings as errors.

Switching build modes in CMake is simple. For debug mode, enter at the command line:

    cmake -D CMAKE_BUILD_TYPE=Debug .

To list other available CMake options, use:

    cmake -LH

Note that, with CMake, if you want to change the compiler or its options after you already ran CMake, you need to clear its cache first, e.g. (using GNU find):

    find . -iname '*cmake*' -not -name CMakeLists.txt -exec rm -rf {} +
    CC=gcc CFLAGS='-fstack-protector-strong -Wa,--noexecstack' cmake .

### Microsoft Visual Studio

The build files for Microsoft Visual Studio are generated for Visual Studio 2010.

The solution file `mbedTLS.sln` contains all the basic projects needed to build the library and all the programs. The files in tests are not generated and compiled, as these need a perl environment as well. However, the selftest program in `programs/test/` is still available.

Example programs
----------------

We've included example programs for a lot of different features and uses in `programs/`. Most programs only focus on a single feature or usage scenario, so keep that in mind when copying parts of the code.

Tests
-----

mbed TLS includes an elaborate test suite in `tests/` that initially requires Perl to generate the tests files (e.g. `test\_suite\_mpi.c`). These files are generated from a `function file` (e.g. `suites/test\_suite\_mpi.function`) and a `data file` (e.g. `suites/test\_suite\_mpi.data`). The `function file` contains the test functions. The `data file` contains the test cases, specified as parameters that will be passed to the test function.

For machines with a Unix shell and OpenSSL (and optionally GnuTLS) installed, additional test scripts are available:

-   `tests/ssl-opt.sh` runs integration tests for various TLS options (renegotiation, resumption, etc.) and tests interoperability of these options with other implementations.
-   `tests/compat.sh` tests interoperability of every ciphersuite with other implementations.
-   `tests/scripts/test-ref-configs.pl` test builds in various reduced configurations.
-   `tests/scripts/key-exchanges.pl` test builds in configurations with a single key exchange enabled
-   `tests/scripts/all.sh` runs a combination of the above tests, plus some more, with various build options (such as ASan, full `config.h`, etc).

Configurations
--------------

We provide some non-standard configurations focused on specific use cases in the `configs/` directory. You can read more about those in `configs/README.txt`

Porting mbed TLS
----------------

mbed TLS can be ported to many different architectures, OS's and platforms. Before starting a port, you may find the following knowledge base articles useful:

-   [Porting mbed TLS to a new environment or OS](https://tls.mbed.org/kb/how-to/how-do-i-port-mbed-tls-to-a-new-environment-OS)
-   [What external dependencies does mbed TLS rely on?](https://tls.mbed.org/kb/development/what-external-dependencies-does-mbedtls-rely-on)
-   [How do I configure mbed TLS](https://tls.mbed.org/kb/compiling-and-building/how-do-i-configure-mbedtls)

Contributing
------------

We gratefully accept bug reports and contributions from the community. There are some requirements we need to fulfill in order to be able to integrate contributions:

-   All contributions, whether large or small require a Contributor's License Agreement (CLA) to be accepted. This is because source code can possibly fall under copyright law and we need your consent to share in the ownership of the copyright.
-   We would ask that contributions conform to [our coding standards](https://tls.mbed.org/kb/development/mbedtls-coding-standards), and that contributions should be fully tested before submission.
-   As with any open source project, contributions will be reviewed by the project team and community and may need some modifications to be accepted.

To accept the Contributor’s Licence Agreement (CLA), individual contributors can do this by creating an mbed account and [accepting the online agreement here with a click through](https://developer.mbed.org/contributor_agreement/). Alternatively, for contributions from corporations, or those that do not wish to create an mbed account, a slightly different agreement can be found [here](https://www.mbed.com/en/about-mbed/contributor-license-agreements/). This agreement should be signed and returned to ARM as described in the instructions given.

### Making a Contribution

1.  [Check for open issues](https://github.com/ARMmbed/mbedtls/issues) or [start a discussion](https://tls.mbed.org/discussions) around a feature idea or a bug.
2.  Fork the [mbed TLS repository on GitHub](https://github.com/ARMmbed/mbedtls) to start making your changes. As a general rule, you should use the "development" branch as a basis.
3.  Write a test which shows that the bug was fixed or that the feature works as expected.
4.  Send a pull request and bug us until it gets merged and published. Contributions may need some modifications, so work with us to get your change accepted. We will include your name in the ChangeLog :)

