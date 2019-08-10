#! /usr/bin/env sh

# all.sh
#
# This file is part of mbed TLS (https://tls.mbed.org)
#
# Copyright (c) 2014-2016, ARM Limited, All Rights Reserved
#
# Purpose
#
# To run all tests possible or available on the platform.
#
# Warning: the test is destructive. It includes various build modes and
# configurations, and can and will arbitrarily change the current CMake
# configuration. After this script has been run, the CMake cache will be lost
# and CMake will no longer be initialised.
#
# The script assumes the presence of gcc and clang (recent enough for using
# ASan with gcc and MemSan with clang, or valgrind) are available, as well as
# cmake and a "good" find.

# Abort on errors (and uninitialised variables)
set -eu

if [ "$( uname )" != "Linux" ]; then
    echo "This script only works in Linux" >&2
    exit 1
elif [ -d library -a -d include -a -d tests ]; then :; else
    echo "Must be run from mbed TLS root" >&2
    exit 1
fi

CONFIG_H='include/mbedtls/config.h'
CONFIG_BAK="$CONFIG_H.bak"

MEMORY=0
FORCE=0
RELEASE=0

# Default commands, can be overriden by the environment
: ${OPENSSL:="openssl"}
: ${OPENSSL_LEGACY:="$OPENSSL"}
: ${GNUTLS_CLI:="gnutls-cli"}
: ${GNUTLS_SERV:="gnutls-serv"}
: ${GNUTLS_LEGACY_CLI:="$GNUTLS_CLI"}
: ${GNUTLS_LEGACY_SERV:="$GNUTLS_SERV"}
: ${OUT_OF_SOURCE_DIR:=./mbedtls_out_of_source_build}
: ${ARMC5_BIN_DIR:=/usr/bin}
: ${ARMC6_BIN_DIR:=/usr/bin}

# if MAKEFLAGS is not set add the -j option to speed up invocations of make
if [ -n "${MAKEFLAGS+set}" ]; then
    export MAKEFLAGS="-j"
fi

usage()
{
    printf "Usage: $0\n"
    printf "  -h|--help\t\tPrint this help.\n"
    printf "  -m|--memory\t\tAdditional optional memory tests.\n"
    printf "  -f|--force\t\tForce the tests to overwrite any modified files.\n"
    printf "  -s|--seed\t\tInteger seed value to use for this test run.\n"
    printf "  -r|--release-test\t\tRun this script in release mode. This fixes the seed value to 1.\n"
    printf "     --out-of-source-dir=<path>\t\tDirectory used for CMake out-of-source build tests."
    printf "     --openssl=<OpenSSL_path>\t\tPath to OpenSSL executable to use for most tests.\n"
    printf "     --openssl-legacy=<OpenSSL_path>\t\tPath to OpenSSL executable to use for legacy tests e.g. SSLv3.\n"
    printf "     --gnutls-cli=<GnuTLS_cli_path>\t\tPath to GnuTLS client executable to use for most tests.\n"
    printf "     --gnutls-serv=<GnuTLS_serv_path>\t\tPath to GnuTLS server executable to use for most tests.\n"
    printf "     --gnutls-legacy-cli=<GnuTLS_cli_path>\t\tPath to GnuTLS client executable to use for legacy tests.\n"
    printf "     --gnutls-legacy-serv=<GnuTLS_serv_path>\t\tPath to GnuTLS server executable to use for legacy tests.\n"
    printf "     --armc5-bin-dir=<ARMC5_bin_dir_path>\t\tPath to the ARM Compiler 5 bin directory.\n"
    printf "     --armc6-bin-dir=<ARMC6_bin_dir_path>\t\tPath to the ARM Compiler 6 bin directory.\n"
}

# remove built files as well as the cmake cache/config
cleanup()
{
    make clean

    find . -name yotta -prune -o -iname '*cmake*' -not -name CMakeLists.txt -exec rm -rf {} \+
    rm -f include/Makefile include/mbedtls/Makefile programs/*/Makefile
    git update-index --no-skip-worktree Makefile library/Makefile programs/Makefile tests/Makefile
    git checkout -- Makefile library/Makefile programs/Makefile tests/Makefile

    if [ -f "$CONFIG_BAK" ]; then
        mv "$CONFIG_BAK" "$CONFIG_H"
    fi
}

trap cleanup INT TERM HUP

msg()
{
    echo ""
    echo "******************************************************************"
    echo "* $1 "
    printf "* "; date
    echo "******************************************************************"
}

armc6_build_test()
{
    FLAGS="$1"

    msg "build: ARM Compiler 6 ($FLAGS), make"
    ARM_TOOL_VARIANT="ult" CC="$ARMC6_CC" AR="$ARMC6_AR" CFLAGS="$FLAGS" \
        WARNING_CFLAGS='-xc -std=c99' make lib
    make clean
}

err_msg()
{
    echo "$1" >&2
}

check_tools()
{
    for TOOL in "$@"; do
        if ! `hash "$TOOL" >/dev/null 2>&1`; then
            err_msg "$TOOL not found!"
            exit 1
        fi
    done
}

while [ $# -gt 0 ]; do
    case "$1" in
        --memory|-m*)
            MEMORY=${1#-m}
            ;;
        --force|-f)
            FORCE=1
            ;;
        --seed|-s)
            shift
            SEED="$1"
            ;;
        --release-test|-r)
            RELEASE=1
            ;;
        --out-of-source-dir)
            shift
            OUT_OF_SOURCE_DIR="$1"
            ;;
        --openssl)
            shift
            OPENSSL="$1"
            ;;
        --openssl-legacy)
            shift
            OPENSSL_LEGACY="$1"
            ;;
        --gnutls-cli)
            shift
            GNUTLS_CLI="$1"
            ;;
        --gnutls-serv)
            shift
            GNUTLS_SERV="$1"
            ;;
        --gnutls-legacy-cli)
            shift
            GNUTLS_LEGACY_CLI="$1"
            ;;
        --gnutls-legacy-serv)
            shift
            GNUTLS_LEGACY_SERV="$1"
            ;;
        --armc5-bin-dir)
            shift
            ARMC5_BIN_DIR="$1"
            ;;
        --armc6-bin-dir)
            shift
            ARMC6_BIN_DIR="$1"
            ;;
        --help|-h|*)
            usage
            exit 1
            ;;
    esac
    shift
done

if [ $FORCE -eq 1 ]; then
    rm -rf yotta/module "$OUT_OF_SOURCE_DIR"
    git checkout-index -f -q $CONFIG_H
    cleanup
else

    if [ -d yotta/module ]; then
        err_msg "Warning - there is an existing yotta module in the directory 'yotta/module'"
        echo "You can either delete your work and retry, or force the test to overwrite the"
        echo "test by rerunning the script as: $0 --force"
        exit 1
    fi

    if [ -d "$OUT_OF_SOURCE_DIR" ]; then
        echo "Warning - there is an existing directory at '$OUT_OF_SOURCE_DIR'" >&2
        echo "You can either delete this directory manually, or force the test by rerunning"
        echo "the script as: $0 --force --out-of-source-dir $OUT_OF_SOURCE_DIR"
        exit 1
    fi

    if ! git diff-files --quiet include/mbedtls/config.h; then
        echo $?
        err_msg "Warning - the configuration file 'include/mbedtls/config.h' has been edited. "
        echo "You can either delete or preserve your work, or force the test by rerunning the"
        echo "script as: $0 --force"
        exit 1
    fi
fi

if [ $RELEASE -eq 1 ]; then
    # Fix the seed value to 1 to ensure that the tests are deterministic.
    SEED=1
fi

msg "info: $0 configuration"
echo "MEMORY: $MEMORY"
echo "FORCE: $FORCE"
echo "SEED: ${SEED-"UNSET"}"
echo "OPENSSL: $OPENSSL"
echo "OPENSSL_LEGACY: $OPENSSL_LEGACY"
echo "GNUTLS_CLI: $GNUTLS_CLI"
echo "GNUTLS_SERV: $GNUTLS_SERV"
echo "GNUTLS_LEGACY_CLI: $GNUTLS_LEGACY_CLI"
echo "GNUTLS_LEGACY_SERV: $GNUTLS_LEGACY_SERV"
echo "ARMC5_BIN_DIR: $ARMC5_BIN_DIR"
echo "ARMC6_BIN_DIR: $ARMC6_BIN_DIR"

ARMC5_CC="$ARMC5_BIN_DIR/armcc"
ARMC5_AR="$ARMC5_BIN_DIR/armar"
ARMC6_CC="$ARMC6_BIN_DIR/armclang"
ARMC6_AR="$ARMC6_BIN_DIR/armar"

# To avoid setting OpenSSL and GnuTLS for each call to compat.sh and ssl-opt.sh
# we just export the variables they require
export OPENSSL_CMD="$OPENSSL"
export GNUTLS_CLI="$GNUTLS_CLI"
export GNUTLS_SERV="$GNUTLS_SERV"

# Avoid passing --seed flag in every call to ssl-opt.sh
[ ! -z ${SEED+set} ] && export SEED

# Make sure the tools we need are available.
check_tools "$OPENSSL" "$OPENSSL_LEGACY" "$GNUTLS_CLI" "$GNUTLS_SERV" \
    "$GNUTLS_LEGACY_CLI" "$GNUTLS_LEGACY_SERV" "doxygen" "dot" \
    "arm-none-eabi-gcc" "$ARMC5_CC" "$ARMC5_AR" "$ARMC6_CC" "$ARMC6_AR" \
    "i686-w64-mingw32-gcc"

#
# Test Suites to be executed
#
# The test ordering tries to optimize for the following criteria:
# 1. Catch possible problems early, by running first tests that run quickly
#    and/or are more likely to fail than others (eg I use Clang most of the
#    time, so start with a GCC build).
# 2. Minimize total running time, by avoiding useless rebuilds
#
# Indicative running times are given for reference.

msg "info: output_env.sh"
OPENSSL="$OPENSSL" OPENSSL_LEGACY="$OPENSSL_LEGACY" GNUTLS_CLI="$GNUTLS_CLI" \
    GNUTLS_SERV="$GNUTLS_SERV" GNUTLS_LEGACY_CLI="$GNUTLS_LEGACY_CLI" \
    GNUTLS_LEGACY_SERV="$GNUTLS_LEGACY_SERV" ARMC5_CC="$ARMC5_CC" \
    ARMC6_CC="$ARMC6_CC" scripts/output_env.sh

msg "test: recursion.pl" # < 1s
tests/scripts/recursion.pl library/*.c

msg "test: freshness of generated source files" # < 1s
tests/scripts/check-generated-files.sh

msg "test: doxygen markup outside doxygen blocks" # < 1s
tests/scripts/check-doxy-blocks.pl

msg "test/build: declared and exported names" # < 3s
cleanup
tests/scripts/check-names.sh

msg "test: doxygen warnings" # ~ 3s
cleanup
tests/scripts/doxygen.sh

# Note - use of yotta is deprecated, and yotta also requires armcc to be on the
# path, and uses whatever version of armcc it finds there.
msg "build: create and build yotta module" # ~ 30s
cleanup
tests/scripts/yotta-build.sh

msg "build: cmake, gcc, ASan" # ~ 1 min 50s
cleanup
CC=gcc cmake -D CMAKE_BUILD_TYPE:String=Asan .
make

msg "test: main suites (inc. selftests) (ASan build)" # ~ 50s
make test

msg "test: ssl-opt.sh (ASan build)" # ~ 1 min
tests/ssl-opt.sh

msg "test/build: ref-configs (ASan build)" # ~ 6 min 20s
tests/scripts/test-ref-configs.pl

msg "build: with ASan (rebuild after ref-configs)" # ~ 1 min
make

msg "test: compat.sh (ASan build)" # ~ 6 min
tests/compat.sh

msg "build: Default + SSLv3 (ASan build)" # ~ 6 min
cleanup
cp "$CONFIG_H" "$CONFIG_BAK"
scripts/config.pl set MBEDTLS_SSL_PROTO_SSL3
CC=gcc cmake -D CMAKE_BUILD_TYPE:String=Asan .
make

msg "test: SSLv3 - main suites (inc. selftests) (ASan build)" # ~ 50s
make test

msg "build: SSLv3 - compat.sh (ASan build)" # ~ 6 min
tests/compat.sh -m 'tls1 tls1_1 tls1_2 dtls1 dtls1_2'
OPENSSL_CMD="$OPENSSL_LEGACY" tests/compat.sh -m 'ssl3'

msg "build: SSLv3 - ssl-opt.sh (ASan build)" # ~ 6 min
tests/ssl-opt.sh

msg "build: cmake, full config, clang, C99" # ~ 50s
cleanup
cp "$CONFIG_H" "$CONFIG_BAK"
scripts/config.pl full
scripts/config.pl unset MBEDTLS_MEMORY_BACKTRACE # too slow for tests
CC=clang cmake -D CMAKE_BUILD_TYPE:String=Check -D ENABLE_TESTING=On .
CFLAGS='-Werror -Wall -Wextra -std=c99 -pedantic' make

msg "test: main suites (full config)" # ~ 5s
CFLAGS='-Werror -Wall -Wextra' make test

msg "test: ssl-opt.sh default (full config)" # ~ 1s
tests/ssl-opt.sh -f Default

msg "test: compat.sh RC4, DES & NULL (full config)" # ~ 2 min
OPENSSL_CMD="$OPENSSL_LEGACY" GNUTLS_CLI="$GNUTLS_LEGACY_CLI" GNUTLS_SERV="$GNUTLS_LEGACY_SERV" tests/compat.sh -e '3DES\|DES-CBC3' -f 'NULL\|DES\|RC4\|ARCFOUR'

msg "test/build: curves.pl (gcc)" # ~ 4 min
cleanup
cmake -D CMAKE_BUILD_TYPE:String=Debug .
tests/scripts/curves.pl

msg "test/build: key-exchanges (gcc)" # ~ 1 min
cleanup
cmake -D CMAKE_BUILD_TYPE:String=Check .
tests/scripts/key-exchanges.pl

msg "build: Unix make, -Os (gcc)" # ~ 30s
cleanup
CC=gcc CFLAGS='-Werror -Wall -Wextra -Os' make

# Full configuration build, without platform support, file IO and net sockets.
# This should catch missing mbedtls_printf definitions, and by disabling file
# IO, it should catch missing '#include <stdio.h>'
msg "build: full config except platform/fsio/net, make, gcc, C99" # ~ 30s
cleanup
cp "$CONFIG_H" "$CONFIG_BAK"
scripts/config.pl full
scripts/config.pl unset MBEDTLS_PLATFORM_C
scripts/config.pl unset MBEDTLS_NET_C
scripts/config.pl unset MBEDTLS_PLATFORM_MEMORY
scripts/config.pl unset MBEDTLS_PLATFORM_PRINTF_ALT
scripts/config.pl unset MBEDTLS_PLATFORM_FPRINTF_ALT
scripts/config.pl unset MBEDTLS_PLATFORM_SNPRINTF_ALT
scripts/config.pl unset MBEDTLS_PLATFORM_TIME_ALT
scripts/config.pl unset MBEDTLS_PLATFORM_EXIT_ALT
scripts/config.pl unset MBEDTLS_ENTROPY_NV_SEED
scripts/config.pl unset MBEDTLS_MEMORY_BUFFER_ALLOC_C
scripts/config.pl unset MBEDTLS_FS_IO
# Note, _DEFAULT_SOURCE needs to be defined for platforms using glibc version >2.19,
# to re-enable platform integration features otherwise disabled in C99 builds
CC=gcc CFLAGS='-Werror -Wall -Wextra -std=c99 -pedantic -O0 -D_DEFAULT_SOURCE' make lib programs
CC=gcc CFLAGS='-Werror -Wall -Wextra -O0' make test

# catch compile bugs in _uninit functions
msg "build: full config with NO_STD_FUNCTION, make, gcc" # ~ 30s
cleanup
cp "$CONFIG_H" "$CONFIG_BAK"
scripts/config.pl full
scripts/config.pl set MBEDTLS_PLATFORM_NO_STD_FUNCTIONS
scripts/config.pl unset MBEDTLS_ENTROPY_NV_SEED
CC=gcc CFLAGS='-Werror -Wall -Wextra -O0' make

msg "build: full config except ssl_srv.c, make, gcc" # ~ 30s
cleanup
cp "$CONFIG_H" "$CONFIG_BAK"
scripts/config.pl full
scripts/config.pl unset MBEDTLS_SSL_SRV_C
CC=gcc CFLAGS='-Werror -Wall -Wextra -O0' make

msg "build: full config except ssl_cli.c, make, gcc" # ~ 30s
cleanup
cp "$CONFIG_H" "$CONFIG_BAK"
scripts/config.pl full
scripts/config.pl unset MBEDTLS_SSL_CLI_C
CC=gcc CFLAGS='-Werror -Wall -Werror -O0' make

# Note, C99 compliance can also be tested with the sockets support disabled,
# as that requires a POSIX platform (which isn't the same as C99).
msg "build: full config except net_sockets.c, make, gcc -std=c99 -pedantic" # ~ 30s
cleanup
cp "$CONFIG_H" "$CONFIG_BAK"
scripts/config.pl full
scripts/config.pl unset MBEDTLS_NET_C # getaddrinfo() undeclared, etc.
scripts/config.pl set MBEDTLS_NO_PLATFORM_ENTROPY # uses syscall() on GNU/Linux
CC=gcc CFLAGS='-Werror -Wall -Wextra -O0 -std=c99 -pedantic' make lib

msg "build: default config with  MBEDTLS_TEST_NULL_ENTROPY (ASan build)"
cleanup
cp "$CONFIG_H" "$CONFIG_BAK"
scripts/config.pl set MBEDTLS_TEST_NULL_ENTROPY
scripts/config.pl set MBEDTLS_NO_DEFAULT_ENTROPY_SOURCES
scripts/config.pl set MBEDTLS_ENTROPY_C
scripts/config.pl unset MBEDTLS_ENTROPY_NV_SEED
scripts/config.pl unset MBEDTLS_ENTROPY_HARDWARE_ALT
scripts/config.pl unset MBEDTLS_HAVEGE_C
CC=gcc cmake  -D UNSAFE_BUILD=ON -D CMAKE_C_FLAGS:String="-fsanitize=address -fno-common -O3" .
make

msg "test: MBEDTLS_TEST_NULL_ENTROPY - main suites (inc. selftests) (ASan build)"
make test

if uname -a | grep -F Linux >/dev/null; then
msg "build/test: make shared" # ~ 40s
cleanup
make SHARED=1 all check
fi

if uname -a | grep -F x86_64 >/dev/null; then
msg "build: i386, make, gcc" # ~ 30s
cleanup
CC=gcc CFLAGS='-Werror -Wall -Wextra -m32' make

msg "build: gcc, force 32-bit compilation"
cleanup
cp "$CONFIG_H" "$CONFIG_BAK"
scripts/config.pl unset MBEDTLS_HAVE_ASM
scripts/config.pl unset MBEDTLS_AESNI_C
scripts/config.pl unset MBEDTLS_PADLOCK_C
CC=gcc CFLAGS='-Werror -Wall -Wextra -DMBEDTLS_HAVE_INT32' make

msg "build: gcc, force 64-bit compilation"
cleanup
cp "$CONFIG_H" "$CONFIG_BAK"
scripts/config.pl unset MBEDTLS_HAVE_ASM
scripts/config.pl unset MBEDTLS_AESNI_C
scripts/config.pl unset MBEDTLS_PADLOCK_C
CC=gcc CFLAGS='-Werror -Wall -Wextra -DMBEDTLS_HAVE_INT64' make

msg "test: gcc, force 64-bit compilation"
make test

msg "build: gcc, force 64-bit compilation"
cleanup
cp "$CONFIG_H" "$CONFIG_BAK"
scripts/config.pl unset MBEDTLS_HAVE_ASM
scripts/config.pl unset MBEDTLS_AESNI_C
scripts/config.pl unset MBEDTLS_PADLOCK_C
CC=gcc CFLAGS='-Werror -Wall -Wextra -DMBEDTLS_HAVE_INT64' make
fi # x86_64

msg "build: arm-none-eabi-gcc, make" # ~ 10s
cleanup
cp "$CONFIG_H" "$CONFIG_BAK"
scripts/config.pl full
scripts/config.pl unset MBEDTLS_NET_C
scripts/config.pl unset MBEDTLS_TIMING_C
scripts/config.pl unset MBEDTLS_FS_IO
scripts/config.pl unset MBEDTLS_ENTROPY_NV_SEED
scripts/config.pl set MBEDTLS_NO_PLATFORM_ENTROPY
# following things are not in the default config
scripts/config.pl unset MBEDTLS_HAVEGE_C # depends on timing.c
scripts/config.pl unset MBEDTLS_THREADING_PTHREAD
scripts/config.pl unset MBEDTLS_THREADING_C
scripts/config.pl unset MBEDTLS_MEMORY_BACKTRACE # execinfo.h
scripts/config.pl unset MBEDTLS_MEMORY_BUFFER_ALLOC_C # calls exit
CC=arm-none-eabi-gcc AR=arm-none-eabi-ar LD=arm-none-eabi-ld CFLAGS='-Werror -Wall -Wextra' make lib

msg "build: arm-none-eabi-gcc -DMBEDTLS_NO_UDBL_DIVISION, make" # ~ 10s
cleanup
cp "$CONFIG_H" "$CONFIG_BAK"
scripts/config.pl full
scripts/config.pl unset MBEDTLS_NET_C
scripts/config.pl unset MBEDTLS_TIMING_C
scripts/config.pl unset MBEDTLS_FS_IO
scripts/config.pl unset MBEDTLS_ENTROPY_NV_SEED
scripts/config.pl set MBEDTLS_NO_PLATFORM_ENTROPY
# following things are not in the default config
scripts/config.pl unset MBEDTLS_HAVEGE_C # depends on timing.c
scripts/config.pl unset MBEDTLS_THREADING_PTHREAD
scripts/config.pl unset MBEDTLS_THREADING_C
scripts/config.pl unset MBEDTLS_MEMORY_BACKTRACE # execinfo.h
scripts/config.pl unset MBEDTLS_MEMORY_BUFFER_ALLOC_C # calls exit
scripts/config.pl set MBEDTLS_NO_UDBL_DIVISION
CC=arm-none-eabi-gcc AR=arm-none-eabi-ar LD=arm-none-eabi-ld CFLAGS='-Werror -Wall -Wextra' make lib
echo "Checking that software 64-bit division is not required"
! grep __aeabi_uldiv library/*.o

msg "build: ARM Compiler 5, make"
cleanup
cp "$CONFIG_H" "$CONFIG_BAK"
scripts/config.pl full
scripts/config.pl unset MBEDTLS_NET_C
scripts/config.pl unset MBEDTLS_TIMING_C
scripts/config.pl unset MBEDTLS_FS_IO
scripts/config.pl unset MBEDTLS_ENTROPY_NV_SEED
scripts/config.pl unset MBEDTLS_HAVE_TIME
scripts/config.pl unset MBEDTLS_HAVE_TIME_DATE
scripts/config.pl set MBEDTLS_NO_PLATFORM_ENTROPY
# following things are not in the default config
scripts/config.pl unset MBEDTLS_DEPRECATED_WARNING
scripts/config.pl unset MBEDTLS_HAVEGE_C # depends on timing.c
scripts/config.pl unset MBEDTLS_THREADING_PTHREAD
scripts/config.pl unset MBEDTLS_THREADING_C
scripts/config.pl unset MBEDTLS_MEMORY_BACKTRACE # execinfo.h
scripts/config.pl unset MBEDTLS_MEMORY_BUFFER_ALLOC_C # calls exit
scripts/config.pl unset MBEDTLS_PLATFORM_TIME_ALT # depends on MBEDTLS_HAVE_TIME

CC="$ARMC5_CC" AR="$ARMC5_AR" WARNING_CFLAGS='--strict --c99' make lib
make clean

# ARM Compiler 6 - Target ARMv7-A
armc6_build_test "--target=arm-arm-none-eabi -march=armv7-a"

# ARM Compiler 6 - Target ARMv7-M
armc6_build_test "--target=arm-arm-none-eabi -march=armv7-m"

# ARM Compiler 6 - Target ARMv8-A - AArch32
armc6_build_test "--target=arm-arm-none-eabi -march=armv8.2-a"

# ARM Compiler 6 - Target ARMv8-M
armc6_build_test "--target=arm-arm-none-eabi -march=armv8-m.main"

# ARM Compiler 6 - Target ARMv8-A - AArch64
armc6_build_test "--target=aarch64-arm-none-eabi -march=armv8.2-a"

msg "build: allow SHA1 in certificates by default"
cleanup
cp "$CONFIG_H" "$CONFIG_BAK"
scripts/config.pl set MBEDTLS_TLS_DEFAULT_ALLOW_SHA1_IN_CERTIFICATES
CFLAGS='-Werror -Wall -Wextra' make
msg "test: allow SHA1 in certificates by default"
make test
tests/ssl-opt.sh -f SHA-1

msg "build: Windows cross build - mingw64, make (Link Library)" # ~ 30s
cleanup
CC=i686-w64-mingw32-gcc AR=i686-w64-mingw32-ar LD=i686-w64-minggw32-ld CFLAGS='-Werror -Wall -Wextra' WINDOWS_BUILD=1 make lib programs

# note Make tests only builds the tests, but doesn't run them
CC=i686-w64-mingw32-gcc AR=i686-w64-mingw32-ar LD=i686-w64-minggw32-ld CFLAGS='-Werror' WINDOWS_BUILD=1 make tests
WINDOWS_BUILD=1 make clean

msg "build: Windows cross build - mingw64, make (DLL)" # ~ 30s
CC=i686-w64-mingw32-gcc AR=i686-w64-mingw32-ar LD=i686-w64-minggw32-ld CFLAGS='-Werror -Wall -Wextra' WINDOWS_BUILD=1 SHARED=1 make lib programs
CC=i686-w64-mingw32-gcc AR=i686-w64-mingw32-ar LD=i686-w64-minggw32-ld CFLAGS='-Werror -Wall -Wextra' WINDOWS_BUILD=1 SHARED=1 make tests
WINDOWS_BUILD=1 make clean

# MemSan currently only available on Linux 64 bits
if uname -a | grep 'Linux.*x86_64' >/dev/null; then

msg "build: MSan (clang)" # ~ 1 min 20s
cleanup
cp "$CONFIG_H" "$CONFIG_BAK"
scripts/config.pl unset MBEDTLS_AESNI_C # memsan doesn't grok asm
CC=clang cmake -D CMAKE_BUILD_TYPE:String=MemSan .
make

msg "test: main suites (MSan)" # ~ 10s
make test

msg "test: ssl-opt.sh (MSan)" # ~ 1 min
tests/ssl-opt.sh

# Optional part(s)

if [ "$MEMORY" -gt 0 ]; then
    msg "test: compat.sh (MSan)" # ~ 6 min 20s
    tests/compat.sh
fi

else # no MemSan

msg "build: Release (clang)"
cleanup
CC=clang cmake -D CMAKE_BUILD_TYPE:String=Release .
make

msg "test: main suites valgrind (Release)"
make memcheck

# Optional part(s)
# Currently broken, programs don't seem to receive signals
# under valgrind on OS X

if [ "$MEMORY" -gt 0 ]; then
    msg "test: ssl-opt.sh --memcheck (Release)"
    tests/ssl-opt.sh --memcheck
fi

if [ "$MEMORY" -gt 1 ]; then
    msg "test: compat.sh --memcheck (Release)"
    tests/compat.sh --memcheck
fi

fi # MemSan

msg "build: cmake 'out-of-source' build"
cleanup
MBEDTLS_ROOT_DIR="$PWD"
mkdir "$OUT_OF_SOURCE_DIR"
cd "$OUT_OF_SOURCE_DIR"
cmake "$MBEDTLS_ROOT_DIR"
make

msg "test: cmake 'out-of-source' build"
make test
cd "$MBEDTLS_ROOT_DIR"
rm -rf "$OUT_OF_SOURCE_DIR"

msg "Done, cleaning up"
cleanup

