# Fuzzing SPIFFS

The SPIFFS test suite includes a test program designed for fuzzing with
[AFL](http://lcamtuf.coredump.cx/afl/). This automatically exercises the 
SPIFFS API and verifies that the file system does not crash or interact incorrectly
with the flash chip. 

There are two steps to fuzzing. The first is to build the test suite with
the AFL version of gcc. The CC variable should point to your copy of afl-gcc.

```
make clean test CC=/usr/local/bin/afl-gcc
```

There is a new test `afl_test` that reads from stdin a list of commands
and arguments. These are interpreted and executed on the API. The `afltests`
directory contains a number of test cases that can be fed to the `afl_test` test.


The second is to run this test suite under afl as follows (where findings is 
the output directory):

```
afl-fuzz -i afltests -o findings ./build/linux_spiffs_test -f afl_test
```

This run will take hours (or days) and will (hopefully) not find any crashes.
If a crash (or hang) is found, then the input file that caused the crash is 
saved. This allows the specific test case to be debugged.

## Reducing the size of the file

AFL comes with `afl-tmin` which can reduce the size of the test input file to
make it easier to debug.

```
afl-tmin -i findings/crashes/<somefile> -o smalltest -- build/linux_spiffs_test -f afl_test
```

This will write a short version of the testcase file to `smalltest`. This can then be
fed into the test program for debugging:

```
build/linux_spiffs_test -f afl_test < smalltest
```

This should still crash, but allows it to be run under a debugger. 
