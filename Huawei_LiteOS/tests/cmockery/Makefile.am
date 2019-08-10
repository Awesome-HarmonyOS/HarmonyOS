## This is a boilerplate file for Google opensource projects.
## To make it useful, replace <<TEXT>> with actual text for your project.
## Also, look at comments with "## double hashes" to see if any are worth
## uncommenting or modifying.

## Process this file with automake to produce Makefile.in

# Make sure that when we re-make ./configure, we get the macros we need
ACLOCAL_AMFLAGS = -I `pwd`/m4

# This is so we can #include <google/foo>
AM_CPPFLAGS = -I$(top_srcdir)/src

# These are good warnings to turn on by default
if GCC
AM_CXXFLAGS = -Wall -Wwrite-strings -Woverloaded-virtual -Wno-sign-compare
endif

googleincludedir = $(includedir)/google
## The .h files you want to install (that is, .h files that people
## who install this package can include in their own applications.)
googleinclude_HEADERS = src/google/cmockery.h

docdir = $(prefix)/share/doc/$(PACKAGE)-$(VERSION)
## This is for HTML and other documentation you want to install.
## Add your documentation files (in doc/) in addition to these
## top-level boilerplate files.  Also add a TODO file if you have one.
dist_doc_DATA = AUTHORS COPYING ChangeLog INSTALL NEWS README doc/index.html \
		windows/makefile

## The libraries (.so's) you want to install
lib_LTLIBRARIES =

## unittests you want to run when people type 'make check'.
## TESTS is for binary unittests, check_SCRIPTS for script-based unittests.
## TESTS_ENVIRONMENT sets environment variables for when you run unittest,
## but it only seems to take effect for *binary* unittests (argh!)
TESTS =
TESTS_ENVIRONMENT = CMOCKERY_ROOTDIR = $(top_srcdir)
check_SCRIPTS =
# Every time you add a unittest to check_SCRIPTS, add it here too
noinst_SCRIPTS =


## vvvv RULES TO MAKE THE LIBRARIES, BINARIES, AND UNITTESTS

lib_LTLIBRARIES += libcmockery.la
libcmockery_la_SOURCES = src/config.h src/cmockery.c src/google/cmockery.h
libcmockery_la_CFLAGS = -I$(top_srcdir)/src/google

noinst_PROGRAMS = calculator
calculator_SOURCES = src/example/calculator.c src/config.h
calculator_CFLAGS =

unit_test_CFLAGS = -I$(top_srcdir)/src/google \
                   -I$(top_srcdir)/src/example -DUNIT_TESTING=1

noinst_PROGRAMS += calculator_test
calculator_test_SOURCES = src/example/calculator_test.c \
                          $(calculator_SOURCES)
calculator_test_CFLAGS = $(unit_test_CFLAGS)
calculator_test_LDADD = libcmockery.la

noinst_PROGRAMS += allocate_module_test
allocate_module_test_SOURCES = src/example/allocate_module_test.c \
                               src/example/allocate_module.c
allocate_module_test_CFLAGS = $(unit_test_CFLAGS)
allocate_module_test_LDADD = libcmockery.la

noinst_PROGRAMS += assert_macro_test
assert_macro_test_SOURCES = src/example/assert_macro_test.c \
                            src/example/assert_macro.c
assert_macro_test_CFLAGS = $(unit_test_CFLAGS)
assert_macro_test_LDADD = libcmockery.la

noinst_PROGRAMS += customer_database_test
customer_database_test_SOURCES = src/example/customer_database_test.c \
                                 src/example/customer_database.c \
                                 src/example/database.h
customer_database_test_CFLAGS = $(unit_test_CFLAGS)
customer_database_test_LDADD = libcmockery.la

noinst_PROGRAMS += key_value_test
key_value_test_SOURCES = src/example/key_value_test.c \
                         src/example/key_value.c
key_value_test_CFLAGS = $(unit_test_CFLAGS)
key_value_test_LDADD = libcmockery.la

noinst_PROGRAMS += product_database_test
product_database_testdir = src/example
product_database_test_SOURCES = src/example/product_database_test.c \
                                src/example/product_database.c \
                                src/example/database.h
product_database_test_CFLAGS = $(unit_test_CFLAGS)
product_database_test_LDADD = libcmockery.la

noinst_PROGRAMS += run_tests
run_tests_SOURCES = src/example/run_tests.c
run_tests_CFLAGS = $(unit_test_CFLAGS)
run_tests_LDADD = libcmockery.la

## ^^^^ END OF RULES TO MAKE THE LIBRARIES, BINARIES, AND UNITTESTS


## This should always include $(TESTS), but may also include other
## binaries that you compile but don't want automatically installed.
noinst_PROGRAMS += $(TESTS)

rpm: dist-gzip packages/rpm.sh packages/rpm/rpm.spec
	@cd packages && ./rpm.sh ${PACKAGE} ${VERSION}

deb: dist-gzip packages/deb.sh packages/deb/*
	@cd packages && ./deb.sh ${PACKAGE} ${VERSION}

## If you're using libtool, add 'libtool' here.  Also add this rule:
libtool: $(LIBTOOL_DEPS)
	$(SHELL) ./config.status --recheck
EXTRA_DIST = packages/rpm.sh packages/rpm/rpm.spec packages/deb.sh packages/deb \
	libtool $(SCRIPTS)
