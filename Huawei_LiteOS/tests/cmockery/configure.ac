## This is a boilerplate file for Google opensource projects.
## To make it useful, replace <<TEXT>> with actual text for your project.
## Also, look at comments with "## double hashes" to see if any are worth
## uncommenting or modifying.

## Process this file with autoconf to produce configure.
## In general, the safest way to proceed is to run ./autogen.sh

# make sure we're interpreted by some minimal autoconf
AC_PREREQ(2.57)

AC_INIT(cmockery, 0.1.2, opensource@google.com)
# The argument here is just something that should be in the current directory
# (for sanity checking)
AC_CONFIG_SRCDIR(README)
AM_INIT_AUTOMAKE
AM_CONFIG_HEADER(src/config.h)

# Checks for programs.
AC_PROG_CC
AC_PROG_CPP
AC_PROG_CXX
AM_CONDITIONAL(GCC, test "$GCC" = yes)   # let the Makefile know if we're gcc

# Uncomment this if you'll be exporting libraries (.so's)
AC_PROG_LIBTOOL
AC_SUBST(LIBTOOL_DEPS)

# Check whether some low-level functions/files are available
AC_HEADER_STDC

# Here are some examples of how to check for the existence of a fn or file
##AC_CHECK_FUNCS(memmove)
##AC_CHECK_HEADERS(sys/resource.h)
AC_CHECK_FUNCS(setjmp)
AC_CHECK_FUNCS(longjmp)
AC_CHECK_FUNCS(strcmp)
AC_CHECK_FUNCS(strcpy)
AC_CHECK_FUNCS(memcpy)
AC_CHECK_FUNCS(memset)
AC_CHECK_FUNCS(malloc)
AC_CHECK_FUNCS(calloc)
AC_CHECK_FUNCS(free)
AC_CHECK_FUNCS(exit)
AC_CHECK_FUNCS(signal)
AC_CHECK_FUNCS(printf)
AC_CHECK_FUNCS(fprintf)
AC_CHECK_FUNCS(snprintf)
AC_CHECK_FUNCS(vsnprintf)
AC_CHECK_HEADERS(assert.h)
AC_CHECK_HEADERS(malloc.h)
AC_CHECK_HEADERS(setjmp.h)
AC_CHECK_HEADERS(stdarg.h)
AC_CHECK_HEADERS(stddef.h)
AC_CHECK_HEADERS(stdio.h)
AC_CHECK_HEADERS(stdlib.h)
AC_CHECK_HEADERS(string.h)
AC_CHECK_HEADERS(signal.h)

# Find out what namespace 'normal' STL code lives in, and also what namespace
# the user wants our classes to be defined in
AC_CXX_STL_NAMESPACE
AC_DEFINE_GOOGLE_NAMESPACE(google)

## Check out ../autoconf/ for other macros you can call to do useful stuff

# Write generated configuration file
AC_CONFIG_FILES([Makefile])
AC_OUTPUT
