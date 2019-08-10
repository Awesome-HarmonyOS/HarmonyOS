#!/usr/bin/perl

# curves.pl
#
# Copyright (c) 2014-2016, ARM Limited, All Rights Reserved
#
# Purpose
#
# To test the code dependencies on individual curves in each test suite. This
# is a verification step to ensure we don't ship test suites that do not work
# for some build options.
#
# The process is:
#       for each possible curve
#           build the library and test suites with the curve disabled
#           execute the test suites
#
# And any test suite with the wrong dependencies will fail.
#
# Usage: curves.pl
#
# This script should be executed from the root of the project directory.

use warnings;
use strict;

-d 'library' && -d 'include' && -d 'tests' or die "Must be run from root\n";

my $sed_cmd = 's/^#define \(MBEDTLS_ECP_DP.*_ENABLED\)/\1/p';
my $config_h = 'include/mbedtls/config.h';
my @curves = split( /\s+/, `sed -n -e '$sed_cmd' $config_h` );

system( "cp $config_h $config_h.bak" ) and die;
sub abort {
    system( "mv $config_h.bak $config_h" ) and warn "$config_h not restored\n";
    die $_[0];
}

for my $curve (@curves) {
    system( "cp $config_h.bak $config_h" ) and die "$config_h not restored\n";
    # depends on a specific curve. Also, ignore error if it wasn't enabled
    system( "scripts/config.pl unset MBEDTLS_KEY_EXCHANGE_ECJPAKE_ENABLED" );
    system( "make clean" ) and die;

    print "\n******************************************\n";
    print "* Testing without curve: $curve\n";
    print "******************************************\n";

    system( "scripts/config.pl unset $curve" )
        and abort "Failed to disable $curve\n";

    system( "CFLAGS='-Werror -Wall -Wextra' make lib" )
        and abort "Failed to build lib: $curve\n";
    system( "cd tests && make" ) and abort "Failed to build tests: $curve\n";
    system( "make test" ) and abort "Failed test suite: $curve\n";

}

system( "mv $config_h.bak $config_h" ) and die "$config_h not restored\n";
system( "make clean" ) and die;
exit 0;
