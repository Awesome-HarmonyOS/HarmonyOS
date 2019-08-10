#!/usr/bin/perl

# run-test-suites.pl
#
# This file is part of mbed TLS (https://tls.mbed.org)
#
# Copyright (c) 2015-2016, ARM Limited, All Rights Reserved
#
# Purpose
#
# Executes all the available test suites, and provides a basic summary of the
# results.
#
# Usage: run-test-suites.pl [-v]
#
# Options :
#   -v|--verbose    - Provide a pass/fail/skip breakdown per test suite and
#                     in total
#

use warnings;
use strict;

use utf8;
use open qw(:std utf8);

use constant FALSE => 0;
use constant TRUE => 1;

my $verbose;
my $switch = shift;
if ( defined($switch) && ( $switch eq "-v" || $switch eq "--verbose" ) ) {
    $verbose = TRUE;
}

# All test suites = executable files, excluding source files, debug
# and profiling information, etc. We can't just grep {! /\./} because
#some of our test cases' base names contain a dot.
my @suites = grep { -x $_ || /\.exe$/ } glob 'test_suite_*';
die "$0: no test suite found\n" unless @suites;

# in case test suites are linked dynamically
$ENV{'LD_LIBRARY_PATH'} = '../library';

my $prefix = $^O eq "MSWin32" ? '' : './';

my ($failed_suites, $total_tests_run, $failed, $suite_cases_passed,
    $suite_cases_failed, $suite_cases_skipped, $total_cases_passed,
    $total_cases_failed, $total_cases_skipped );

for my $suite (@suites)
{
    print "$suite ", "." x ( 72 - length($suite) - 2 - 4 ), " ";
    my $result = `$prefix$suite`;

    $suite_cases_passed = () = $result =~ /.. PASS/g;
    $suite_cases_failed = () = $result =~ /.. FAILED/g;
    $suite_cases_skipped = () = $result =~ /.. ----/g;

    if( $result =~ /PASSED/ ) {
        print "PASS\n";
    } else {
        $failed_suites++;
        print "FAIL\n";
    }

    my ($passed, $tests, $skipped) = $result =~ /([0-9]*) \/ ([0-9]*) tests.*?([0-9]*) skipped/;
    $total_tests_run += $tests - $skipped;

    if ( $verbose ) {
        print "(test cases passed:", $suite_cases_passed,
                " failed:", $suite_cases_failed,
                " skipped:", $suite_cases_skipped,
                " of total:", ($suite_cases_passed + $suite_cases_failed +
                               $suite_cases_skipped),
                ")\n"
    }

    $total_cases_passed += $suite_cases_passed;
    $total_cases_failed += $suite_cases_failed;
    $total_cases_skipped += $suite_cases_skipped;
}

print "-" x 72, "\n";
print $failed_suites ? "FAILED" : "PASSED";
printf " (%d suites, %d tests run)\n", scalar @suites, $total_tests_run;

if ( $verbose ) {
    print "  test cases passed :", $total_cases_passed, "\n";
    print "             failed :", $total_cases_failed, "\n";
    print "            skipped :", $total_cases_skipped, "\n";
    print "  of tests executed :", ( $total_cases_passed + $total_cases_failed ),
            "\n";
    print " of available tests :",
            ( $total_cases_passed + $total_cases_failed + $total_cases_skipped ),
            "\n"
    }

exit( $failed_suites ? 1 : 0 );

