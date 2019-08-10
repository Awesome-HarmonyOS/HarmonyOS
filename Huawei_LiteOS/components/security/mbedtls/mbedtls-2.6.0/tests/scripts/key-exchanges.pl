#!/usr/bin/perl

# test that all configs with only a single key exchange enabled build
#
# Usage: tests/scripts/key-exchanges.pl

use warnings;
use strict;

-d 'library' && -d 'include' && -d 'tests' or die "Must be run from root\n";

my $sed_cmd = 's/^#define \(MBEDTLS_KEY_EXCHANGE_.*_ENABLED\)/\1/p';
my $config_h = 'include/mbedtls/config.h';
my @kexes = split( /\s+/, `sed -n -e '$sed_cmd' $config_h` );

system( "cp $config_h $config_h.bak" ) and die;
sub abort {
    system( "mv $config_h.bak $config_h" ) and warn "$config_h not restored\n";
    die $_[0];
}

for my $kex (@kexes) {
    system( "cp $config_h.bak $config_h" ) and die "$config_h not restored\n";
    system( "make clean" ) and die;

    print "\n******************************************\n";
    print "* Testing with key exchange: $kex\n";
    print "******************************************\n";

    # full config with all key exchanges disabled except one
    system( "scripts/config.pl full" ) and abort "Failed config full\n";
    for my $k (@kexes) {
        next if $k eq $kex;
        system( "scripts/config.pl unset $k" )
            and abort "Failed to disable $k\n";
    }

    system( "make lib CFLAGS='-Os -Werror'" ) and abort "Failed to build lib: $kex\n";
}

system( "mv $config_h.bak $config_h" ) and die "$config_h not restored\n";
system( "make clean" ) and die;
exit 0;
