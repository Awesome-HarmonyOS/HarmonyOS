#!/usr/bin/perl

# Check for malloc calls not shortly followed by initialisation.
#
# Known limitations:
# - false negative: can't see allocations spanning more than one line
# - possible false negatives, see patterns
# - false positive: malloc-malloc-init-init is not accepted
# - false positives: "non-standard" init functions (eg, the things being
# initialised is not the first arg, or initialise struct members)
#
# Since false positives are expected, the results must be manually reviewed.
#
# Typical usage: scripts/malloc-init.pl library/*.c

use warnings;
use strict;

use utf8;
use open qw(:std utf8);

my $limit = 7;
my $inits = qr/memset|memcpy|_init|fread|base64_..code/;

# cases to bear in mind:
#
# 0. foo = malloc(...); memset( foo, ... );
# 1. *foo = malloc(...); memset( *foo, ... );
# 2. type *foo = malloc(...); memset( foo, ...);
# 3. foo = malloc(...); foo_init( (type *) foo );
# 4. foo = malloc(...); for(i=0..n) { init( &foo[i] ); }
#
# The chosen patterns are a bit relaxed, but unlikely to cause false positives
# in real code (initialising *foo or &foo instead of foo will likely be caught
# by functional tests).
#
my $id = qr/([a-zA-Z-0-9_\->\.]*)/;
my $prefix = qr/\s(?:\*?|\&?|\([a-z_]* \*\))\s*/;

my $name;
my $line;
my @bad;

die "Usage: $0 file.c [...]\n" unless @ARGV;

while (my $file = shift @ARGV)
{
    open my $fh, "<", $file or die "read $file failed: $!\n";
    while (<$fh>)
    {
        if( /mbedtls_malloc\(/ ) {
            if( /$id\s*=.*mbedtls_malloc\(/ ) {
                push @bad, "$file:$line:$name" if $name;
                $name = $1;
                $line = $.;
            } else {
                push @bad, "$file:$.:???" unless /return mbedtls_malloc/;
            }
        } elsif( $name && /(?:$inits)\($prefix\Q$name\E\b/ ) {
            undef $name;
        } elsif( $name && $. - $line > $limit ) {
            push @bad, "$file:$line:$name";
            undef $name;
            undef $line;
        }
    }
    close $fh or die;
}

print "$_\n" for @bad;
