#!/bin/sh

# check if generated files are up-to-date

set -eu

if [ -d library -a -d include -a -d tests ]; then :; else
    echo "Must be run from mbed TLS root" >&2
    exit 1
fi

check()
{
    FILE=$1
    SCRIPT=$2

    cp $FILE $FILE.bak
    $SCRIPT
    diff $FILE $FILE.bak
    mv $FILE.bak $FILE
}

check library/error.c scripts/generate_errors.pl
check library/version_features.c scripts/generate_features.pl
