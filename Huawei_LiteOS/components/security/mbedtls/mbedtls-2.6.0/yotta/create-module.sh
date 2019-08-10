#!/bin/sh

set -eu

# relative to the script's directory
TREE=..
DEST=module

# make sure we're running in our own directory
if [ -f create-module.sh ]; then :; else
    cd $( dirname $0 )
    if [ -f create-module.sh ]; then :; else
        echo "Please run the script from is directory." >&2
        exit 1
    fi
fi

# use a temporary directory to build the module, then rsync to DEST
# this allows touching only new files, for more efficient re-builds
TMP=$DEST-tmp
rm -rf $TMP

mkdir -p $TMP/mbedtls $TMP/source
cp $TREE/include/mbedtls/*.h $TMP/mbedtls
cp $TREE/library/*.c $TMP/source

# temporary, should depend on external module later
cp data/entropy_hardware_poll.c $TMP/source
cp data/target_config.h $TMP/mbedtls

data/adjust-config.sh $TREE/scripts/config.pl $TMP/mbedtls/config.h

mkdir -p $TMP/test
cp -r data/example-* $TMP/test
# later we should have the generated test suites here too

cp data/module.json $TMP
cp data/README.md $TMP

cp ../LICENSE $TMP
if [ -f ../apache-2.0.txt ]; then cp ../apache-2.0.txt $TMP; fi

mkdir -p $DEST
rsync -cr --delete --exclude build --exclude yotta_\* $TMP/ $DEST/
rm -rf $TMP

echo "mbed TLS yotta module created in '$PWD/$DEST'."
