#!/bin/sh


# this file is used to place the compiled binaries in the correct location
# arg1: the path to the compiled binaries   e.g. a/b/c/armv7a/ft8_decoder
# arg2: the path to the ft8 module directory  e.g. x/y/SDRPlusPlus/decoder_modules/ft8_decoder
# the android bin path will be calculated.
(
set -e
set -x
BINNAME=`basename $1`
DIRNAME=`dirname $1`
ARCH=`basename $DIRNAME`
APP=$2/../../android/app
mkdir -p $APP/bin/$ARCH
cp $1 $APP/bin/$ARCH/$BINNAME.so
) >> /tmp/logg2
