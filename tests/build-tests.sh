#!/bin/bash

set -e

which arm-none-eabi-g++
arm-none-eabi-g++ --version

TESTS="blink"
TESTS+=" bldc_turn_open_loop"
TESTS+=" bldc_block"
TESTS+=" bldc_foc"
TESTS+=" can"
TESTS+=" gatedriver"
TESTS+=" current_measurement"

for test in $TESTS
do
    pushd $test
    lbuild build
    scons -j8
    popd
done
