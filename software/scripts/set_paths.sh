#!/bin/bash

# load default path
source /etc/environment

# get current directory
SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# get arm compiler directory
#COMPILER_DIR=${SCRIPT_DIR}/../toolchain/gcc-arm-none-eabi/bin
TOOLCHAIN_PATH=~/gcc-arm-none-eabi-9-2020-q2-update/

JLINK_DIR=/opt/SEGGER/JLink/

# set new path
export PATH=${TOOLCHAIN_PATH}/bin:${JLINK_DIR}:${PATH}
echo Added ${TOOLCHAIN_PATH}/bin to PATH
echo Added ${JLINK_DIR} to PATH
