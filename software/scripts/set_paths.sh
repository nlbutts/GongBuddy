#!/bin/bash

# load default path
source /etc/environment

# get current directory
SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# get arm compiler directory
#COMPILER_DIR=${SCRIPT_DIR}/../toolchain/gcc-arm-none-eabi/bin
COMPILER_DIR=${TOOLCHAIN_PATH}/bin

JLINK_DIR=/opt/SEGGER/JLink/

# set new path
export PATH=${COMPILER_DIR}:${JLINK_DIR}:${PATH}
echo Added ${COMPILER_DIR} to PATH
echo Added ${JLINK_DIR} to PATH
