#!/bin/bash

# --------------------------------- Get the directory that this script lives in
SCRIPT_WORKING_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
PROJECT_DIR=${SCRIPT_WORKING_DIR}/..


# ------------------------------------------------ Add the J-Link tools to path
source ${SCRIPT_WORKING_DIR}/set_paths.sh


# -------------------------------------------------------- Find the target file
BUILD_DIR=${PROJECT_DIR}/build/arm
TARGET_FILE=${BUILD_DIR}/application/application.elf

if [[ ! -e ${BUILD_DIR} ]]; then
    cd ${PROJECT_DIR}
    ./generate_build.sh
fi

if [[ ! -e ${TARGET_FILE} ]]; then
    cd ${BUILD_DIR}
    make -j4
fi

if [[ ! -e ${TARGET_FILE} ]]; then
    printf "\n[ERROR - ${LINENO}] Artifact ${TARGET_FILE} does not exist\n"
    exit 1
fi


# -------------------------------------------------------- Execute the debugger
arm-none-eabi-gdb -tui \
    --command=${SCRIPT_WORKING_DIR}/gdbinit_local \
    ${TARGET_FILE}
