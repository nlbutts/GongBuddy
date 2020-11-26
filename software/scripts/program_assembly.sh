#!/bin/bash

# --------------------------------- Get the directory that this script lives in
SCRIPT_WORKING_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
PROJECT_DIR=${SCRIPT_WORKING_DIR}/..

BUILD_DIR=${PROJECT_DIR}/build/arm
ASSEMBLY_FILE=${BUILD_DIR}/fwimages/assembly.hex


# ------------------------------------------------ Add the J-Link tools to path
source ${SCRIPT_WORKING_DIR}/set_paths.sh


# ------------------ Check if the assembly file exists, build it if it does not
if [[ ! -e ${BUILD_DIR} ]]; then
    cd ${PROJECT_DIR}
    ./generate_build.sh
fi

if [[ ! -e ${ASSEMBLY_FILE} ]]; then
    cd ${BUILD_DIR}
    make -j4
fi

if [[ ! -e ${ASSEMBLY_FILE} ]]; then
    printf "\n[ERROR - ${LINENO}] Artifact ${ASSEMBLY_FILE} does not exist\n"
    exit 1
fi


# ------------------------------------------------------ Create the jflash file
jflash_file=`mktemp -u`

function cleanup {
    rm -f ${jflash_file}
}

trap cleanup EXIT


printf "loadfile ${PROJECT_DIR}/build/arm/fwimages/assembly.bin\nr\nq\n" \
    >> ${jflash_file}


# ----------------------------------------------------- Start JLink application
JLinkExe \
    -device STM32L476RG \
    -if SWD -speed 4000 \
    -CommanderScript ${jflash_file}
