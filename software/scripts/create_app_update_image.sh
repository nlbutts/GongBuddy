#!/bin/bash

# --------------------------------- Get the directory that this script lives in
SCRIPT_WORKING_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
PROJECT_DIR=${SCRIPT_WORKING_DIR}/..


# ------------------------------------------------------- Find the target files
ARM_BUILD_DIR=${PROJECT_DIR}/build/arm
X86_BUILD_DIR=${PROJECT_DIR}/build/x86

INPUT_FILE=${ARM_BUILD_DIR}/fwimages/application_crc.bin
IMAGEGEN_TOOL=${X86_BUILD_DIR}/imagegen/imagegen

DEFAULT_OUTPUT_FILE=${ARM_BUILD_DIR}/fwimages/application_crc.image


if [[ ! -e ${ARM_BUILD_DIR} ]]; then
    cd ${PROJECT_DIR}
    ./generate_build.sh
fi


if [[ ! -e ${INPUT_FILE} ]]; then
    cd ${ARM_BUILD_DIR}
    make -j4
fi

if [[ ! -e ${INPUT_FILE} ]]; then
    printf "\n[ERROR - ${LINENO}] Artifact ${INPUT_FILE} does not exist\n"
    exit 1
fi


if [[ ! -e ${IMAGEGEN_TOOL} ]]; then
    cd ${X86_BUILD_DIR}
    make -j4
fi

if [[ ! -e ${IMAGEGEN_TOOL} ]]; then
    printf "\n[ERROR - ${LINENO}] Build tool ${IMAGEGEN_TOOL} does not exist\n"
    exit 1
fi


# ------------------------------------------------- Detect the output file name
if [[ -z "${1// }" ]]; then
    printf "\nUsing default output file"
    output_file=${DEFAULT_OUTPUT_FILE}
else
    printf "\nUsing ${1} as the output file"
    output_file=${1}
fi

if [[ -z "${2// }" ]]; then
    printf "\nMissing hardware compatibility flag, allowing imagegen to default\n"
    hw_comp=""
else
    printf "\nUsing ${2} as the hardware compatibility flag\n"
    hw_comp="-w ${2}"
fi

# ----------------------------------------------------- Create the update image
${IMAGEGEN_TOOL} -c -a ${INPUT_FILE} ${output_file} ${hw_comp} || exit 1
