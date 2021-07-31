#!/bin/bash
# This script generates and builds everything for the sensortile build
# If you want the script to build the projects, pass in "make" as a commandline option
#
# Examples:
# 1) generates build code using cmake: ./generate_build
# 2) generates and makes projects:     ./generate_build make
# 3) generates debug build:            ./generate_build debug

main()
{
    if [[ -h gcc-arm-none-eabi ]]; then
        echo "Symlink to gcc exists, using that as the toolchain"
        export TOOLCHAIN_PATH=gcc-arm-none-eabi
    else
        echo "Add a symlink called gcc-arm-none-eabi that points to the toolchain"
    fi
    # Get the directory that this script lives in
    THIS_SCRIPT_PATH=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

    # Set the build output path
    if [ -z "${BUILD_ROOT_PATH}" ]; then
        BUILD_ROOT_PATH=${THIS_SCRIPT_PATH}/build
    fi

    prepare_environment

    # Remove all previous builds
    mkdir -p ${BUILD_ROOT_PATH}
    rm -rf ${BUILD_ROOT_PATH}/*

    # Create builds
    create_build    arm         .           "${THIS_SCRIPT_PATH}/toolchain/arm_embedded_toolchain.txt" $1 || exit 1
    create_build    x86         .                                                           || exit 1

    # Build created projects, if option is set
    if [ "$1" == "make" ]; then
        make_all_builds
    fi

}

# $1 File to get the absolute path of
function realpath()
{
    echo $(cd $(dirname $1); pwd)/$(basename $1);
}

# Function - Creates the Cmake files for the build and makes
# $1 - Build platform name
# $2 - Optional toolchain file
function create_build()
{
    build_path="${BUILD_ROOT_PATH}/$1"
    cmake_args="-DENABLE_DEBUG_PRINT=1"

    if [ -z "$1" ]; then
        printf "\n[ERROR - ${LINENO}] Bad arguments\n"
        #exit 1
    fi

    if [ -z "$2" ]; then
        printf "\n[ERROR - ${LINENO}] Bad arguments\n"
        #exit 1
    fi

    if [ "$4" == "debug" ]; then
        printf "DEBUG build\n"
        cmake_args="-DCMAKE_BUILD_TYPE=Debug"
        #exit 1
    fi

    # if [ ! -z "$4" ]; then
    #     cmake_args=${4}
    # fi

    # Clear and create the build path
    printf "Clearing ${build_path}\n"
    mkdir -p ${build_path}
    rm -rf ${build_path}/*

    # Optionally select the toolchain file
    if [ ! -z "${3}" ]; then
        toolchain_file=$( realpath "${3}" )
        cmake_args="-DCMAKE_TOOLCHAIN_FILE:PATH=${toolchain_file} ${cmake_args}"
    fi

    # Call CMake to generate the build
    (
        set -x
        cd ${build_path} && cmake ${cmake_args} ${THIS_SCRIPT_PATH}/${2}
    )

    # Make sure cmake ran without issue
    if ! [ "$?" = "0" ]; then
        printf "\n[ERROR - ${LINENO}] Build step failed\n"
        exit 1
    fi
}

#This function prepares the environment for CMake
function prepare_environment()
{
    cd third-party
    if [[ ! -e /opt/SEGGER/JLink ]]; then
        echo "JLink is not installed on this computer. Please install via:"
        echo "sudo dpkg -i third-party/JLink_Linux_<whatever is in that directory>"
        exit 1
    fi

    RTT_VER=`ls /opt/SEGGER/JLink/Samples/RTT/*RTT*`
    echo "Found RTT file: ${RTT_VER}"

    tar -xf ${RTT_VER}
}

#Builds all directories in "BUILD_ROOT_PATH" when called.
function make_all_builds()
{
    for dir in ${BUILD_ROOT_PATH}/*
    {
        echo "---------- Building in: ${dir} ----------"
        (
        set -x
        cd "${dir}" && make -j4 install VERBOSE=1 || exit 1
        )
    }
}

# calls main function - basically, just keeps the main up top for readability.
main "$@"
