# ------------------------------------------------------- MINIMUM CMAKE VERSION
cmake_minimum_required(VERSION 3.7)


# ---------------------------------------------------------------- PROJECT NAME
project(gongbuddy LANGUAGES C CXX ASM)

Message(STATUS "CMAKE_SYSTEM_PROCESSOR: " ${CMAKE_SYSTEM_PROCESSOR})
Message(STATUS "CMAKE_SYSTEM_NAME: " ${CMAKE_SYSTEM_NAME})


# --------------------------------------------------------------------- OPTIONS
option(BUILD_UT "Builds the unit tests" ON)
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES arm_baremetal)
    set(BUILD_UT OFF)
endif()

set(HW_COMPAT_BITFIELD 1
    CACHE STRING "Hardware compatibility bitfield"
)
message(STATUS "Hardware compatibility bitfield: ${HW_COMPAT_BITFIELD}")


# ------------------------------------------------- PROJECT INSTALL AND STAGING
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/build_output)


# ------------------------------------------------------ C/C++ LANGUAGE VERSION
include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++1y" COMPILER_SUPPORTS_CXX1Y)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
    message(STATUS "Enabled C++14 support")
elseif(COMPILER_SUPPORTS_CXX1Y)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y")
    message(STATUS "Enabled C++1y support")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} does not have C++14 support!")
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")
message(STATUS "Enabled C99 support")


# ---------------------------------------------------------- OPTIMIZATION FLAGS
# Do NOT set CMAKE_BUILD_TYPE, otherwise CMake will add in its own optimization
# flags, which will default to -O3 optimization
if(${CMAKE_BUILD_TYPE} MATCHES Debug)
    add_compile_options(-O0)
    message(STATUS "Building debug version")
else()
    add_compile_options(-Os)
    message(STATUS "Building release version")
endif()

# Debug symbols only appear in the elf file, not in the bin or hex files
add_compile_options(-g3 -ggdb)

# Separate data / function sections, garbage collect unused ones during linking
add_compile_options(-ffunction-sections -fdata-sections)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Xlinker --gc-sections")

# ----------------------------------------------- ARCHITECTURE SPECIFIC OPTIONS
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES arm_baremetal)
    # Processor specific flags
    set(CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 --specs=nano.specs")

    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CPU_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CPU_FLAGS}")

    add_definitions(-D__FPU_PRESENT -DARM_MATH_CM4 -DUSE_HAL_DRIVER
                    -DSTM32L476xx)

    # Don't use exceptions or RTTI on the target
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti -fno-exceptions")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nosys.specs")
    else()
    # Test coverage flags
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftest-coverage -fprofile-arcs")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lgcov")
endif()


# ---------------------------------------------------- WARNINGS AND SUPRESSIONS
# Enable warnings
add_compile_options(-Wall)

# There is legacy code in libAppareo that will not compile with newer
# compilers without this.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-strict-aliasing")


# ------------------------------------------------------ VERSIONING INFORMATION


# ---------------------------------------------------------- LOCATE EXTRA TOOLS
find_program(SREC_CAT srec_cat NO_CMAKE_FIND_ROOT_PATH)
if(NOT "${SREC_CAT}" STREQUAL "SREC_CAT-NOTFOUND")
    message(STATUS "Found srec_cat: ${SREC_CAT}")
    execute_process(
        COMMAND ${SREC_CAT} --version
        OUTPUT_VARIABLE SREC_CAT_OUTPUT
    )

    string(REGEX MATCH "([0-9]+(\\.[^ \n\r]+)+)" SREC_CAT_VERSION "${SREC_CAT_OUTPUT}")
    message(STATUS "srec_cat version: ${SREC_CAT_VERSION}")

    if(SREC_CAT_VERSION VERSION_LESS "1.62")
        message(FATAL_ERROR "srec_cat is too old. Please install 1.62.D001")
    endif()
else()
    message(FATAL_ERROR "Could not locate srec_cat. You need to install srecord 1.62.D001")
endif()


# ------------------------------------------------------------ INCLUDE PROJECTS
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES arm_baremetal)
    # --------------------------------------------------STRATUS DEBUG LOG LEVEL
    # build for arm only

    # Libraries
    #add_subdirectory(drivers)
    #add_subdirectory(middleware)
    add_subdirectory(messages)
    add_subdirectory(libupdate)
    add_subdirectory(libbuildinfo)

    # Executables
    add_subdirectory(application)
    add_subdirectory(fwimages)

else ()
    # --------------------------------------------------STRATUS DEBUG LOG LEVEL
    # build for x86 only
    # TODO: Do not pass this definition globally, make it part of the libAppareo build
    add_definitions(-DSYS_LOG_LEVEL=LOG_NONE)

    # Libraries
    add_subdirectory(libupdate)
    add_subdirectory(libbuildinfo)
    add_subdirectory(third-party/tclap-1.2.1)
    add_subdirectory(imagegen)

    # Executables
endif ()
