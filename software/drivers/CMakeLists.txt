# Top level CMake for building the libj1939 library and unit tests
# ----------------------------------------------- Set the minimum CMake version
cmake_minimum_required(VERSION 3.7)


# -------------------------------------------------------- Set the project type
set(THIS drivers)

# ------------------------------------------------------------ Aliases
set(SENSOR_TILE_DIR ${CMAKE_SOURCE_DIR}/STSW-STLKT01_V2.4.0)
set(DRIVER_DIR      ${SENSOR_TILE_DIR}/Drivers)

# ------------------------------------------------------------ Get files
file(GLOB_RECURSE SRC_FILES
    ${DRIVER_DIR}/*.c
)

# foreach(item ${SRC_FILES})
#     if(${item} MATCHES ".*template.c")
#             message(STATUS "excluding for STM32F405: ${item}")
#             list(REMOVE_ITEM SRC_FILES ${item})
#     endif()
#     # Because ST is stupid and the code doesn't compile with this file.
#     if(${item} MATCHES "SensorTile_audio_out.c")
#             message(STATUS "excluding for STM32F405: ${item}")
#             list(REMOVE_ITEM SRC_FILES ${item})
#     endif()
# endforeach()

#----------------------------------------------------- Define the build target
add_library(${THIS} STATIC ${SRC_FILES})

# ----------------------------------------------------------- Set include paths
target_include_directories(${THIS}
    PUBLIC ${DRIVER_DIR}/CMSIS/Include
    PUBLIC ${DRIVER_DIR}/CMSIS/Device/ST/STM32L4xx/Include
    PUBLIC ${DRIVER_DIR}/STM32L4xx_HAL_Driver/Inc
    PUBLIC ${DRIVER_DIR}/BSP/Components/Common
    PUBLIC ${DRIVER_DIR}/BSP/Components/hts221
    PUBLIC ${DRIVER_DIR}/BSP/Components/lps22hb
    PUBLIC ${DRIVER_DIR}/BSP/Components/lsm303agr
    PUBLIC ${DRIVER_DIR}/BSP/Components/lsm6dsm
    PUBLIC ${DRIVER_DIR}/BSP/Components/pcm1774
    PUBLIC ${DRIVER_DIR}/BSP/Components/stc3115
    PUBLIC ${DRIVER_DIR}/BSP/SensorTile
)
