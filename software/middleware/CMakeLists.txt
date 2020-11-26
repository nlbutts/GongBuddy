# Top level CMake for building the libj1939 library and unit tests
# ----------------------------------------------- Set the minimum CMake version
cmake_minimum_required(VERSION 3.7)


# -------------------------------------------------------- Set the project type
set(THIS middleware)

# ------------------------------------------------------------ Aliases
set(SENSOR_TILE_DIR ${CMAKE_SOURCE_DIR}/STSW-STLKT01_V2.4.0)
set(MIDDLEWARE_DIR  ${SENSOR_TILE_DIR}/Middlewares)


# ------------------------------------------------------------ Get files
file(GLOB_RECURSE SRC_FILES
    ${MIDDLEWARE_DIR}/ST/*.c
    FreeRTOS-Plus-CLI/*.c
)

# foreach(item ${SRC_FILES})
#     if(${item} MATCHES ".*template.c")
#             message(STATUS "excluding for STM32F405: ${item}")
#             list(REMOVE_ITEM SRC_FILES ${item})
#     endif()
#     if(${item} MATCHES "bluenrg_utils_small.c")
#             message(STATUS "excluding for STM32F405: ${item}")
#             list(REMOVE_ITEM SRC_FILES ${item})
#     endif()
#     if(${item} MATCHES "hci_dma_lp.c")
#             message(STATUS "excluding for STM32F405: ${item}")
#             list(REMOVE_ITEM SRC_FILES ${item})
#     endif()
# endforeach()

#----------------------------------------------------- Define the build target
add_library(${THIS} STATIC ${SRC_FILES})

# ----------------------------------------------------------- Set include paths
target_include_directories(${THIS}
    PUBLIC ${MIDDLEWARE_DIR}/ST/STM32_BlueNRG/Interface
    PUBLIC ${MIDDLEWARE_DIR}/ST/STM32_BlueNRG/SimpleBlueNRG_HCI/includes
    PUBLIC ${MIDDLEWARE_DIR}/ST/STM32_USB_Device_Library/Core/Inc
    PUBLIC ${MIDDLEWARE_DIR}/ST/STM32_USB_Device_Library/Class/AUDIO/Inc
    PUBLIC ${MIDDLEWARE_DIR}/ST/STM32_USB_Device_Library/Class/CDC/Inc
)

