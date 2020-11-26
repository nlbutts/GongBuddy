# ------------------------------------------------------- MINIMUM CMAKE VERSION
cmake_minimum_required(VERSION 3.7)


# ---------------------------------------------------------- ONLY BUILD FOR ARM
if(NOT ${CMAKE_SYSTEM_PROCESSOR} MATCHES arm_baremetal)
    message(FATAL_ERROR "Application can only be built for arm_baremetal!")
endif()


# --------------------------------------------------------------- PROJECT NAMES
set(THIS_RELEASE application.elf)
set(THIS_DEBUG application_debug.elf)

# ------------------------------------------------------------ Aliases
set(SENSOR_TILE_DIR ${CMAKE_SOURCE_DIR}/STSW-STLKT01_V2.4.0)
set(APPLICATION_DIR ${SENSOR_TILE_DIR}/Projects/SensorTile/Applications/DataLog)
set(DRIVER_DIR      ${SENSOR_TILE_DIR}/Drivers)
set(MIDDLEWARE_DIR  ${SENSOR_TILE_DIR}/Middlewares)

# -------------------------------------------------------------- FIND SRC FILES
file(GLOB_RECURSE SRC_FILES
    ${APPLICATION_DIR}/Src/*.c
    ${APPLICATION_DIR}/STM32CubeIDE/*.s
    ${DRIVER_DIR}/BSP/*.c
    ${DRIVER_DIR}/STM32L4xx_HAL_Driver/*.c
    ${MIDDLEWARE_DIR}/ST/STM32_USB_Device_Library/Core/*.c
    ${MIDDLEWARE_DIR}/ST/STM32_USB_Device_Library/Class/CDC/*.c
    ${MIDDLEWARE_DIR}/Third_Party/FatFs/src/diskio.c
    ${MIDDLEWARE_DIR}/Third_Party/FatFs/src/ff_gen_drv.c
    ${MIDDLEWARE_DIR}/Third_Party/FatFs/src/ff.c
    ${MIDDLEWARE_DIR}/Third_Party/FatFs/src/option/syscall.c
    ${MIDDLEWARE_DIR}/Third_Party/FatFs/src/option/unicode.c
    ${MIDDLEWARE_DIR}/Third_Party/FreeRTOS/Source/list.c
    ${MIDDLEWARE_DIR}/Third_Party/FreeRTOS/Source/tasks.c
    ${MIDDLEWARE_DIR}/Third_Party/FreeRTOS/Source/timers.c
    ${MIDDLEWARE_DIR}/Third_Party/FreeRTOS/Source/queue.c
    ${MIDDLEWARE_DIR}/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
    ${MIDDLEWARE_DIR}/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    ${MIDDLEWARE_DIR}/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
    ${MIDDLEWARE_DIR}/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
    ${CMAKE_CURRENT_LIST_DIR}/../FreeRTOS-Plus-CLI/FreeRTOS_CLI.c
)

foreach(item ${SRC_FILES})
    if(${item} MATCHES ".*template.*")
            message(STATUS "excluding for template: ${item}")
            list(REMOVE_ITEM SRC_FILES ${item})
    endif()
    if(${item} MATCHES ".*Template.*")
            message(STATUS "excluding for Template directory: ${item}")
            list(REMOVE_ITEM SRC_FILES ${item})
    endif()
    if(${item} MATCHES "SensorTile_audio_out.c")
            message(STATUS "excluding for STM32F405: ${item}")
            list(REMOVE_ITEM SRC_FILES ${item})
    endif()
    if(${item} MATCHES "syscalls.c")
            message(STATUS "excluding for STM32F405: ${item}")
            list(REMOVE_ITEM SRC_FILES ${item})
    endif()
endforeach()

# --------------------------------------------------------- DEFINE BUILD TARGET
set(THIS_LINK_LIBS )
add_executable(${THIS_RELEASE} ${SRC_FILES})
target_link_libraries(${THIS_RELEASE} ${THIS_LINK_LIBS})

add_executable(${THIS_DEBUG} ${SRC_FILES})
target_link_libraries(${THIS_DEBUG} ${THIS_LINK_LIBS})


# ------------------------------------------------------------- INCLUDE FOLDERS
set (INC_DIRS
    ${APPLICATION_DIR}/Inc
    ${MIDDLEWARE_DIR}/ST/STM32_USB_Device_Library/Core/Inc
    ${MIDDLEWARE_DIR}/ST/STM32_USB_Device_Library/Class/CDC/Inc
    ${MIDDLEWARE_DIR}/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    ${MIDDLEWARE_DIR}/Third_Party/FreeRTOS/Source/include
    ${MIDDLEWARE_DIR}/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ${MIDDLEWARE_DIR}/Third_Party/FatFs/src
    ${DRIVER_DIR}/CMSIS/Include
    ${DRIVER_DIR}/CMSIS/Device/ST/STM32L4xx/Include
    ${DRIVER_DIR}/STM32L4xx_HAL_Driver/Inc
    ${DRIVER_DIR}/BSP/Components/Common
    ${DRIVER_DIR}/BSP/Components/hts221
    ${DRIVER_DIR}/BSP/Components/lps22hb
    ${DRIVER_DIR}/BSP/Components/lsm303agr
    ${DRIVER_DIR}/BSP/Components/lsm6dsm
    ${DRIVER_DIR}/BSP/Components/pcm1774
    ${DRIVER_DIR}/BSP/Components/stc3115
    ${DRIVER_DIR}/BSP/SensorTile
    ${CMAKE_CURRENT_LIST_DIR}/../FreeRTOS-Plus-CLI
)

target_include_directories(${THIS_RELEASE}
    PRIVATE ${INC_DIRS}
)

target_include_directories(${THIS_DEBUG}
    PRIVATE ${INC_DIRS}
)


# ----------------------------------------------------------- SET LINKER SCRIPT
set(LDSCRIPTS_PATH ${PROJECT_SOURCE_DIR}/linker)

target_link_libraries(${THIS_RELEASE}
    "-T ${LDSCRIPTS_PATH}/STM32L476RGTx_FLASH.ld -L${LDSCRIPTS_PATH}"
)

target_link_libraries(${THIS_DEBUG}
    "-T ${LDSCRIPTS_PATH}/STM32L476RGTx_FLASH.ld -L${LDSCRIPTS_PATH}"
)


# ---------------------------------------------------------------- SET MAP FILE
target_link_libraries(${THIS_RELEASE}
    "-Xlinker -Map=${CMAKE_CURRENT_BINARY_DIR}/${THIS_RELEASE}.map"
)

target_link_libraries(${THIS_DEBUG}
    "-Xlinker -Map=${CMAKE_CURRENT_BINARY_DIR}/${THIS_DEBUG}.map"
)


# ----------------------------------------------------- ADD VERSION INFORMATION
set_property(
    SOURCE ${CMAKE_CURRENT_LIST_DIR}/src/appareo/application/FirmwareVersion.cpp
    APPEND PROPERTY COMPILE_DEFINITIONS
        SMART_NOZZLE_APP_PN=\"${APP_PN}\"
        SMART_NOZZLE_BUILD_VERSION=\"${BUILD_VERSION}\"
        SMART_NOZZLE_BUILD_DATE=\"${BUILD_DATE}\"
        SMART_NOZZLE_VCS_HASH=\"${VCS_HASH}\"
)


# -------------------------------------------------------------- INSTALL TARGET
# The buildserver.sh script (and Jenkins) will depend on these files

set(INSTALL_BASENAME "application-${APP_PN}-${BUILD_VERSION}")

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${THIS_RELEASE}
        RENAME ${INSTALL_BASENAME}.elf
        DESTINATION ./
)
