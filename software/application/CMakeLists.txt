# ------------------------------------------------------- MINIMUM CMAKE VERSION
cmake_minimum_required(VERSION 3.10)

# RANT: ST vomits dependency all over the place. You can't really isolate
# parts of the code. Therefore I have one big CMakeList file because ST
# can't figure out how to isolate their code


# ---------------------------------------------------------- ONLY BUILD FOR ARM
if(NOT ${CMAKE_SYSTEM_PROCESSOR} MATCHES arm_baremetal)
    message(FATAL_ERROR "Application can only be built for arm_baremetal!")
endif()


# --------------------------------------------------------------- PROJECT NAMES
set(THIS_RELEASE application.elf)
set(THIS_DEBUG application_debug.elf)


# ---------------------------------------------------------------- DEFINIATIONS
add_definitions(-DUSE_HAL_DRIVER -DSTM32L476xx)

# ----------------------------------------------------------------------- ALIAS
set(CUBEMXDIR       ${CMAKE_SOURCE_DIR}/cubemx)
set(CORE_DIR        ${CUBEMXDIR}/Core)
set(DRIVER_DIR      ${CUBEMXDIR}/Drivers)
set(MIDDLEWARE_DIR  ${CUBEMXDIR}/Middlewares)
set(USB_DIR         ${CUBEMXDIR}/USB_DEVICE)
message(STATUS ${CUBEMXDIR})

# -------------------------------------------------------------- FIND SRC FILES
file(GLOB_RECURSE SRC_FILES
        ${CMAKE_CURRENT_LIST_DIR}/src/*.c
        ${CORE_DIR}/*.c
        ${CUBEMXDIR}/startup_stm32l476xx.s
        ${DRIVER_DIR}/CMSIS/Device/ST/STM32L4xx/system_stm32l4xx.c
        ${DRIVER_DIR}/STM32L4xx_HAL_Driver/Src/*.c
        ${DRIVER_DIR}/BSP/*.c
        ${MIDDLEWARE_DIR}/ST/*.c
        ${MIDDLEWARE_DIR}/Third_Party/FreeRTOS/Source/croutine.c
        ${MIDDLEWARE_DIR}/Third_Party/FreeRTOS/Source/event_groups.c
        ${MIDDLEWARE_DIR}/Third_Party/FreeRTOS/Source/list.c
        ${MIDDLEWARE_DIR}/Third_Party/FreeRTOS/Source/queue.c
        ${MIDDLEWARE_DIR}/Third_Party/FreeRTOS/Source/stream_buffer.c
        ${MIDDLEWARE_DIR}/Third_Party/FreeRTOS/Source/tasks.c
        ${MIDDLEWARE_DIR}/Third_Party/FreeRTOS/Source/timers.c
        ${MIDDLEWARE_DIR}/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/*.c
        ${MIDDLEWARE_DIR}/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/*.c
        ${MIDDLEWARE_DIR}/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
        ${USB_DIR}/*.c
)

foreach(item ${SRC_FILES})
    if(${item} MATCHES ".*template.*" OR ${item} MATCHES ".*Template.*")
            message(STATUS "excluding: ${item}")
            list(REMOVE_ITEM SRC_FILES ${item})
    endif()
endforeach()

# --------------------------------------------------------- DEFINE BUILD TARGET
set(THIS_LINK_LIBS messages)
#add_executable(${THIS_RELEASE} ${SRC_FILES})
#target_link_libraries(${THIS_RELEASE} ${THIS_LINK_LIBS})

add_executable(${THIS_DEBUG} ${SRC_FILES})
target_link_libraries(${THIS_DEBUG} ${THIS_LINK_LIBS})

# ------------------------------------------------------------- INCLUDE FOLDERS
set (INC_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/inc
    ${CMAKE_CURRENT_LIST_DIR}/inc/test
    ${CMAKE_CURRENT_LIST_DIR}/inc/lora
    ${CORE_DIR}/Inc
    ${DRIVER_DIR}/CMSIS/Include
    ${DRIVER_DIR}/CMSIS/Device/ST/STM32L4xx/Include
    ${DRIVER_DIR}/CMSIS/RTOS2/Include
    ${DRIVER_DIR}/STM32L4xx_HAL_Driver/Inc
    ${DRIVER_DIR}/BSP/Components/lsm6dsm
    ${MIDDLEWARE_DIR}/ST/STM32_USB_Device_Library/Class/CDC/Inc
    ${MIDDLEWARE_DIR}/ST/STM32_USB_Device_Library/Core/Inc
    ${MIDDLEWARE_DIR}/Third_Party/FreeRTOS/Source/include
    ${MIDDLEWARE_DIR}/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ${MIDDLEWARE_DIR}/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
    ${MIDDLEWARE_DIR}/Third_Party/LoRaWAN/Utilities
    ${MIDDLEWARE_DIR}/Third_Party/LoRaWAN/Phy
    ${USB_DIR}/App
    ${USB_DIR}/Target
)

# target_include_directories(${THIS_RELEASE}
#     PRIVATE ${INC_DIRS}
# )

target_include_directories(${THIS_DEBUG}
    PRIVATE ${INC_DIRS}
)


# ----------------------------------------------------------- SET LINKER SCRIPT
# target_link_libraries(${THIS_RELEASE} buildinfo third_party
#     "-T ${CUBEMXDIR}/STM32L476JGYx_FLASH.ld -L${CUBEMXDIR}"
# )

target_link_libraries(${THIS_DEBUG} buildinfo third_party
    "-T ${CUBEMXDIR}/STM32L476JGYx_FLASH.ld -L${CUBEMXDIR}"
)


# ---------------------------------------------------------------- SET MAP FILE
# target_link_libraries(${THIS_RELEASE}
#     "-Xlinker -Map=${CMAKE_CURRENT_BINARY_DIR}/${THIS_RELEASE}.map"
# )

target_link_libraries(${THIS_DEBUG}
    "-Xlinker -Map=${CMAKE_CURRENT_BINARY_DIR}/${THIS_DEBUG}.map"
)


# -------------------------------------------------------------- INSTALL TARGET
# The buildserver.sh script (and Jenkins) will depend on these files

set(INSTALL_BASENAME "application-${APP_PN}-${BUILD_VERSION}")

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${THIS_RELEASE}
        RENAME ${INSTALL_BASENAME}.elf
        DESTINATION ./
)
