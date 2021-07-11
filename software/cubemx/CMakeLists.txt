# ------------------------------------------------------- MINIMUM CMAKE VERSION
cmake_minimum_required(VERSION 3.10)


# ---------------------------------------------------------------- PROJECT NAME
set(LOCAL_PROJ_NAME application)

# -------------------------------------------------------------- DEFINE SYMBOLS
add_definitions(-DUSE_HAL_DRIVER -DSTM32L476xx)

# -------------------------------------------------------------- FIND SRC FILES
file(GLOB_RECURSE CSRC ${CMAKE_CURRENT_LIST_DIR}/Core/*.c
                       ${CMAKE_CURRENT_LIST_DIR}/startup_stm32l476xx.s
                       ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/Device/ST/STM32L4xx/system_stm32l4xx.c
                       ${CMAKE_CURRENT_LIST_DIR}/Drivers/RTOS2/*.c
                       ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32L4xx_HAL_Driver/Src/*.c
                       ${CMAKE_CURRENT_LIST_DIR}/Middlewares/ST/*.c
                       ${CMAKE_CURRENT_LIST_DIR}/Middlewares/Third_Party/FreeRTOS/Source/croutine.c
                       ${CMAKE_CURRENT_LIST_DIR}/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
                       ${CMAKE_CURRENT_LIST_DIR}/Middlewares/Third_Party/FreeRTOS/Source/list.c
                       ${CMAKE_CURRENT_LIST_DIR}/Middlewares/Third_Party/FreeRTOS/Source/queue.c
                       ${CMAKE_CURRENT_LIST_DIR}/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
                       ${CMAKE_CURRENT_LIST_DIR}/Middlewares/Third_Party/FreeRTOS/Source/tasks.c
                       ${CMAKE_CURRENT_LIST_DIR}/Middlewares/Third_Party/FreeRTOS/Source/timers.c
                       ${CMAKE_CURRENT_LIST_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/*.c
                       ${CMAKE_CURRENT_LIST_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/*.c
                       ${CMAKE_CURRENT_LIST_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
                       ${CMAKE_CURRENT_LIST_DIR}/USB_DEVICE/*.c
    )

# ------------------------------------------------ FILE EXCLUSION FOR STM32F469
foreach(item ${CSRC})
    if(${item} MATCHES ".*template.*" OR ${item} MATCHES ".*Template.*")
            message(STATUS "excluding: ${item}")
            list(REMOVE_ITEM CSRC ${item})
    endif()
endforeach()

# message(STATUS "")
# message(STATUS "SOURCE FILES:")
# foreach(item ${CSRC})
#     message(STATUS ${item})
# endforeach()

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES arm_baremetal)
    # ----------------------------------------------- DEFINE BUILD TARGET (ARM)
    add_executable(${LOCAL_PROJ_NAME} ${CSRC})

    #------------------------------------------------- ADD INCLUDE PATHS (ARM)
    target_include_directories(${LOCAL_PROJ_NAME}
        PUBLIC ${CMAKE_CURRENT_LIST_DIR}/Core/Inc
        PUBLIC ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/Include
        PUBLIC ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/Device/ST/STM32L4xx/Include
        PUBLIC ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/RTOS2/Include
        PUBLIC ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32L4xx_HAL_Driver/Inc
        PUBLIC ${CMAKE_CURRENT_LIST_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
        PUBLIC ${CMAKE_CURRENT_LIST_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
        PUBLIC ${CMAKE_CURRENT_LIST_DIR}/Middlewares/Third_Party/FreeRTOS/Source/include
        PUBLIC ${CMAKE_CURRENT_LIST_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
        PUBLIC ${CMAKE_CURRENT_LIST_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
        PUBLIC ${CMAKE_CURRENT_LIST_DIR}/USB_DEVICE/App
        PUBLIC ${CMAKE_CURRENT_LIST_DIR}/USB_DEVICE/Target
    )

    # ------------------------------------------------------- SET LINKER SCRIPT
    set(LDSCRIPTS_PATH ${CMAKE_CURRENT_LIST_DIR}/../ldscripts)

    target_link_libraries(${LOCAL_PROJ_NAME}
        "-T ${CMAKE_CURRENT_LIST_DIR}/STM32L476JGYx_FLASH.ld -L${LDSCRIPTS_PATH}"
    )


    # ------------------------------------------------------------ SET MAP FILE
    target_link_libraries(${LOCAL_PROJ_NAME}
        "-Xlinker -Map=${CMAKE_CURRENT_BINARY_DIR}/${LOCAL_PROJ_NAME}.map"
    )


else()
    # -------------------------------------------- DEFINE BUILD TARGET (X86/UT)
    add_library(${LOCAL_PROJ_NAME} INTERFACE)

    # ---------------------------------------------------- ADD HEADERS (X86/UT)
    file(GLOB_RECURSE LOCAL_HEADERS ${CMAKE_CURRENT_LIST_DIR}/*.h)
    target_sources(${LOCAL_PROJ_NAME} INTERFACE ${LOCAL_HEADERS})

    # ---------------------------------------------- ADD INCLUDE PATHS (x86/UT)
    target_include_directories(${LOCAL_PROJ_NAME}
        INTERFACE ${CMAKE_CURRENT_LIST_DIR}
        INTERFACE ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/Include
        INTERFACE ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/Device/ST/STM32F3xx/Include
        INTERFACE ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F3xx_HAL_Driver/Inc
        INTERFACE ${CMAKE_CURRENT_LIST_DIR}/Core/Inc
    )
endif()
