# ------------------------------------------------------- MINIMUM CMAKE VERSION
cmake_minimum_required(VERSION 3.10)

# ---------------------------------------------------------------- PROJECT NAME
set(LOCAL_PROJ_NAME third_party)

# ------------------------------------------------------------------ SEGGER RTT
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES arm_baremetal)

file(GLOB_RECURSE SEGGER_RTT */SEGGER_RTT.c
                             */SEGGER_RTT_printf.c
                             */SEGGER_RTT_ASM_ARMv7M.S
                             */SEGGER_RTT_Syscalls_GCC.c)
set(SRC_FILES ${SEGGER_RTT})
# -------------------------------------------------------------- FIND SRC FILES

# --------------------------------------------------------- DEFINE BUILD TARGET
add_library(${LOCAL_PROJ_NAME} STATIC ${SRC_FILES})

# -------------------------------------------------------------- FIND SRC FILES
# Extract the SEGGER_RTT_Vxxxy from the path
string(REGEX MATCH ".*/(SEGGER_RTT_V[0-9a-z]*)/.*" SEGGER_RTT_VER ${SEGGER_RTT})
target_include_directories(${LOCAL_PROJ_NAME} PUBLIC
                           ${CMAKE_CURRENT_LIST_DIR}/include
                           ${CMAKE_CURRENT_LIST_DIR}/${CMAKE_MATCH_1}/RTT

)

endif()

if((${CMAKE_SYSTEM_PROCESSOR} MATCHES arm_baremetal) AND (${ENABLE_DEBUG_PRINT}))
    target_compile_definitions(${LOCAL_PROJ_NAME} PUBLIC -DENABLE_DEBUG_PRINT)
    message(STATUS "Segger RTT debugging enabled")
endif()
