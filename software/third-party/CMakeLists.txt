# ------------------------------------------------------- MINIMUM CMAKE VERSION
cmake_minimum_required(VERSION 3.10)

# ---------------------------------------------------------------- PROJECT NAME
set(LOCAL_PROJ_NAME third_party)

# ------------------------------------------------------------------ SEGGER RTT
set(SRC_FILES   ${CMAKE_CURRENT_LIST_DIR}/SEGGER_RTT_V686e/RTT/SEGGER_RTT.c
                ${CMAKE_CURRENT_LIST_DIR}/SEGGER_RTT_V686e/RTT/SEGGER_RTT_printf.c
                ${CMAKE_CURRENT_LIST_DIR}/SEGGER_RTT_V686e/RTT/SEGGER_RTT_ASM_ARMv7M.S
                ${CMAKE_CURRENT_LIST_DIR}/SEGGER_RTT_V686e/Syscalls/SEGGER_RTT_Syscalls_GCC.c
    )
# -------------------------------------------------------------- FIND SRC FILES

# --------------------------------------------------------- DEFINE BUILD TARGET
add_library(${LOCAL_PROJ_NAME} STATIC ${SRC_FILES})

# -------------------------------------------------------------- FIND SRC FILES
target_include_directories(${LOCAL_PROJ_NAME} PUBLIC
                           ${CMAKE_CURRENT_LIST_DIR}/include
                           ${CMAKE_CURRENT_LIST_DIR}/SEGGER_RTT_V686e/RTT

)
