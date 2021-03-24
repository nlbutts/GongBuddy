# ------------------------------------------------------- MINIMUM CMAKE VERSION
cmake_minimum_required(VERSION 3.4)

# ---------------------------------------------------------------- PROJECT NAME
set(LOCAL_PROJ_NAME update)

# -------------------------------------------------------------- FIND SRC FILES
file(GLOB_RECURSE SRC_FILES
    ${CMAKE_CURRENT_LIST_DIR}/src/*.c
    ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)

# --------------------------------------------- FILE EXCLUSION FOR X86/UT BUILD
if(NOT ${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm_baremetal")
    foreach(item ${SRC_FILES})
        if(${item} MATCHES "AdvancedFlashProgram.cpp"
            OR ${item} MATCHES "BootblockLogic.cpp"
            OR ${item} MATCHES ".*debug.*\.cpp"
            OR ${item} MATCHES ".*NVSection\.cpp"
            OR ${item} MATCHES ".*JEDEC\.cpp"
            OR ${item} MATCHES ".*Button\.cpp")
            message(STATUS "excluding from build: ${item}")
            list(REMOVE_ITEM SRC_FILES ${item})
        endif()
    endforeach()
endif()

# --------------------------------------------------------- DEFINE BUILD TARGET
add_library(${LOCAL_PROJ_NAME} STATIC ${SRC_FILES})

#target_link_libraries(${LOCAL_PROJ_NAME} PUBLIC)

# -------------------------------------------------------------- FIND SRC FILES
target_include_directories(${LOCAL_PROJ_NAME} PUBLIC
                           ${CMAKE_CURRENT_LIST_DIR}/include
                           ${gongbuddy_SOURCE_DIR}/STSW-STLKT01_V2.4.0/Projects/SensorTile/Applications/DataLog/Inc)
