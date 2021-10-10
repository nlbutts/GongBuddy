# ------------------------------------------------------- MINIMUM CMAKE VERSION
cmake_minimum_required(VERSION 3.4)

# ---------------------------------------------------------------- PROJECT NAME
set(LOCAL_PROJ_NAME utility)

# -------------------------------------------------------------- FIND SRC FILES
if (BUILD_UT)
    set(SRC_FILES src/Crc32.cpp)
else()
    file(GLOB_RECURSE SRC_FILES
        ${CMAKE_CURRENT_LIST_DIR}/src/*.c
        ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)
endif()
# --------------------------------------------------------- DEFINE BUILD TARGET
add_library(${LOCAL_PROJ_NAME} STATIC ${SRC_FILES})

target_link_libraries(${LOCAL_PROJ_NAME} PUBLIC messages)

target_include_directories(${LOCAL_PROJ_NAME} PUBLIC
${CMAKE_CURRENT_LIST_DIR}/include)

