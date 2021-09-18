# ------------------------------------------------------- MINIMUM CMAKE VERSION
cmake_minimum_required(VERSION 3.4)

# ---------------------------------------------------------------- PROJECT NAME
set(LOCAL_PROJ_NAME comms)

# -------------------------------------------------------------- FIND SRC FILES
file(GLOB_RECURSE SRC_FILES
    ${CMAKE_CURRENT_LIST_DIR}/src/*.c
    ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)

# --------------------------------------------------------- DEFINE BUILD TARGET
add_library(${LOCAL_PROJ_NAME} STATIC ${SRC_FILES})

target_link_libraries(${LOCAL_PROJ_NAME} PUBLIC buildinfo messages update utility)

# -------------------------------------------------------------- FIND SRC FILES
target_include_directories(${LOCAL_PROJ_NAME} PUBLIC
                           ${CMAKE_CURRENT_LIST_DIR}/include)
