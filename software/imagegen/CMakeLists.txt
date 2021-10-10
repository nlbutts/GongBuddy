# ------------------------------------------------------- MINIMUM CMAKE VERSION
cmake_minimum_required(VERSION 3.4)


# ---------------------------------------------------------------- PROJECT NAME
set(THIS imagegen)


# ---------------------------------------------------------- ONLY BUILD FOR X86
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES arm_baremetal)
    message(FATAL_ERROR "${THIS} CANNOT be built for arm_baremetal!")
endif()


# -------------------------------------------------------------- FIND SRC FILES
file(GLOB_RECURSE SRC_FILES
    ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/*.c
)


# ------------------------------------------------------- DEFINE THE EXECUTABLE
add_executable(${THIS} ${SRC_FILES})
target_link_libraries(${THIS} tclap update buildinfo utility)


# ------------------------------------------------------------- INCLUDE FOLDERS
target_include_directories(${THIS}
    PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include
)


# ------------------------------------------------------------- INSTALL TARGETS
install(TARGETS ${THIS} RUNTIME DESTINATION ./)
