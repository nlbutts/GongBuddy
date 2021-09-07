# ------------------------------------------------------- MINIMUM CMAKE VERSION
cmake_minimum_required(VERSION 3.4)

if (BUILD_UT)

    # --------------------------------------------------- INCLUDE GMOCK / GTEST
    set(GMOCK_DIR "${CMAKE_CURRENT_LIST_DIR}/googletest")
    add_subdirectory(${GMOCK_DIR})


    # ------------------------------------------------------------ PROJECT NAME
    set(THIS ut)


    # ------------------------------------------------------------- FIND SOURCE
    file(GLOB_RECURSE SRC_FILES
        ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/*.c
    )


    # ------------------------------------------------------------ BUILD TARGET
    add_executable(${THIS} ${SRC_FILES})
    target_link_libraries(${THIS} PRIVATE update comms gmock_main)


    # ------------------------------------------------------- ADD INCLUDE PATHS
    target_include_directories(${THIS}
        PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include
        PRIVATE ${GMOCK_DIR}/googlemock/include
        PRIVATE ${GMOCK_DIR}/googletest/include
    )

    # --------------------------------------------------------- INSTALL TARGETS
    install(TARGETS ${THIS} RUNTIME DESTINATION ./)

endif()
