cmake_minimum_required(VERSION 3.7)
#project(NANOPB_CMAKE_SIMPLE C)
set(THIS messages)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../nanopb/extra)
find_package(Nanopb REQUIRED)
include_directories(${NANOPB_INCLUDE_DIRS})

nanopb_generate_cpp(PROTO_SRCS PROTO_HDRS gb_messages.proto)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
#add_custom_target(generate_proto_sources DEPENDS ${PROTO_SRCS} ${PROTO_HDRS})
set_source_files_properties(${PROTO_SRCS} ${PROTO_HDRS}
    PROPERTIES GENERATED TRUE)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror -g -O0")

#add_executable(simple simple.c ${PROTO_SRCS} ${PROTO_HDRS})
add_library(${THIS} ${PROTO_SRCS} ${PROTO_HDRS})
target_include_directories(${THIS} PUBLIC ${CMAKE_CURRENT_BINARY_DIR} ${NANOPB_INCLUDE_DIRS})