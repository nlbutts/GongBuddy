cmake_minimum_required(VERSION 3.7)

set(THIS messages)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../nanopb/extra)
find_package(Nanopb REQUIRED)
include_directories(${NANOPB_INCLUDE_DIRS})

#nanopb_generate_cpp(PROTO_SRCS PROTO_HDRS gb_messages.proto)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
#add_custom_target(generate_proto_sources DEPENDS ${PROTO_SRCS} ${PROTO_HDRS})
set_source_files_properties(${PROTO_SRCS} ${PROTO_HDRS}
    PROPERTIES GENERATED TRUE)

#add_executable(simple simple.c ${PROTO_SRCS} ${PROTO_HDRS})
set(PROTO_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/../nanopb/pb_encode.c ${CMAKE_CURRENT_SOURCE_DIR}/../nanopb/pb_decode.c ${CMAKE_CURRENT_SOURCE_DIR}/../nanopb/pb_common.c)
add_library(${THIS} gb_messages.pb.c ${PROTO_SRCS} ${PROTO_HDRS})
target_include_directories(${THIS} PUBLIC ${CMAKE_CURRENT_LIST_DIR} ${NANOPB_INCLUDE_DIRS})
