cmake_minimum_required(VERSION 3.0.2)
project(can-frame-interpreter)

include(CMakeSettings.txt)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra)
add_compile_options(-DCMAKE_BUILD_TYPE=Debug)

add_definitions("-DSOCKETCAN_INTERFACE=\"${SOCKETCAN_INTERFACE}\"")
message(STATUS "Socketcan interface is set to ${SOCKETCAN_INTERFACE}")

add_definitions("-DRESPONSE_TIMEOUT=${RESPONSE_TIMEOUT}")
message(STATUS "Timeout value is set to ${RESPONSE_TIMEOUT} ms")

set(SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/src CACHE INTERNAL "")
set(INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/include CACHE INTERNAL "")

set(HEADER_FILES ${INCLUDE_DIR}/socketcan.h ${INCLUDE_DIR}/motor.h ${INCLUDE_DIR}/motor_commands.h)
add_library(socketcan ${SRC_DIR}/socketcan.cpp ${HEADER_FILES})
target_include_directories(socketcan PRIVATE ${INCLUDE_DIR})

add_library(motor ${SRC_DIR}/motor.cpp ${HEADER_FILES})
target_include_directories(motor PRIVATE ${INCLUDE_DIR})

add_library(motor_commands ${SRC_DIR}/motor_commands.cpp ${HEADER_FILES})
target_include_directories(motor_commands PRIVATE ${INCLUDE_DIR})

add_executable(frameInterpreter ${SRC_DIR}/frameInterpreter.cpp ${HEADER_FILES})
target_include_directories(frameInterpreter PRIVATE ${INCLUDE_DIR})
target_link_libraries(frameInterpreter socketcan motor motor_commands pthread)
