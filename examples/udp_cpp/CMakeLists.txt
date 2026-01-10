cmake_minimum_required(VERSION 3.10)
project(FProtocolUdpExample)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(fprotocol_cpp fprotocol.cpp)
target_include_directories(fprotocol_cpp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(proto RobotProto.cpp)
target_link_libraries(proto fprotocol_cpp)
target_include_directories(proto PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(main RobotMain.cpp)
target_link_libraries(main proto fprotocol_cpp)
