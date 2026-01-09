cmake_minimum_required(VERSION 3.10)
project(FProtocolUdpExample)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(fprotocol_cpp fprotocol.cpp)
target_include_directories(fprotocol_cpp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(proto RobotProto.cpp)
target_link_libraries(proto fprotocol_cpp)
target_include_directories(proto PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(test_main TestRobotMain.cpp)
target_link_libraries(test_main proto fprotocol_cpp)

if(WIN32)
    target_link_libraries(test_main ws2_32)
endif()
