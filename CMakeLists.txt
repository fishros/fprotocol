cmake_minimum_required(VERSION 3.10)
project(fprotocol)

# Set build type to Debug if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# Set the C standard
set(CMAKE_C_STANDARD 99)

# Add the include directories
include_directories(src fproto example/circle_c/)

# Add the -fPIC flag and debug flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -DDEBUG")

add_library(fprotocol_shared SHARED src/fprotocol.c src/fprotocol.h)

# Add the executable for testing
add_executable(TestMain example/circle_c/TestMain.c example/circle_c/ExampleProto.c)

# Link the test executable with the shared library
target_link_libraries(TestMain fprotocol_shared)