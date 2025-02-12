cmake_minimum_required(VERSION 3.10)
project(fprotocol)

# Set the C standard
set(CMAKE_C_STANDARD 99)

# Add the include directories
include_directories(src fproto example/circle_c/)

# Add the -fPIC flag and disable optimizations
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -O0")



#target_link_libraries(mainbproto_shared fprotocol_shared)
add_library(fprotocol_shared SHARED src/fprotocol.c src/fprotocol.h )



# Add the executable for testing
add_executable(TestMain example/circle_c/TestMain.c  example/circle_c/ExampleProto.c)

# Link the test executable with the shared library
target_link_libraries(TestMain fprotocol_shared)