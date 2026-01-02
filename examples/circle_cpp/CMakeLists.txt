cmake_minimum_required(VERSION 3.10)
project(FProtocolRobotTest)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加编译选项
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 添加fprotocol库
add_library(fprotocol_cpp 
    fprotocol.cpp
)

target_include_directories(fprotocol_cpp PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 添加RobotProto库
add_library(robot_proto
    RobotProto.cpp
)

target_link_libraries(robot_proto fprotocol_cpp)
target_include_directories(robot_proto PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 添加测试可执行文件
add_executable(test_robot TestRobotMain.cpp)

target_link_libraries(test_robot robot_proto fprotocol_cpp)
target_include_directories(test_robot PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 编译器特定选项
if(MSVC)
    target_compile_options(fprotocol_cpp PRIVATE /W4)
    target_compile_options(robot_proto PRIVATE /W4)
    target_compile_options(test_robot PRIVATE /W4)
else()
    target_compile_options(fprotocol_cpp PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(robot_proto PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(test_robot PRIVATE -Wall -Wextra -Wpedantic)
endif()