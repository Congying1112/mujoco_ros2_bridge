cmake_minimum_required(VERSION 3.10)
project(mujoco-ros-bridge)

set(CMAKE_CXX_STANDARD 17)

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 包含头文件
include_directories(${CMAKE_SOURCE_DIR}/include)

# MuJoCo
set(MUJOCO_PATH "/opt/mujoco/mujoco-3.3.5") # UPDATE THIS TO YOUR MUJOCO PATH
include_directories(${MUJOCO_PATH}/include/mujoco)
include_directories(${MUJOCO_PATH}/include)
link_directories(${MUJOCO_PATH}/lib)

# GLFW
find_package(glfw3 REQUIRED)

# 查找所有源文件
file(GLOB SRC_FILES src/*.cpp)

# 添加静态库（或动态库）
add_library(cls_mujoco STATIC src/cls_mujoco.cpp)

# 添加可执行文件（可扩展）
add_executable(test_ctrl src/test_ctrl.cpp)
# 你可以继续添加其他可执行文件，如：
# add_executable(other_exec src/other_exec.cpp)

target_link_libraries(test_ctrl cls_mujoco mujoco glfw)
# target_link_libraries(other_exec cls_mujoco mujoco glfw)

# 复制models文件夹到输出目录
add_custom_command(TARGET test_ctrl POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/models $<TARGET_FILE_DIR:test_ctrl>/models)
# 如有其他可执行文件，复制models同理
# add_custom_command(TARGET other_exec POST_BUILD ...)
