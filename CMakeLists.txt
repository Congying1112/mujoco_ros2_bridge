cmake_minimum_required(VERSION 3.10)
project(mujoco_ros2_bridge)

set(CMAKE_CXX_STANDARD 17)

# Use ROS2 ament_cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# MuJoCo
set(MUJOCO_PATH "/opt/mujoco/mujoco-3.3.5") # UPDATE THIS TO YOUR MUJOCO PATH
include_directories(${MUJOCO_PATH}/include/mujoco)
include_directories(${MUJOCO_PATH}/include)
link_directories(${MUJOCO_PATH}/lib)

# GLFW
find_package(glfw3 REQUIRED)

# 头文件
include_directories(${CMAKE_SOURCE_DIR}/include)

# 源文件
file(GLOB SRC_FILES src/*.cpp)

# 添加静态库（或动态库）
add_library(cls_mujoco STATIC src/cls_mujoco.cpp)
add_library(cls_mujoco_node STATIC src/cls_mujoco_node.cpp)

# 添加可执行文件
add_executable(test_ctrl src/test_ctrl.cpp)
add_executable(mujoco_node src/mujoco_node.cpp)

ament_target_dependencies(mujoco_node rclcpp std_msgs sensor_msgs geometry_msgs)
ament_target_dependencies(cls_mujoco_node rclcpp std_msgs sensor_msgs geometry_msgs)
ament_target_dependencies(cls_mujoco rclcpp std_msgs sensor_msgs geometry_msgs)

target_link_libraries(test_ctrl cls_mujoco mujoco glfw)
target_link_libraries(mujoco_node cls_mujoco_node cls_mujoco mujoco glfw)

# Install targets
install(TARGETS test_ctrl mujoco_node cls_mujoco
    DESTINATION lib/${PROJECT_NAME}
)

# Install dirs
install(
    DIRECTORY models launch
    DESTINATION share/${PROJECT_NAME})

ament_package()
