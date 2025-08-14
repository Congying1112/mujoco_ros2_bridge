# mujoco_ros2_bridge

This package bridges MuJoCo simulation with ROS2. It provides a C++ interface to MuJoCo and exposes ROS2 nodes for simulation control and data exchange.

## Build

```bash
cd ~/workspace/ros2_ws
colcon build --packages-select mujoco_ros2_bridge
```

## Usage

```bash
. install/setup.bash
ros2 run mujoco_ros2_bridge test_ctrl <path_to_model.xml>
```

## Dependencies
- MuJoCo 3.3.5
- GLFW
- ROS2 (rclcpp, std_msgs, sensor_msgs, geometry_msgs)

## Models
MuJoCo models are installed to the package share directory.
