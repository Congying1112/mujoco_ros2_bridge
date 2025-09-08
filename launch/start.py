import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    directory = get_package_share_directory('mujoco_ros2_bridge')
    
    scene_arg = DeclareLaunchArgument(
        'scene',
        default_value=os.path.join(directory, 'models', 'SO101', 'scene.xml'),
        description='Path to scene XML file'
    )

    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='SO101',
        description='Model name to use'
    )
    
    return LaunchDescription([
        scene_arg,
        model_name_arg,
        Node(
            package    = "mujoco_ros2_bridge",
            executable = "mujoco_node",
            output     = "screen",
            arguments  = [LaunchConfiguration('scene'), LaunchConfiguration('model_name')],
            parameters = [   
                            {"joint_state_topic_name" : "joint_state"},
                            {"joint_command_topic_name" : "joint_commands"},
                            {"control_mode" : "POSITION"},
                            {"simulation_frequency" : 1000},
                            {"visualisation_frequency" : 20},
                            {"camera_focal_point": [0.0, 0.0, 0.25]},
                            {"camera_distance": 2.5},
                            {"camera_azimuth": 135.0},
                            {"camera_elevation": -20.0},
                            {"camera_orthographic": True}
                        ]
            )
        ])