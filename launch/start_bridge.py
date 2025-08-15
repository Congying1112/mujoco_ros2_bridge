import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    directory = get_package_share_directory('mujoco_ros2_bridge')
    
    return LaunchDescription([
        Node(
            package    = "mujoco_ros2_bridge",
            executable = "mujoco_node",
            output     = "screen",
            # arguments  = [os.path.join(directory, 'models', 'rf2502_new_3', 'rf2502_new_3.xml')],
            # arguments  = [os.path.join(directory, 'models', 'rf2502_new_3', 'scene.xml')],
            arguments  = [os.path.join(directory, 'models', 'SO101', 'scene.xml'), 'SO101'],
            # arguments  = [os.path.join(directory, 'models', 'SO101', 'so101_new_calib.xml')],
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