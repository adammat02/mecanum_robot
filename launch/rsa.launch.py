import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    pkg_name = 'mecanum_robot'

    use_sim_time = LaunchConfiguration('use_sim_time')
    
    use_sim_time_args = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    urdf_file_path = os.path.join(
        get_package_share_directory(pkg_name), 'description', 'robot.urdf.xacro'
    )
    
    robot_config = xacro.process_file(urdf_file_path).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_config}]
    )

    return LaunchDescription([
        use_sim_time_args,
        robot_state_publisher
    ])


