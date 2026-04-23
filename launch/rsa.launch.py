import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

    pkg_name = 'mecanum_robot'

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim = LaunchConfiguration('use_sim')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    use_sim_time_args = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    use_sim_args = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation (Gazebo) if true'
    )

    use_ros2_control_args = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Use ros2_control if true'
    )

    urdf_file_path = os.path.join(
        get_package_share_directory(pkg_name), 'description', 'robot.urdf.xacro'
    )
    
    robot_config = Command(['xacro ', urdf_file_path, ' use_sim:=', use_sim, ' use_ros2_control:=', use_ros2_control])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_config}]
    )

    return LaunchDescription([
        use_sim_time_args,
        use_sim_args,
        use_ros2_control_args,
        robot_state_publisher
    ])


