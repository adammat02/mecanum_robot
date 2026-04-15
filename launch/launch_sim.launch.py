import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'mecanum_robot'

    world = LaunchConfiguration('world')

    pkg_share = get_package_share_directory(pkg_name)

    world_args = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, 'worlds', 'empty.sdf'),
        description='World to load'
    )
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(pkg_share)
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'rsa.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    gz_path = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(gz_path, 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'mecanum_robot',
            '-z', '0.1'
        ],
        output='screen'
    )

    bridge_params = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}'
        ],
        parameters=[{'use_sim_time': True}]
    )

    mecanum_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'mecanum_controller',
            '--controller-ros-args',
            '-r /mecanum_controller/reference:=/cmd_vel '
            '-r /mecanum_controller/odometry:=/odom '
            '-r /mecanum_controller/tf_odometry:=/tf'
        ]
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad']   
    )

    rviz_config = os.path.join(
        pkg_share, 'config', 'rviz.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            rviz_config
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        world_args,
        rsp,
        gazebo,
        spawn_robot,
        gz_bridge,
        mecanum_drive_spawner,
        joint_broad_spawner,
        rviz
    ])