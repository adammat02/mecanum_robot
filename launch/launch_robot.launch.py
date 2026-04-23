import os

import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'mecanum_robot'
    pkg_share = get_package_share_directory(pkg_name)

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'rsa.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'use_sim': 'false',
            'use_ros2_control': 'true'
        }.items()
    )

    urdf_file_path = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')

    robot_description = Command([
        'xacro ', urdf_file_path, ' use_sim_time:=false', ' use_ros2_control:=true'
    ])

    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_yaml],
        output='both',
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

    delayed_controller_manager = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    delayed_mecanum_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[mecanum_drive_spawner],
        )
    )

    rviz_config = os.path.join(pkg_share, 'config', 'rviz.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        controller_manager,
        delayed_controller_manager,
        delayed_mecanum_spawner,
        rviz,
    ])
