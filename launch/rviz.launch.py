import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mecanum_robot')

    rviz_config = os.path.join(pkg_share, 'config', 'rviz.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    rviz_overlay = Node(
        package='battery_state_rviz_overlay',
        executable='battery_state_rviz_overlay',
        output='screen',
        remappings=[('battery_state', 'battery_state_broad/battery_state')]
    )

    return LaunchDescription([
        rviz,
        rviz_overlay,
    ])
