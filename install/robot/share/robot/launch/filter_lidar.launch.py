import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='robot',
            executable='geometry_lidar_filter.py',
            name='laser_filter_node'
        )
    ])
