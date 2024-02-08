from ast import arguments
from http.server import executable
import os
from matplotlib import container
import yaml
import launch
import datetime
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_dir = get_package_share_directory("bringup")
    list = [
        Node(
            package='route_calculator',
            executable='route_calculator',
            output='screen'
        ),
        Node(
            package='route_controller',
            executable='route_controller',
            output='screen'
        ),
        Node(
            package='route_subscriber',
            executable='route_subscriber',
            output='screen'
        ),
        Node(
            package='minimal_timer',
            executable='minimal_timer',
            output='screen'
        ),
        Node(
            package='map_publisher',
            executable='map_publisher',
            output='screen'
        ),

    ]

    return LaunchDescription(list)