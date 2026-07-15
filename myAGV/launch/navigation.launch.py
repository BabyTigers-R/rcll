"""Bring up Nav2 (map_server, AMCL, planner, controller, bt_navigator,
lifecycle managers) for myAGV, reusing nav2_bringup's bringup_launch.py.

Modeled after myagv_navigation2/launch/navigation2_active.launch.py from
elephantrobotics/myagv_ros2, adjusted to load this package's own map/params.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('myagv_navigation')
    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    map_yaml_path = LaunchConfiguration(
        'map', default=os.path.join(pkg_share, 'maps', 'myagv_mapA-v2.yaml'))
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_share, 'params', 'nav2_params.yaml'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_yaml_path,
            description='Full path to the map yaml file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Full path to the Nav2 params file to use'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to start RViz2 alongside Nav2'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_launch_dir, 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen'),
    ])
