#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Declare 'use_sim_time' launch argument (false for real robot, true for simulation)
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true, otherwise use the system clock'
    )

    # Include the Cartographer SLAM launch file from the tuos_simulations package
    carto_pkg = get_package_share_directory('tuos_simulations')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(carto_pkg, 'launch', 'cartographer.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # Launch the Nav2 map_saver_server (lifecycle node) to provide the '/map_saver/save_map' service
    map_saver_server = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # Delay launching the MapSaverNode client to allow SLAM time to build the map
    map_saver_client = TimerAction(
        period=10.0,  # Wait 10 seconds after SLAM starts before saving
        actions=[
            Node(
                package='com2009_team69_2025',
                executable='map_saver_node',
                name='map_saver_node',
                output='screen',
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            )
        ]
    )

    # Launch the exploration action server
    explore_server = Node(
        package='com2009_team69_2025',
        executable='advanced_travel_server.py',
        name='advanced_travel_server',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # Launch the exploration action client
    explore_client = Node(
        package='com2009_team69_2025',
        executable='advanced_travel_client.py',
        name='advanced_travel_client',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_launch,
        map_saver_server,
        map_saver_client,
        explore_server,
        explore_client,
    ])
