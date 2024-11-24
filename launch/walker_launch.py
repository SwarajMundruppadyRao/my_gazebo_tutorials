# Description: Launch file for the walker node
# Authors : Swaraj Mundruppady Rao (swarajmr@umd.edu)
# Version 1.3
# File : walker_launch.py
# Copyright (c) 2024


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Argument for enabling/disabling rosbag recording
    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='true',
        description='Enable rosbag recording (true/false)'
    )

    # Node for the walker
    walker_node = Node(
        package='walker',
        executable='walker_node',
        name='walker_node',
    )

    # Feedback if rosbag recording is enabled
    rosbag_feedback = LogInfo(
        condition=IfCondition(LaunchConfiguration('record_bag')),
        msg='Rosbag recording is enabled.'
    )

    # Create a results directory if it does not exist
    results_dir = os.path.join(os.getcwd(), 'results')
    if not os.path.exists(results_dir):
        os.makedirs(results_dir)

    # Rosbag recording command with output to results directory
    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-x', '/camera/.*', '--output', results_dir],
        condition=IfCondition(LaunchConfiguration('record_bag')),
        output='screen'
    )

    # Timer to stop rosbag recording after 30 seconds
    stop_rosbag_record = TimerAction(
        period=30.0,
        actions=[
            ExecuteProcess(
                cmd=['pkill', '-f', 'ros2 bag record'],
                output='screen'
            )
        ],
        condition=IfCondition(LaunchConfiguration('record_bag'))
    )

    # Launch all the nodes and actions
    return LaunchDescription([
        record_bag_arg,
        walker_node,
        rosbag_feedback,
        rosbag_record,
        stop_rosbag_record
    ])
