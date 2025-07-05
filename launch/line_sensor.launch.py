#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('ae_line_sensor')
    xacro_file = os.path.join(pkg, 'urdf', 'ae_njl5901ar_8ch.xacro')

    # Gazebo プラグイン検索パス
    env = SetEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        '/opt/ros/humble/lib:' + os.environ.get('GAZEBO_PLUGIN_PATH', '')
    )

    # gzserver + factory + init
    server = ExecuteProcess(
        cmd=['gzserver', '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    client = ExecuteProcess(cmd=['gzclient'], output='screen')

    # robot_description を publish
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command([f'bash -c "xacro {xacro_file}"'])
        }],
        output='screen')

    # ← spawn_entity.py は **オプション無し** が一番安全
    spawn = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-topic', 'robot_description',
            '-entity', 'line_sensor'
        ],
        output='screen')

    sensor = Node(
        package='ae_line_sensor',
        executable='line_sensor_node',
        output='screen')

    return LaunchDescription([env, server, client, rsp, spawn, sensor])

