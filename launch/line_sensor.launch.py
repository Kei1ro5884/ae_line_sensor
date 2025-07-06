#!/usr/bin/env python3
"""
Gazebo Classic で 8-ch ラインセンサをスポーンし，
bin/line_sensor_node を直接実行して 8bit ビット列を配信する Launch ファイル
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os, shutil


def generate_launch_description():
    # ───── xacro パスと URDF ─────
    share = get_package_share_directory('ae_line_sensor')
    xacro_file = os.path.join(share, 'urdf', 'ae_njl5901ar_8ch.xacro')
    xacro_exec = shutil.which('xacro') or '/usr/bin/xacro'

    # ★★★ 文字列を 5 トークンに分解し，自前でスペースを埋め込む ★★★
    robot_description = Command([
        'bash -c "',          # トークン 0 末尾に空白+引用符
        xacro_exec,           # トークン 1
        ' ',                  # トークン 2 （区切り用スペース）
        xacro_file,           # トークン 3
        '"'                   # トークン 4 終端の引用符
    ])

    # bin/line_sensor_node のフルパス
    prefix = get_package_prefix('ae_line_sensor')
    node_bin = os.path.join(prefix, 'bin', 'line_sensor_node')

    return LaunchDescription([
        ExecuteProcess(cmd=['gzserver', '--verbose',
                            '-s', 'libgazebo_ros_init.so',
                            '-s', 'libgazebo_ros_factory.so'],
                       output='screen'),
        ExecuteProcess(cmd=['gzclient'], output='screen'),

        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}],
             output='screen'),

        Node(package='gazebo_ros',
             executable='spawn_entity.py',
             arguments=['-topic', 'robot_description',
                        '-entity', 'ae_line_sensor'],
             output='screen'),

        Node(executable=node_bin,
             name='line_sensor_node',
             output='screen'),
    ])

