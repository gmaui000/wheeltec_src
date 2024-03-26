#!/usr/bin/env python3
'''
@company: Copyright (C) 2022, WHEELTEC (Dongguan) Co., Ltd
@product: LSM10
@filename: ls_m10.launch.py
@brief:
@version:       date:       author:            comments:
@v2.0           22-4-12      Tues          ROS2

'''
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    frequency = LaunchConfiguration('frequency', default='5.5')
    serial_port_ = LaunchConfiguration('port', default='/dev/wheeltec_lidar')
    baud_rate_ = LaunchConfiguration('baud_rate_', default='460800')
    frame_id_ = LaunchConfiguration('frame_id', default='laser_link')
    scan_topic = LaunchConfiguration('scan_topic', default='scan')

    return LaunchDescription([

        DeclareLaunchArgument('serial_port_', default_value=serial_port_,
            description='Specifying port to connected lidar'),
            
        DeclareLaunchArgument('baud_rate_', default_value=baud_rate_,
            description='Specifying serial baudrate to connected lidar'),
            
        DeclareLaunchArgument('frame_id_', default_value=frame_id_,
            description='Specifying frame_id of lidar. Default frame_id is \'laser\''),
            
        DeclareLaunchArgument('frequency', default_value=frequency,
            description='Specifying frequency property of lidar'),
            
        DeclareLaunchArgument('scan_topic', default_value=scan_topic,
            description='Specifying scan_topic property of lidar'),

        launch_ros.actions.Node(
            package='lsm10_v2',
            executable='LSM10',
            parameters=[{'serial_port_': serial_port_, 
                         'baud_rate_': baud_rate_, 
                         'frame_id_': frame_id_,
                         'frequency': frequency, 
                         'scan_topic': scan_topic}],         
           ),
    ])
