import os
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.actions import ExecuteProcess

def generate_launch_description():
    is_uvc_cam = LaunchConfiguration('is_uvc_cam', default='false')

    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')
    camera_dir = get_package_share_directory('usb_cam')
    camera_launch_dir = os.path.join(camera_dir, 'launch')
    usb_cam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(camera_launch_dir, 'usb_cam_launch.py')),
            condition=IfCondition(is_uvc_cam),)
    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')),
    )
    return LaunchDescription([
        wheeltec_robot,usb_cam,

        DeclareLaunchArgument('is_uvc_cam',default_value='false'),
  
        launch_ros.actions.Node(
            package='simple_follower_ros2', 
            executable='line_follow', 
            name='line_follow',
            )]
    )

