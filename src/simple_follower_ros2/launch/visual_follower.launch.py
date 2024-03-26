import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from launch.conditions import IfCondition

#def launch(launch_descriptor, argv):
def generate_launch_description():
    use_astrapro = LaunchConfiguration('use_astrapro', default='false')
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    astra_dir = get_package_share_directory('ros2_astra_camera')
    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')),
    )

    depth_img = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(astra_dir,'launch', 'astra_pro_launch.py')),
            condition=IfCondition(use_astrapro),
            )

    return LaunchDescription([
        wheeltec_robot,depth_img,
        
        DeclareLaunchArgument('use_astrapro',default_value='false'),
        
        launch_ros.actions.Node(
            package='simple_follower_ros2', 
            executable='visualtracker', 
            name='visualtracker',
             ),
        launch_ros.actions.Node(
            package='simple_follower_ros2', 
            executable='visualfollow', 
            name='visualfollow',
            output='screen',
            ),]
            
    )

