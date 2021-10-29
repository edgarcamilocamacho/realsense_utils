import os
from ament_index_python.packages import get_package_share_directory as pkgpath
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


rviz_config = os.path.join(pkgpath('camera_pose_estimator'), 'rviz', 'realsense_apriltag.rviz')


def generate_launch_description():

    to_launch = []

    to_launch.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
                pkgpath('realsense2_camera'), 'launch'), '/rs_launch.py'] 
            ),
        launch_arguments={
                'enable_pointcloud': 'True',
                'align_depth': 'True',
                'device_type': 'd435',
            }.items(),
        ))

    to_launch.append(Node( package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_config]
        ))

    return LaunchDescription(to_launch)
