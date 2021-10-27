import os
from ament_index_python.packages import get_package_share_directory as pkgpath
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


rviz_config = os.path.join(pkgpath('raya_cameras'), 'rviz', 
                                            'camera_pose_estimator.rviz')
config = os.path.join(pkgpath('raya_cameras'), 'config', 
                                        'camera_pose_estimator_params.yaml')

def generate_launch_description():

    to_launch = []

    # to_launch.append(IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #             pkgpath('realsense2_camera'), 'launch'), '/rs_launch.py'] 
    #         ),
    #     launch_arguments={
    #             'enable_pointcloud': 'True',
    #             'device_type': 'd435',
    #         }.items(),
    #     ))

    # to_launch.append(IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #             pkgpath('apriltag_ros'), 'launch'), '/tag_realsense.launch.py'] 
    #         ),
    #     launch_arguments={
    #             'camera_name': '/camera/color',
    #             'image_topic': 'image_raw',
    #         }.items(),
    #     ))

    to_launch.append(Node(
            package='raya_cameras',
            executable='camera_pose_estimator',
            parameters = [config],
            name='camera_pose_estimator',
            output='screen',
            emulate_tty=True,
        ))

    # to_launch.append(Node( package='rviz2', executable='rviz2', name='rviz2',
    #         arguments=['-d', rviz_config]
    #     ))

    return LaunchDescription(to_launch)
