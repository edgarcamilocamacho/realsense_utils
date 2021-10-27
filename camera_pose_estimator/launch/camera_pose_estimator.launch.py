import os
from ament_index_python.packages import get_package_share_directory as pkgpath
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


rviz_config = os.path.join(pkgpath('camera_pose_estimator'), 'rviz', 
                                            'camera_pose_estimator.rviz')
config = os.path.join(pkgpath('camera_pose_estimator'), 'config', 
                                        'camera_pose_estimator_params.yaml')

def generate_launch_description():

    use_gui = LaunchConfiguration('use_gui', default='False')
    launch_rs = LaunchConfiguration('launch_rs', default='False')

    to_launch = []

    to_launch.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                    pkgpath('camera_pose_estimator'), 'launch'), '/rs.launch.py'] 
                ),
            launch_arguments={
                    'enable_pointcloud': 'True',
                    'device_type': 'd435',
                }.items(),
            condition=IfCondition(launch_rs)
            ),
        )

    to_launch.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
                pkgpath('apriltag_ros'), 'launch'), '/tag_realsense.launch.py'] 
            ),
        launch_arguments={
                'camera_name': '/camera/color',
                'image_topic': 'image_raw',
            }.items(),
        ))
    
    # to_launch.append(Node(
    #         package='camera_pose_estimator',
    #         executable='camera_pose_estimator',
    #         parameters = [config],
    #         name='camera_pose_estimator',
    #         output='screen',
    #         emulate_tty=True,
    #     ))

    to_launch.append(Node( package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_config, '--ros-args', '--log-level', 'error'],
            condition = IfCondition(use_gui),
            output='log',
        ))

    return LaunchDescription(to_launch)
