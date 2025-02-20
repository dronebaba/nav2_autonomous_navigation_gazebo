import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'nav2_autonomous_gazebo'

    rviz_config_dir = os.path.join(
            get_package_share_directory(package_name),
            'config',
            'view_slam.rviz')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'gz_control.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'online_async_launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    start_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir],
        output='screen'
    )

    return LaunchDescription([
        simulation,
        slam,
        start_rviz2
    ])
