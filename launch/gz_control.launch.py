import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'nav2_autonomous_gazebo'

    # Declare a launch argument for custom world name
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='simple_arena.sdf',    
        description='Specify world file name.'
    )

    # Use PathJoinSubstitution to create the world path dynamically
    custom_world_path = PathJoinSubstitution([
        get_package_share_directory(package_name), 'worlds', LaunchConfiguration('world')
    ])

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]),
        launch_arguments={'gz_args': ['-r -v7 ', custom_world_path], 'on_exit_shutdown':'true'}.items(),  # use '-s' to use headless mode.
    )

    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'autonomous_robot',
                                   '-x', '0.0',
                                   '-y', '0.0',
                                   '-z', '0.085'],
                        output='screen')

    load_joint_state_broadcaster = Node(
            package='controller_manager',
            executable='spawner',
            name='spawner_joint_state_broadcaster',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
    )

    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel_gz@geometry_msgs/msg/Twist]ignition.msgs.Twist'],
        output='screen'
    )

    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
        output='screen'
    )

    bridge_tf = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
        output='screen'
    )

    bridge_joint_states = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model'],
        output='screen'
    )

    bridge_scan = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
        output='screen'
    )
    
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out','/cmd_vel_gz')]
    )

    return LaunchDescription([
        declare_world,
        rsp,
        gz_sim,
        spawn_entity,
        bridge_clock,
        bridge_cmd_vel,
        bridge_odom,
        bridge_tf,
        bridge_joint_states,
        bridge_scan,
        twist_mux,
    ])
