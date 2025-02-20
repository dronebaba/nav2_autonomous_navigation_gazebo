from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the path to the package where your robot description is located
    package_name = "nav2_autonomous_gazebo"  # Replace with your package name
    package_path = get_package_share_directory(package_name)


    # Path to the robot description file
    default_robot_description_file = PathJoinSubstitution(
        [package_path, "description", "robot.urdf.xacro"]
    )

    world_file_path = PathJoinSubstitution(
        [package_path, "worlds", "simple_arena.sdf"]
    )

    # Declare the robot description file argument
    robot_description_file = LaunchConfiguration("robot_description_file")
    declare_robot_description_file_cmd = DeclareLaunchArgument(
        "robot_description_file",
        default_value=default_robot_description_file,
        description="Path to the robot URDF or SDF file",
    )

    # Declare the robot namespace argument
    robot_namespace = LaunchConfiguration("robot_namespace")
    declare_robot_namespace_cmd = DeclareLaunchArgument(
        "robot_namespace",
        default_value="robot",
        description="Namespace for the robot",
    )

    # Gazebo simulator launch command
    spawn_robot_cmd = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", '/robot_description',
            "-name", "robot",
            "-x", "0", "-y", "0", "-z", "0.2",
        ],
        output="screen",
    )

    # Gazebo server and client processes
    gazebo_server_cmd = ExecuteProcess(
        cmd=["gz", "sim", "-v", "4", world_file_path],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_robot_description_file_cmd,
            declare_robot_namespace_cmd,
            gazebo_server_cmd,
            spawn_robot_cmd,
        ]
    )
