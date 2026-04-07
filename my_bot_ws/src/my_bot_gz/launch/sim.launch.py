import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def _cleanup_sdf(_context, sdf_path):
    if os.path.exists(sdf_path):
        os.remove(sdf_path)
    return []


def generate_launch_description():
    my_bot_dir = get_package_share_directory("my_bot")
    sim_dir = get_package_share_directory("my_bot_gz")

    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    use_sim_time = LaunchConfiguration("use_sim_time")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    z_pose = LaunchConfiguration("z_pose")
    yaw = LaunchConfiguration("yaw")

    sdf_file = tempfile.mktemp(prefix="my_bot_", suffix=".sdf")

    robot_description = Command([
        "xacro ",
        os.path.join(my_bot_dir, "description", "robot.urdf.xacro"),
    ])

    generate_sdf = ExecuteProcess(
        cmd=[
            "xacro",
            "-o",
            sdf_file,
            os.path.join(sim_dir, "models", "my_bot", "model.sdf.xacro"),
        ],
        output="screen",
    )

    cleanup_sdf = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[OpaqueFunction(function=lambda context: _cleanup_sdf(context, sdf_file))]
        )
    )

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": ["-r -s ", world], "on_exit_shutdown": "true"}.items(),
    )

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        condition=UnlessCondition(headless),
        launch_arguments={"gz_args": "-g"}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_description}],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        parameters=[{
            "file": sdf_file,
            "name": "my_bot",
            "allow_renaming": False,
            "x": x_pose,
            "y": y_pose,
            "z": z_pose,
            "Y": yaw,
        }],
    )

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_bridge"), "launch", "ros_gz_bridge.launch.py")
        ),
        launch_arguments={
            "bridge_name": "my_bot_bridge",
            "config_file": os.path.join(sim_dir, "config", "bridge.yaml"),
        }.items(),
    )

    encoder_angles_sim = Node(
        package="my_bot_gz",
        executable="encoder_angles_sim",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    encoder_to_odom = Node(
        package="my_bot",
        executable="encoder_to_odom",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(sim_dir, "worlds", "empty.sdf"),
            description="Gazebo world file",
        ),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("x_pose", default_value="0.0"),
        DeclareLaunchArgument("y_pose", default_value="0.0"),
        DeclareLaunchArgument("z_pose", default_value="0.05"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(my_bot_dir)),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(sim_dir)),
        generate_sdf,
        cleanup_sdf,
        gz_server,
        gz_client,
        robot_state_publisher,
        TimerAction(period=2.0, actions=[spawn_robot]),
        bridge,
        encoder_angles_sim,
        encoder_to_odom,
    ])
