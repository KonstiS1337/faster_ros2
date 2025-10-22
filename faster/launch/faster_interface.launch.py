from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Declare launch arguments
    quad_arg = DeclareLaunchArgument(
        "quad",
        default_value="SQ01s",
        description="Name of the robot namespace"
    )

    is_ground_robot_arg = DeclareLaunchArgument(
        "is_ground_robot",
        default_value="false",
        description="Whether the robot is a ground robot"
    )

    # LaunchConfigurations to use in Node definitions
    quad = LaunchConfiguration("quad")
    is_ground_robot = LaunchConfiguration("is_ground_robot")

    faster_pkg_share = FindPackageShare("faster")

    return LaunchDescription([
        quad_arg,
        is_ground_robot_arg,

        # Start faster_commands.py
        Node(
            package="faster",
            executable="faster_commands.py",
            namespace=quad,
            name="faster_commands",
            output="screen",
            parameters=[{"is_ground_robot": is_ground_robot}]
        ),

        # Start RViz with config file
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            arguments=[
                "-d",
                PathJoinSubstitution([
                    faster_pkg_share,
                    "rviz_cfgs",
                    PathJoinSubstitution(["cvx_", quad, ".rviz"])
                ])
            ]
        ),

        # Start rqt_gui with a perspective file
        Node(
            package="rqt_gui",
            executable="rqt_gui",
            name="rqt_gui",
            arguments=[
                "--perspective-file",
                PathJoinSubstitution([
                    faster_pkg_share,
                    "rqt_cfgs",
                    "demo_interface.perspective"
                ])
            ]
        )
    ])
