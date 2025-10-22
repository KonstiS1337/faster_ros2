from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    quad_arg = DeclareLaunchArgument(
        "quad",
        default_value="SQ01s",
        description="Namespace for the robot"
    )

    goal_topic_arg = DeclareLaunchArgument(
        "goal_topic",
        default_value="goal",
        description="Topic name for the planner's goal output"
    )

    quad = LaunchConfiguration("quad")
    goal_topic = LaunchConfiguration("goal_topic")

    faster_share = FindPackageShare("faster")

    return LaunchDescription([
        quad_arg,
        goal_topic_arg,

        Node(
            package="faster",
            executable="faster_node",
            namespace=quad,
            name="faster",
            output="screen",
            parameters=[
                # Load parameters from YAML files
                PathJoinSubstitution([
                    faster_share,
                    "param",
                    "faster.yaml"
                ]),
                {
                    "world_dimensions_x" : 30.0, #in m
                    "world_dimensions_y" : 30.0, #in m
                    "world_dimensions_z" : 10.0, #in m
                    "resolution" : 0.5, #in m
                }
            ],
            remappings=[
                ("~/state", "state"),
                ("~/term_goal", "/move_base_simple/goal"),
                ("~/occup_grid", "global_mapper_ros/occupancy_grid"),
                ("~/unknown_grid", "global_mapper_ros/unknown_grid"),
                ("~/pcloud", "camera/cloud"),
                # ("~/odom", "odometry/local_filtered"),  # Optional
                ("~/traj", "traj"),
                ("~/goal", goal_topic),
                ("~/setpoint", "setpoint"),
                ("~/log_topic", "log_topic"),
                # ("~/goal_jackal", "jackal_velocity_controller/cmd_vel"),  # Optional
                # ("~/goal_jackal", "autonomous_cmd_vel"),  # Optional
            ]
        )
    ])
