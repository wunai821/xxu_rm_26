"""Start the AI controller tuner together with its staged Nav2 goal sender."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    debug_stage = LaunchConfiguration("debug_stage")
    loop_goals = LaunchConfiguration("loop_goals")
    start_goal_sender = LaunchConfiguration("start_goal_sender")

    default_params_file = PathJoinSubstitution([
        FindPackageShare("ai_controller_tuner"),
        "config",
        "ai_controller_tuner.yaml",
    ])

    tuner = Node(
        package="ai_controller_tuner",
        executable="ai_controller_tuner",
        name="ai_controller_tuner",
        output="screen",
        parameters=[
            params_file,
            {
                "debug_stage": debug_stage,
            },
        ],
    )

    goal_sender = Node(
        package="ai_controller_tuner",
        executable="ai_controller_goal_sender",
        name="ai_controller_goal_sender",
        output="screen",
        condition=IfCondition(start_goal_sender),
        parameters=[
            params_file,
            {
                "debug_stage": debug_stage,
                "loop_goals": ParameterValue(loop_goals, value_type=bool),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="YAML parameter file for both tuner and goal sender.",
        ),
        DeclareLaunchArgument(
            "debug_stage",
            default_value="translation",
            description="Tuning stage: translation, lookahead, rotation, curvature, approach, comprehensive, custom.",
        ),
        DeclareLaunchArgument(
            "loop_goals",
            default_value="true",
            description="Repeat the stage goal sequence continuously.",
        ),
        DeclareLaunchArgument(
            "start_goal_sender",
            default_value="true",
            description="Start ai_controller_goal_sender together with the tuner.",
        ),
        tuner,
        goal_sender,
    ])
