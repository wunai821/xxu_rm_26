"""Start the AI controller tuner together with its staged Nav2 goal sender."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
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
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_clock_bridge = LaunchConfiguration("start_clock_bridge")
    ui_enabled = LaunchConfiguration("ui_enabled")
    ui_host = LaunchConfiguration("ui_host")
    ui_port = LaunchConfiguration("ui_port")
    ui_fallback_to_ephemeral_port = LaunchConfiguration("ui_fallback_to_ephemeral_port")

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
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "ui_enabled": ParameterValue(ui_enabled, value_type=bool),
                "ui_host": ParameterValue(ui_host, value_type=str),
                "ui_port": ParameterValue(ui_port, value_type=int),
                "ui_fallback_to_ephemeral_port": ParameterValue(
                    ui_fallback_to_ephemeral_port, value_type=bool
                ),
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
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            },
        ],
    )

    # Standalone fallback for simulations that have not already bridged Gazebo's
    # /clock topic. Keep this disabled for the normal xxu_bringup simulation,
    # which owns its bridge, so there is exactly one /clock publisher.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ai_tuner_clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
        condition=IfCondition(start_clock_bridge),
    )

    # The XXU simulation launch uses CycloneDDS.  Keep the standalone tuner on
    # the same RMW implementation; otherwise a host with a broken/default
    # Fast DDS setup can fail before either tuner node reaches its own logic.
    rmw_implementation = SetEnvironmentVariable(
        "RMW_IMPLEMENTATION",
        "rmw_cyclonedds_cpp",
    )

    return LaunchDescription([
        rmw_implementation,
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
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Gazebo /clock for both the tuner and its goal sender.",
        ),
        DeclareLaunchArgument(
            "start_clock_bridge",
            default_value="false",
            description="Bridge Gazebo /clock only when the simulation launch does not already do so.",
        ),
        DeclareLaunchArgument(
            "ui_enabled",
            default_value="true",
            description="Enable the tuner Web UI.",
        ),
        DeclareLaunchArgument(
            "ui_host",
            default_value="127.0.0.1",
            description="Tuner Web UI bind host.",
        ),
        DeclareLaunchArgument(
            "ui_port",
            default_value="8765",
            description="Tuner Web UI port; use 0 to request an ephemeral port.",
        ),
        DeclareLaunchArgument(
            "ui_fallback_to_ephemeral_port",
            default_value="true",
            description="Use a free ephemeral port if the configured UI port is occupied.",
        ),
        tuner,
        goal_sender,
        clock_bridge,
    ])
