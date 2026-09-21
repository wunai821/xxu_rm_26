"""Launch the independent FlyBrain bridge in safe shadow mode by default."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare("flybrain_nav_bridge"), "config", "flybrain_controller.yaml"
    ])
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="shadow"),
        DeclareLaunchArgument("control_group", default_value="flybrain"),
        DeclareLaunchArgument("input_cmd_topic", default_value="/cmd_vel_nav"),
        DeclareLaunchArgument("output_cmd_topic", default_value="/cmd_vel_nav"),
        DeclareLaunchArgument("brain_device", default_value="auto"),
        DeclareLaunchArgument("rewired_data_dir", default_value=""),
        DeclareLaunchArgument("real_data_dir", default_value="/home/naiwu/fly-data"),
        DeclareLaunchArgument("brain_checksum_sha256", default_value=""),
        DeclareLaunchArgument("benchmark_scenario", default_value=""),
        DeclareLaunchArgument("benchmark_seed", default_value="-1"),
        DeclareLaunchArgument("logging", default_value="false"),
        DeclareLaunchArgument("logging_root", default_value="/home/naiwu/fly_brain/gazebo_results"),
        # MaleCNS has a pinned scientific Python stack. Run the bridge under
        # that interpreter while preserving the system ROS path and its PyYAML
        # module. This avoids importing incompatible system NumPy into the
        # process before FlyBrain / SciPy are loaded.
        ExecuteProcess(
            cmd=[
                "/home/naiwu/fly_brain/.venv/bin/python",
                "-c",
                "from flybrain_nav_bridge.flybrain_node import main; main()",
                "--ros-args",
                "--params-file", config,
                "-p", ["mode:=", LaunchConfiguration("mode")],
                "-p", ["control_group:=", LaunchConfiguration("control_group")],
                "-p", ["input_cmd_topic:=", LaunchConfiguration("input_cmd_topic")],
                "-p", ["output_cmd_topic:=", LaunchConfiguration("output_cmd_topic")],
                "-p", ["brain.device:=", LaunchConfiguration("brain_device")],
                "-p", ["brain.rewired_data_dir:=", LaunchConfiguration("rewired_data_dir")],
                "-p", ["brain.real_data_dir:=", LaunchConfiguration("real_data_dir")],
                "-p", ["brain.expected_brain_sha256:=", LaunchConfiguration("brain_checksum_sha256")],
                "-p", ["benchmark.scenario:=", LaunchConfiguration("benchmark_scenario")],
                "-p", ["benchmark.seed:=", LaunchConfiguration("benchmark_seed")],
                "-p", ["logging.enabled:=", LaunchConfiguration("logging")],
                "-p", ["logging.root_dir:=", LaunchConfiguration("logging_root")],
            ],
            additional_env={
                "PYTHONPATH": [
                    "/usr/lib/python3/dist-packages:",
                    EnvironmentVariable("PYTHONPATH", default_value=""),
                ],
            },
            output="screen",
        ),
    ])
