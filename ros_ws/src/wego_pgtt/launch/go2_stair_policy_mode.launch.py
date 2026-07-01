import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("wego_pgtt")
    repo_root = os.path.abspath(os.path.join(package_share, "..", "..", "..", "..", ".."))
    default_policy = os.path.join(repo_root, "policies", "policy_go2_pgtt_level20_run0")
    default_interface = os.environ.get("GO2_IFACE", "")

    return LaunchDescription([
        DeclareLaunchArgument("policy_file", default_value=default_policy),
        DeclareLaunchArgument("command_source", default_value="controller"),
        DeclareLaunchArgument("auto_start_go2_driver", default_value="true"),
        DeclareLaunchArgument("go2_driver_setup", default_value="/home/wego/dddmr_navigation/install/setup.bash"),
        Node(
            package="wego_pgtt",
            executable="policy_mode_server",
            name="wego_pgtt_policy_mode_server",
            output="screen",
            parameters=[{
                "service_name": "toggle_pgtt_mode",
                "repo_root": repo_root,
                "policy_file": LaunchConfiguration("policy_file"),
                "command_source": LaunchConfiguration("command_source"),
                "network_interface": default_interface,
                "cmd_vel_topic": "/cmd_vel",
                "motion_service_name": "/go2_motion_cmd",
                "go2_driver_setup": LaunchConfiguration("go2_driver_setup"),
                "auto_start_go2_driver": ParameterValue(
                    LaunchConfiguration("auto_start_go2_driver"),
                    value_type=bool,
                ),
                "auto_arm": True,
                "restore_to_classic": True,
            }],
        ),
    ])
