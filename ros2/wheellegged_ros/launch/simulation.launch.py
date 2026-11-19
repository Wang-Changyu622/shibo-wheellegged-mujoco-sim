from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("terrain", default_value="flat"),
        DeclareLaunchArgument("profile", default_value="hardware_2s"),
        DeclareLaunchArgument("viewer", default_value="false"),
        DeclareLaunchArgument("realtime_factor", default_value="1.0"),
        Node(
            package="wheellegged_ros", executable="sim_node", output="screen",
            parameters=[{
                "terrain": LaunchConfiguration("terrain"),
                "profile": LaunchConfiguration("profile"),
                "viewer": ParameterValue(LaunchConfiguration("viewer"), value_type=bool),
                "realtime_factor": ParameterValue(
                    LaunchConfiguration("realtime_factor"), value_type=float),
                "use_sim_time": True,
            }],
        ),
    ])
