from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace, SetRemap, SetParameter


def generate_launch_description():
    return LaunchDescription([
        # Handle parameters.
        DeclareLaunchArgument(
            "id", default_value=TextSubstitution(text="0")
        ),
        DeclareLaunchArgument(
            'name', default_value=["agent", LaunchConfiguration("id")]
        ),
        DeclareLaunchArgument(
            'algorithm_pkg', default_value='patrol_algorithms_base'
        ),
        DeclareLaunchArgument(
            'algorithm_name', default_value='Random'
        ),

        # Launch the agent logic node.
        Node(
            package=LaunchConfiguration("algorithm_pkg"),
            executable=LaunchConfiguration("algorithm_name"),
            name="patrol_agent",
            exec_name="patrol_agent"
        )
    ])