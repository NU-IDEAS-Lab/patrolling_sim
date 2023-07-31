from launch import LaunchDescription, LaunchContext
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace, SetRemap, SetParametersFromFile, SetParameter

from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction, OpaqueFunction, SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo, EmitEvent
from launch.actions import Shutdown as ShutdownAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration


def generate_launch_description():
    ''' Generates the overall launch description. '''

    startMonitorAction = Node(
        # package="monitoring_control",
        package="patrol_algorithms_cdc2023",
        executable="monitor",
        name="monitor",
        exec_name="monitor"
    )

    return LaunchDescription([
        # Arguments.
        DeclareLaunchArgument(
            'map', default_value='cumberland'
        ),
        DeclareLaunchArgument(
            'algorithm_pkg', default_value='patrol_algorithms_base'
        ),
        DeclareLaunchArgument(
            'algorithm_name', default_value='Random'
        ),
        DeclareLaunchArgument(
            'runtime', default_value='0'
        ),
        DeclareLaunchArgument(
            'output', default_value='./results.zarr'
        ),
        DeclareLaunchArgument(
            'attrition_times', default_value="-1.0,-1.0"
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('patrolling_sim'),
                'config',
                'params_sim_default.yml'
            ])
        ),

        # Enabled colorized console output.
        SetEnvironmentVariable(
            name="RCUTILS_COLORIZED_OUTPUT",
            value="1"
        ),

        # Set parameters from file.
        SetParametersFromFile(LaunchConfiguration("params_file")),

        # Set override parameters.
        SetParameter(name="runtime", value=LaunchConfiguration("runtime")),
        SetParameter(name="agent_count", value=LaunchConfiguration("agent_count")),
        SetParameter(name="algorithm_name", value=LaunchConfiguration("algorithm_name")),
        SetParameter(name="map", value=LaunchConfiguration("map")),
        SetParameter(name="output_file", value=LaunchConfiguration("output")),
        SetParameter(name="attrition_times", value=LaunchConfiguration("attrition_times")),
        SetParameter(
            name="patrol_graph_file",
            value=[
                FindPackageShare("patrolling_sim"),
                "/models/maps/",
                LaunchConfiguration("map"),
                "/",
                LaunchConfiguration("map"),
                ".graph"
            ]
        ),

        # Launch the simulation base.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('grex'),
                    'launch',
                    'simulation.launch.py'
                ])
            ]),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "gazebo_world_file": [FindPackageShare("grex"), "/models/maps/", LaunchConfiguration("map"), "/model.sdf"],
                "agent_launch_file": [FindPackageShare("patrolling_sim"), "/launch/agent.launch.py"],
            }.items()
        ),

        # Monitor/control node.
        startMonitorAction,

        RegisterEventHandler(
            event_handler=OnProcessExit(
            target_action=startMonitorAction,
            on_exit=EmitEvent(event=Shutdown(reason='monitor stopped')))
        )
    ])