from launch import LaunchDescription, LaunchContext
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace, SetRemap, SetParametersFromFile, SetParameter

from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction, OpaqueFunction
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from statistics import mean

# import launch

def generate_agents(context: LaunchContext, agent_count_subst):
    ''' Generates the list of agent launch descriptions. '''

    # Convert agent count to integer.
    agent_count = int(context.perform_substitution(agent_count_subst))
    agents = []
    for agent in range(agent_count):
        agents += [
            LogInfo(msg=TextSubstitution(text="Creating patrol agent " + str(agent))),
            GroupAction(
                actions = [
                    # Set parameters.
                    SetParameter(name="id_robot", value=str(agent)),
                    SetParameter(name="agent_count", value=str(agent_count)),
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
                    SetParameter(name="initial_pos.x", value="0.0"),
                    SetParameter(name="initial_pos.y", value="0.0"),

                    # Include the robot launch file.
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([
                            PathJoinSubstitution([
                                FindPackageShare('patrolling_sim'),
                                'launch',
                                'robot.launch.py'
                            ])
                        ]),
                        launch_arguments={
                            "id": str(agent),
                            "name": "agent" + str(agent),
                            "algorithm_pkg": LaunchConfiguration("algorithm_pkg"),
                            "algorithm_name": LaunchConfiguration("algorithm_name"),
                        }.items()
                    )
                ]
            )
        ]
    
    return agents

def generate_launch_description():
    ''' Generates the overall launch description. '''

    return LaunchDescription([
        # Arguments.
        DeclareLaunchArgument(
            'agent_count', default_value='1'
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='false'
        ),
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
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('patrolling_sim'),
                'config',
                'params_sim_default.yml'
            ])
        ),


        # Launch the simulation base.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('simulation_base'),
                    'launch',
                    'mas_simulation.launch.py'
                ])
            ]),
            launch_arguments={
                "agent_count": LaunchConfiguration("agent_count"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "map": LaunchConfiguration("map"),
                "gazebo_world_file": [FindPackageShare("simulation_base"), "/models/maps/", LaunchConfiguration("map"), "/model.sdf"]
            }.items()
        ),

        # Set parameters from file.
        SetParametersFromFile(LaunchConfiguration("params_file")),

        # Agent nodes.
        OpaqueFunction(
            function=generate_agents,
            args=[LaunchConfiguration('agent_count')]
        ),
    ])