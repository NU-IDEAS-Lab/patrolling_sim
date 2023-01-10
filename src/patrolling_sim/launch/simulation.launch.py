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
from ament_index_python.packages import get_package_share_directory
import os
from statistics import mean
import configparser


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

        # Enabled colorized console output.
        SetEnvironmentVariable(
            name="RCUTILS_COLORIZED_OUTPUT",
            value="1"
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
                "use_agents": "false",
                "use_rviz": LaunchConfiguration("use_rviz"),
                "map": LaunchConfiguration("map"),
                "gazebo_world_file": [FindPackageShare("simulation_base"), "/models/maps/", LaunchConfiguration("map"), "/model.sdf"]
            }.items()
        ),

        # Set parameters from file.
        SetParametersFromFile(LaunchConfiguration("params_file")),

        # Set override parameters.
        SetParameter(name="agent_count", value=LaunchConfiguration("agent_count")),
        SetParameter(name="algorithm_name", value=LaunchConfiguration("algorithm_name")),
        SetParameter(name="map", value=LaunchConfiguration("map")),
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

        # Agent nodes.
        OpaqueFunction(
            function=generate_agents,
            args=[LaunchConfiguration('agent_count'), LaunchConfiguration('map')]
        ),

        # Monitor/control node.
        Node(
            package="monitoring_control",
            executable="monitor",
            name="monitor",
            exec_name="monitor"
        ),
    ])


def generate_agents(context: LaunchContext, agent_count_subst, map_subst):
    ''' Generates the list of agent launch descriptions. '''

    # Convert contextual info to variables.
    agent_count = int(context.perform_substitution(agent_count_subst))
    map = str(context.perform_substitution(map_subst))

    # Load starting positions.
    try:
        initPosesFile = os.path.join(get_package_share_directory("patrolling_sim"), "config", "initial_poses.txt")
        initPosesDict = loadInitPoses(initPosesFile)
        initPosesKey = map.lower() + "_" + str(agent_count)
        initPoses = str(initPosesDict[initPosesKey])
        initPoses = initPoses.split(" ")
    except:
        return [
            LogInfo(
                msg="Failed to create agents! Could not load initial poses from file {0} with key {1}".format(initPosesFile, initPosesKey)
            ),
            ShutdownAction(
                reason="Failed to create agents! Could not load initial poses from file {0} with key {1}".format(initPosesFile, initPosesKey)
            )
        ]

    agents = []
    for agent in range(agent_count):
        agents += [
            LogInfo(msg=TextSubstitution(text="Creating patrol agent " + str(agent))),
            GroupAction(
                actions = [
                    # Set parameters.
                    SetParameter(name="id_robot", value=str(agent)),
                    SetParameter(name="initial_pos.x", value=str(initPoses[agent * 2])),
                    SetParameter(name="initial_pos.y", value=str(initPoses[agent * 2 + 1])),

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
                    ),

                    # Launch the actual robot (copied from simulation_base)
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([
                            PathJoinSubstitution([
                                FindPackageShare('simulation_base'),
                                'launch',
                                'robot.launch.py'
                            ])
                        ]),
                        launch_arguments={
                            "id": str(agent),
                            "name": "agent" + str(agent),
                            "use_rviz": LaunchConfiguration("use_rviz"),
                            "map": LaunchConfiguration("map"),
                            "pose_x": str(initPoses[agent * 2]),
                            "pose_y": str(initPoses[agent * 2 + 1]),
                        }.items()
                    )
                ]
            )
        ]
    
    return agents


def loadInitPoses(configFile):
    ''' Load initial poses from configuration file. '''
    initPoses = {}
    try:
        ConfigIP = configparser.ConfigParser()
        ConfigIP.read(configFile)
        for option in ConfigIP.options("InitialPoses"):
            # print(option)
            initPoses[option] = ConfigIP.get("InitialPoses", option)
    except:
        print("Could not load initial poses file")
    return initPoses
