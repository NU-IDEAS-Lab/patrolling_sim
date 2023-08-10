from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from launch.actions import GroupAction, OpaqueFunction, LogInfo
from launch.actions import Shutdown as ShutdownAction
from launch_ros.actions import SetParameter
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


import configparser
import os

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

def generate_agent(context: LaunchContext, id_subst, agent_count_subst, map_subst):
    ''' Generates the list of agent launch descriptions. '''

    # Convert contextual info to variables.
    agent_id = int(context.perform_substitution(id_subst))
    agent_count = int(context.perform_substitution(agent_count_subst))
    map = str(context.perform_substitution(map_subst))

    # Load starting positions.
    try:
        initPosesFile = os.path.join(get_package_share_directory("patrolling_sim"), "config", "initial_poses.txt")
        initPosesDict = loadInitPoses(initPosesFile)
        initPosesKey = map.lower() + "_" + str(agent_count)
        initPoses = str(initPosesDict[initPosesKey])
        initPoses = initPoses.split(" ")
        initPosesFloat = [float(x) for x in initPoses]
    except:
        return [
            LogInfo(
                msg="Failed to create agent! Could not load initial poses from file {0} with key {1}".format(initPosesFile, initPosesKey)
            ),
            ShutdownAction(
                reason="Failed to create agent! Could not load initial poses from file {0} with key {1}".format(initPosesFile, initPosesKey)
            )
        ]

    agent =[
        LogInfo(msg=TextSubstitution(text="Creating patrol agent " + str(agent_id))),
        GroupAction(
            actions = [
                # Set parameters.
                SetParameter(name="id_robot", value=str(agent_id)),
                SetParameter(name="initial_pos.x", value=str(initPoses[agent_id * 2])),
                SetParameter(name="initial_pos.y", value=str(initPoses[agent_id * 2 + 1])),
                SetParameter(name="initial_poses", value=str(initPosesFloat)),
                SetParameter(name="tf_prefix", value=f"agent{agent_id}/"),

                # Include the robot launch file.
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('patrolling_sim'),
                            'launch',
                            'robot.launch.py'
                        ])
                    ]),
                    launch_arguments={
                        "id": str(agent_id),
                        "name": "agent" + str(agent_id),
                        "algorithm_pkg": LaunchConfiguration("algorithm_pkg"),
                        "algorithm_name": LaunchConfiguration("algorithm_name"),
                    }.items()
                ),

                # Launch the actual robot
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('grex'),
                            'launch',
                            'agent',
                            'example',
                            'agent.launch.yaml'
                        ])
                    ]),
                    launch_arguments={
                        "initial_pos_x": str(initPoses[agent_id * 2]),
                        "initial_pos_y": str(initPoses[agent_id * 2 + 1]),
                    }.items()
                )
            ]
        )
    ]
    
    return agent


def generate_launch_description():
    return LaunchDescription([
        # Handle parameters.
        DeclareLaunchArgument(
            "id", default_value=TextSubstitution(text="0")
        ),
        DeclareLaunchArgument(
            "namespace", default_value=["agent", LaunchConfiguration("id")]
        ),
        DeclareLaunchArgument(
            "initial_pos_x", default_value='0.0'
        ),
        DeclareLaunchArgument(
            "initial_pos_y", default_value='0.0'
        ),
        DeclareLaunchArgument(
            "initial_pos_z", default_value='0.0'
        ),
        DeclareLaunchArgument(
            'map', default_value='cumberland'
        ),
        DeclareLaunchArgument(
            'map_file', default_value=[FindPackageShare("grex"), "/models/maps/", LaunchConfiguration("map"), "/", LaunchConfiguration("map"), ".yaml"]
        ),
        DeclareLaunchArgument(
            'model_name', default_value='waffle'
        ),
        DeclareLaunchArgument(
            'urdf_path', default_value=[FindPackageShare('grex'), '/models/robots/', LaunchConfiguration("model_name"), '.model']
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='false'
        ),
        DeclareLaunchArgument(
            'simulator_agent_integration_launch_file',
            default_value=[FindPackageShare("grex"), "/launch/simulator/gazebo/agent.launch.yaml"]
        ),


        # Agent nodes.
        OpaqueFunction(
            function=generate_agent,
            args=[LaunchConfiguration('id'), LaunchConfiguration('agent_count'), LaunchConfiguration('map')]
        ),
    ])
