#!/usr/bin/env python3
"""
A launch file to visualize the OnRobot URDF.
It selects the proper top-level xacro file based on the onrobot_type argument.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    # Get evaluated value of onrobot_type
    onrobot_type_val = LaunchConfiguration("onrobot_type").perform(context)
    if onrobot_type_val == "rg6":
        description_file = "rg6.urdf.xacro"
    else:
        description_file = "rg2.urdf.xacro"
    
    description_package = LaunchConfiguration("description_package")
    prefix = LaunchConfiguration("prefix")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "name:=",
            "onrobot",
            " ",
            "prefix:=",
            prefix,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_onrobot.rviz"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    return [joint_state_publisher_node, robot_state_publisher_node, rviz_node]

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "onrobot_type",
            default_value="rg2",
            description="OnRobot type to load.",
            choices=["rg2", "rg6"]
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="onrobot_description",
            description="Package with the OnRobot URDF/XACRO files."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix for joint names (useful for multi-robot setups)."
        )
    )
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
