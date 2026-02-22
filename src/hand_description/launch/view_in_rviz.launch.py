#!/usr/bin/env python3

from pathlib import Path
from ament_index_python import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    path_share = Path(get_package_share_directory("hand_description"))
    path_urdf = path_share / "urdf/robot.urdf"

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": open(path_urdf).read()}],
    )

    jsp_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            path_share / "rviz/basic.rviz",
            # "--ogre-log",
            # "--ros-args",
            # "--log-level",
            # "warn",
        ],
    )

    return LaunchDescription([jsp_gui_node, rsp_node, rviz_node])
