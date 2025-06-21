#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    remappable_topics = [
        DeclareLaunchArgument("event_topic", default_value="~/events"),
        DeclareLaunchArgument("output_topic", default_value="~/output"),
    ]

    args = [
        DeclareLaunchArgument(
            "name", default_value="pandect_event_hough_node", description="node name"
        ),
        DeclareLaunchArgument(
            "namespace", default_value="", description="node namespace"
        ),
        DeclareLaunchArgument(
            "params",
            default_value=os.path.join(
                get_package_share_directory("pandect_event_hough"),
                "config",
                "params.yml",
            ),
            description="path to parameter file",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS logging level (debug, info, warn, error, fatal)",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false", description="use simulation clock"
        ),
        *remappable_topics,
    ]

    nodes = [
        ComposableNodeContainer(
            name="pandect_event_hough_container",
            namespace=LaunchConfiguration("namespace"),
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="pandect_event_hough",
                    plugin="pandect_event_hough::HoughTransformNode",
                    name=LaunchConfiguration("name"),
                    remappings=[
                        ("~/events", LaunchConfiguration("event_topic")),
                        ("~/output", LaunchConfiguration("output_topic")),
                    ],
                )
            ],
        ),
    ]

    return LaunchDescription(
        [
            *args,
            SetParameter("use_sim_time", LaunchConfiguration("use_sim_time")),
            *nodes,
        ]
    )
