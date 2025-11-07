# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES.
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Compute the default path to your Spot RL policy
    policy_path = os.path.join(
        get_package_share_directory('spot_isaac_driver'),
        'policy',
        'spot_policy.pt'
    )

    return LaunchDescription([
        # --- Parameters you can override from CLI ---
        DeclareLaunchArgument(
            "publish_period_ms",
            default_value="5",
            description="Publishing period (ms) for Spot controller updates."
        ),
        DeclareLaunchArgument(
            "policy_path",
            default_value=policy_path,
            description="Path to the RL policy file for Spot."
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True",
            description="Use simulation time from Isaac Sim."
        ),

        # --- Node Definition ---
        Node(
            package='spot_isaac_driver',
            executable='spot_controller',
            name='spot_fullbody_controller',
            output='screen',
            parameters=[{
                'publish_period_ms': LaunchConfiguration('publish_period_ms'),
                'policy_path': LaunchConfiguration('policy_path'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
