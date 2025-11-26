import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Use simulation time if running in Isaac Sim
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Map file
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('spot_nav'),
            'maps',
            'warehouse.yaml')
    )

    # Nav2 params
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('spot_nav'),
            'config',
            'nav2_params.yaml')
    )

    # Bringup launch from nav2_bringup package
    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )

    # RViz config (optional)
    rviz_config = os.path.join(
        get_package_share_directory('spot_nav'),
        'rviz',
        'spot_nav.rviz'
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="body_to_base_link_tf",
        arguments=[
            "0", "0", "0",        # x y z
            "0", "0", "0",        # roll pitch yaw
            "body", "base_link",  # parent → child
        ],
        parameters=[{"use_sim_time": True}],
        output="screen"
    )
    static_tf_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="body_to_base_link_tf",
        arguments=[
            "0", "0", "0.8",        # x y z
            "0", "0", "0",        # roll pitch yaw
            "map", "world",  # parent → child
        ],
        parameters=[{"use_sim_time": True}],
        output="screen"
    )
    return LaunchDescription([

        # -------------------------------
        # Launch Arguments
        # -------------------------------
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time (Isaac Sim)'
        ),

        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to occupancy grid map file'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Full path to the Nav2 param file'
        ),

        # -------------------------------
        # Nav2 Bringup (Planner, AMCL, Controller, Costmaps)
        # -------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_file_dir, 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
            }.items()
        ),

        # static_tf,
        # static_tf_map,

    ])
