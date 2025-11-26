from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package paths
    nav_dir = get_package_share_directory('spot_nav')

    # Map files
   
    plan_map_yaml = "/home/ubuntu/quad_ws/src/SPOT ISAAC/spot_nav/maps/warehouse.yaml"


    # AMCL parameter file
    amcl_params = PathJoinSubstitution([
        FindPackageShare('spot_nav'),
        'config',
        'amcl_config.yaml'
    ])

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
    return LaunchDescription([
        # Map Server (Localization)
        
        # static_tf,
        # Map Server (Planning)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server_planning',
            output='screen',
            parameters=[{
                'yaml_filename': plan_map_yaml,
                'frame_id': 'map',          
                'use_sim_time': True,
            }],
            
        ),

        # AMCL node
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_params],

        ),

        # Lifecycle Manager (bring up both map servers + AMCL)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [

                    'map_server_planning',
                    'amcl'
                ],
            }],
        ),
    ])