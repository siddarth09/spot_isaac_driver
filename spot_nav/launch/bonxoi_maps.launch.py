from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    params_file = os.path.join(
            get_package_share_directory('spot_nav'),
            'config',
            'bonxoi_maps.yaml')
 
    return LaunchDescription([

        Node(
            package='easynav_system',
            executable='system_main',
            name='easynav_system',
            output='screen',

            parameters=[
                params_file,
                {"use_real_time": False},
                {"use_sim_time": True},
            ],

            remappings=[
                ('/maps_manager_node/navmap/incoming_pc2_map', '/ouster/point_cloud'),
                ('/maps_manager_node/bonxai/incoming_pc2_map', '/ouster/point_cloud'),
            ]
        )
    ])
