from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        # Static transform: map -> body
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_body',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'spot']
        ),

        # Octomap server
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            parameters=[
                {"resolution": 0.05},
                {"frame_id": "body"},          # sensor frame
                {"sensor_model": {"max_range": 15.0}},
                {"publish_free_space": True},
            ],
            remappings=[
                ("cloud_in", "/point_cloud"),
            ],
            output='screen'
        ),
    ])
