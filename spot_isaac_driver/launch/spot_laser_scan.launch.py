from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # -------------------------------------------------------------
    #  Static transform: body → base_link
    # -------------------------------------------------------------
    # static_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="body_to_base_link_tf",
    #     arguments=[
    #         "0", "0", "0",        # x y z
    #         "0", "0", "0",        # roll pitch yaw
    #         "body", "base_link",  # parent → child
    #     ],
    #     parameters=[{"use_sim_time": True}],
    #     output="screen"
    # )
    # static_tf_map = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="body_to_base_link_tf",
    #     arguments=[
    #         "0", "0", "0",        # x y z
    #         "0", "0", "0",        # roll pitch yaw
    #         "map", "world",  # parent → child
    #     ],
    #     parameters=[{"use_sim_time": True}],
    #     output="screen"
    # )

    # -------------------------------------------------------------
    #  PointCloud → LaserScan
    # -------------------------------------------------------------
    pcl_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pcl_to_scan",
        output="screen",
        remappings=[
            ("cloud_in", "/point_cloud"),
            ("scan", "/scan"),
        ],
        parameters=[{
                
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 4.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
    )

    return LaunchDescription([
        
        pcl_to_scan
    ])
