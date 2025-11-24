from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # -------------------------------------------------------------
    #  Static transform: body → base_link
    # -------------------------------------------------------------
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
        parameters=[
            {"min_height": -1.0},
            {"max_height":  1.0},
            {"angle_min": -3.14},
            {"angle_max":  3.14},
            {"angle_increment": 0.0035},
            {"range_min": 0.05},
            {"range_max": 20.0},
        ],
    )

    return LaunchDescription([
        static_tf,
        pcl_to_scan
    ])
