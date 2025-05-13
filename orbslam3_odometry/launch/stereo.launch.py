from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    odometry_config = os.path.join(
        get_package_share_directory("orbslam3_odometry"),
        "config",
        "orbslam3_odometry.yaml",
    )

    orbslam3_config = os.path.join(
        get_package_share_directory("orbslam3_odometry"),
        "config",
        "aquarium_nano_stereo.yaml",
    )

    orbslam3_vocabulary = os.path.join(
        get_package_share_directory("orbslam3_odometry"), "config", "ORBvoc.txt"
    )

    nodes = [
        Node(
            package="image_proc",
            name="left_rectify_node",
            executable="rectify_node",
            remappings=[
                ("image", "/left/image_raw"),
                ("camera_info", "/left/camera_info"),
                ("image_rect", "/left/image_rect"),
            ],
        ),
        Node(
            package="image_proc",
            name="right_rectify_node",
            executable="rectify_node",
            remappings=[
                ("image", "/right/image_raw"),
                ("camera_info", "/right/camera_info"),
                ("image_rect", "/right/image_rect"),
            ],
        ),
        Node(
            package="orbslam3_odometry",
            name="orbslam3_odometry",
            executable="orbslam3_stereo",
            output="screen",
            # prefix="gdbserver localhost:3000",
            parameters=[
                odometry_config,
                {
                    "path_yaml_settings": orbslam3_config,
                    "path_vocabulary": orbslam3_vocabulary,
                },
            ],
        ),
    ]

    return LaunchDescription(nodes)
