from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

# Uses the orbslam_helpers::camera_info_injector to "add" a camera_info stream
# to a bagfile which does not contain camera info.
#
# It expects the _input_ image streams to be in the non-standard /bag/...
# namespace, the injector will then output into the "standard" locations
# /left/image_raw, /left/camera_info, etc
#
# Topics stored in a bagfile can be remapped as (for example):
#
#  ros2 bag play  -m /left/image_raw:=/bag/left/image_raw
#                    /right/image_raw:=/bag/right/image_raw
#                 -i <bagfile>


def generate_launch_description():
    left_camera_info = os.path.join(
        get_package_share_directory("orbslam3_odometry"),
        "config",
        "calibrations",
        "clyde",
        "in_water",
        "left_stereo_params.yaml",
    )

    right_camera_info = os.path.join(
        get_package_share_directory("orbslam3_odometry"),
        "config",
        "calibrations",
        "clyde",
        "in_water",
        "right_stereo_params.yaml",
    )

    nodes = [
        Node(
            package="orbslam_helpers",
            name="left_camera_info_injector",
            executable="camera_info_injector",
            parameters=[{"camera_info_file": left_camera_info}],
            remappings=[
                ("input/image_raw", "/bag/left/image_raw"),
                ("output/camera_info", "/left/camera_info"),
                ("output/image_raw", "/left/image_raw"),
            ],
        ),
        Node(
            package="orbslam_helpers",
            name="right_camera_info_injector",
            executable="camera_info_injector",
            parameters=[{"camera_info_file": right_camera_info}],
            remappings=[
                ("input/image_raw", "/bag/right/image_raw"),
                ("output/camera_info", "/right/camera_info"),
                ("output/image_raw", "/right/image_raw"),
            ],
        ),
    ]

    return LaunchDescription(nodes)
