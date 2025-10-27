import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = "ias0220_246075"
    student_code = "246075"

    rviz_config_file = os.path.join(
        get_package_share_directory(package_name),
        "config",
        f"task6_part1_{student_code}.rviz"
    )

    image_publisher_node = Node(
        package=package_name,
        executable="image_publisher",
        name="image_publisher",
        output="screen"
    )

    camera_calibration_node = Node(
        package=package_name,
        executable="camera_calibration",
        name="camera_calibration",
        output="screen"
    )

    image_proc_node = Node(
        package="image_proc",
        executable="image_proc",
        name="image_proc",
        remappings=[
            ("image_raw", "/image_raw"),
            ("camera_info", "/camera_info"),
            ("image", "/image_rect")
        ],
        output="screen"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen"
    )

    return LaunchDescription([
        image_publisher_node,
        camera_calibration_node,
        image_proc_node,
        rviz_node
    ])
