import os
import datetime
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = "ias0220_246075"
    sensors_package = "ias0220_sensors"

    bags_dir = os.path.join(
        os.path.expanduser("~/ros2_ws/src"),
        package_name,
        "bags"
    )
    os.makedirs(bags_dir, exist_ok=True)

    # unique name to avoid collisions
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_output_path = os.path.join(bags_dir, f"recorded_{timestamp}")

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_pub",
        arguments=["--frame-id", "map", "--child-frame-id", "imu_link"],
        output="screen"
    )

    serial_node = Node(
        package=sensors_package,
        executable="serial_interface",
        name="serial_interface",
        output="screen",
        parameters=[{'serial_port': '/dev/ttyUSB0'}]
    )

    record_bag = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record", "-a",
            "-o", bag_output_path
        ],
        output="screen"
    )

    return LaunchDescription([
        static_tf,
        serial_node,
        record_bag
    ])
