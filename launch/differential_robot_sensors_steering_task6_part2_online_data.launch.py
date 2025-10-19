import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = "ias0220_246075"
    sensors_package = "ias0220_sensors"
    gazebo_pkg_name = "setup_gazebo_ias0220"

    path_to_xacro = os.path.join(
        get_package_share_directory(package_name),
        "urdf",
        "diff.xacro",
    )

    ias_launch_file = os.path.join(
        get_package_share_directory(sensors_package), "launch", "sensors_interface.launch.py"
    )

    sensors_interface = IncludeLaunchDescription(PythonLaunchDescriptionSource(ias_launch_file))

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(gazebo_pkg_name),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={"xacro_file": path_to_xacro}.items(),
    )

    my_odom_node = Node(
        package="ias0220_246075",
        executable="position_calculator",
        name="my_odom",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    
    encoders_node = Node(
        package="encoders_pkg",
        executable="encoders_node",
        name="encoders_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    steering_node = Node(
        package=package_name,
        executable="steering_node",
        name="steering_node",
        output="screen",
    )

    return LaunchDescription(
        [
            gazebo_launch,
            sensors_interface,
            my_odom_node,
            encoders_node,
            static_tf_node,
            steering_node,
        ]
    )
