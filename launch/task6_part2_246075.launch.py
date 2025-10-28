import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, TimerAction
import xacro


def generate_launch_description():
    package_name = "ias0220_246075"
    machine_vision_pkg_name = "machine_vision_part2"

    path_to_xacro = os.path.join(
        get_package_share_directory(package_name),
        "urdf",
        "differential_robot_simu_task6_part2.xacro",
    )

    doc = xacro.parse(open(path_to_xacro))
    xacro.process_doc(doc)
    params = {"robot_description": doc.toxml(), "use_sim_time": True}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[params],
    )

    rviz_config_file = os.path.join(
        get_package_share_directory(package_name), "config", "task6_config_part2.rviz"
    )

    teleop_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        prefix="xterm -e",
        remappings=[("/cmd_vel", "/my_robot/cmd_vel")],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["--display-config", rviz_config_file],
    )

    encoders_node = Node(
        package="encoders_pkg",
        executable="encoders_node",
        name="encoders_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    my_odom_node = Node(
        package="ias0220_246075",
        executable="position_calculator",
        name="my_odom",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    gazebo_playground = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(machine_vision_pkg_name),
                "launch",
                "mvt_main.launch.py",
            )
        ),
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_entity",
        arguments=[
            "-topic",
            "/robot_description",
            "-entity",
            "my_robot",
            "-x",
            "0.6",
            "-y",
            "-7.5",
            "-Y",
            "1.6",
        ],
        output="screen",
    )

    object_recognition_node = Node(
        package="ias0220_246075",
        executable="object_recognition",
        name="object_recognition",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            encoders_node,
            my_odom_node,
            static_tf_node,
            rviz_node,
            teleop_node,
            gazebo_playground,
            spawn_robot,
            object_recognition_node,
        ]
    )
