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
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    package_name = "ias0220_246075"
    gazebo_pkg_name = "setup_gazebo_ias0220"

    path_to_xacro = os.path.join(
        get_package_share_directory(package_name),
        "urdf",
        "differential_robot_simu_task7.xacro",
    )

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

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", path_to_xacro]), value_type=str
                )
            }
        ],
    )

    rviz_config_file = os.path.join(
        get_package_share_directory(package_name), "config", "differential_robot_rviz_task7.rviz"
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

    config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'simple_control_v2.yaml'
    )
    
    simple_control = Node(
        package=package_name,
        executable='simple_control',
        name='simple_control',
        output='screen',
        parameters=[config],
        emulate_tty=True
    )

    return LaunchDescription([
            robot_state_publisher_node,
            encoders_node,
            my_odom_node,
            static_tf_node,
            gazebo_launch,
            rviz_node,
            simple_control,
        ]
    )
