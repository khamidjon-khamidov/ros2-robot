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
    gazebo_pkg_name = 'setup_gazebo_ias0220'
    
    path_to_xacro = os.path.join(
        get_package_share_directory(package_name), "urdf", "differential_robot_simu_task4_part1.xacro"
    )
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(gazebo_pkg_name), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'xacro_file': path_to_xacro}.items()
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

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    rviz_config_file = os.path.join(
        get_package_share_directory(package_name), "config", "task2_config.rviz"
    )

    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        remappings=[
            ('/cmd_vel', '/cmd_vel')
        ]
    )

    # move_node = Node(
    #     package='transform_frame',
    #     executable='move',
    #     name='move'
    # )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["--display-config", rviz_config_file],
    )

    return LaunchDescription(
        [
            gazebo_launch,
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
            teleop_node,
            # move_node,
        ]
    )
