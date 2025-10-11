import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = "ias0220_246075"

    # Path to your xacro file
    path_to_xacro = os.path.join(
        get_package_share_directory(package_name),
        "urdf",
        "differential_robot.xacro"
    )

    # RViz config
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "task2_config.rviz"
    )

    # robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', path_to_xacro]), value_type=str
            )
        }]
    )

    # Joint state publisher GUI (needed for robot to show in RViz)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # move node from transform_frame package
    move_node = Node(
        package='transform_frame',
        executable='move',
        name='move'
    )

    # teleop_twist_keyboard node with remapping
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',  # opens in separate terminal
        remappings=[
            ('/cmd_vel', '/move/cmd_vel')  # remap to match move node subscription
        ]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['--display-config', rviz_config_file]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        move_node,
        teleop_node,
        rviz_node,
    ])
