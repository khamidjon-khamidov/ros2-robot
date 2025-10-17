import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = "ias0220_246075"
    ias_package_name = "ias0220_sensors"

    declare_bag_arg = DeclareLaunchArgument(
        'which_bag',
        default_value='bag1',
        description='Bag file to play'
    )

    ias_launch_file = os.path.join(
        get_package_share_directory(ias_package_name),
        "launch",
        "sensors_rviz.launch.py"
    )

    include_ias_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ias_launch_file),
        launch_arguments={'which_bag': LaunchConfiguration('which_bag')}.items()
    )

    rviz_config_file = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "task5_part1_orientation.rviz"
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['--display-config', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        declare_bag_arg,
        include_ias_launch,
        rviz_node,
    ])
