# Launch package

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ias0220_246075 differential_robot_task4_part2.launch.py

ros2 run ias0220_246075 position_calculator

ros2 topic echo /encoders_ticks

ros2 node listst

# Record messages from topics. -a flag is to record all topics
ros2 bag record -a

# Record specific topic
ros2 bag record /topic1 /topic2

# Replay recorded bag file
ros2 bag play <bag_file>

# What is in bag
ros2 bag info <bag_file>

# Run plotjuggler
ros2 run plotjuggler plotjuggler

pkill -f gzserver
pkill -f gzclient
ps aux | grep gz

sudo kill -9 28053


# Node list
ros2 node list

# Topic list
ros2 topic list

# Overview of system (source ROS before running)
rqt_graph

# To run your nodes in your package, you need to add entry points: to do this, open the setup.py file. AAdd the following line within the console_scripts brackets of the entry_points field:

```
entry_points={   
       'console_scripts': [
                'entry_point_name = package_name.name_of_the_node_file:main',
        ],
}
```

## Don’t forget to save. Every node has its unique entry point name. You can run your nodes using

```
ros2 run package_name entry_point_name
```

cd ~/ros2_ws && colcon build && ros2 run ias0220_246075 walker

cd ~/ros2_ws && colcon build && ros2 run ias0220_246075 position_calculator


# TURTLESIM
ros2 run turtlesim turtlesim_node

ros2 run turtlesim turtle_teleop_key

ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel

# Odometry message
Odometry message consists of four main components:

1. std_msgs/msg/Header header
   - Contains metadata about the message, including:
     • timestamp (stamp): time when the data was recorded
     • coordinate frame (frame_id): the frame in which the pose is given

2. string child_frame_id
   - Identifies the moving frame, usually the base of the robot

3. geometry_msgs/msg/PoseWithCovariance pose
   - Describes the robot’s position and orientation in the frame_id coordinate frame.

   • Position (geometry_msgs/Point position)
       (x, y, z): Robot’s 3D position
         - For a ground robot, z = 0 as we assume uniform ground.

   • Orientation (geometry_msgs/Quaternion orientation)
       - Orientation is given in quaternions.
       - You need to convert your angles into quaternions for this message.

4. geometry_msgs/msg/TwistWithCovariance twist
   - Describes the robot’s velocity in the child_frame_id coordinate frame.

   • Linear Velocity (geometry_msgs/Vector3 linear)
       (x, y, z): Robot’s speed along each axis
         - For differential drive, y ≈ 0 and z = 0 (you can set both to 0).

   • Angular Velocity (geometry_msgs/Vector3 angular)
       (x, y, z): Rotation rate in rad/s
         - For differential drive, only z is relevant (set x = 0, y = 0).

   • Covariance (float64[36] covariance)
       - A 6x6 matrix representing velocity uncertainty.
       - You do not have to worry about this for now.
