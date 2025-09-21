# Launch package
cd ~/ros2_ws
colcon build
source install/setup.bash

ros2 launch IAS0220_246075IVSM task2.launch.py

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

ros2 run IAS0220_246075IVSM test_node