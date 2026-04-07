# my_bot

ROS 2 package for the robot description, launch files, configuration, simulation assets, and encoder odometry node.

## Package layout

- `src/`: C++ nodes and libraries
- `launch/`: launch entry points
- `config/`: RViz and parameter YAML files
- `description/`: URDF/Xacro robot description files
- `worlds/`: Gazebo world files

## Build

```bash
colcon build --packages-select my_bot
```

## Run

```bash
ros2 launch my_bot rsp.launch.py
ros2 run my_bot encoder_to_odom
```
