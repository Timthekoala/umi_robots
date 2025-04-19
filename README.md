# Z1 Arm Controller

A ROS interface for controlling the Unitree Z1 robot arm.

## Overview

This package provides a ROS node for controlling the Unitree Z1 robotic arm. It offers two primary control interfaces:

1. **Joint Control**: Direct control of the 6 arm joints using position commands
2. **End Effector Pose Control**: Control of the end effector position and orientation using inverse kinematics

The controller handles the communication with the Z1 arm hardware through the Unitree SDK and exposes simple ROS interfaces for commanding the arm.

## Prerequisites

- ROS (tested on ROS Noetic)
- Python 3
- NumPy
- SciPy
- Unitree Z1 SDK (included in `/libs/z1_sdk`)

## Installation

1. Clone this repository into your catkin workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/your-username/umi_robots.git
   ```

2. Build the package:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

3. Source the workspace:
   ```bash
   source ~/catkin_ws/devel/setup.bash
   ```

4. Build Z1 SDK
    ```bash
    cd src/umi_robots/libs/z1_sdk
    mkdir build
    cd build && rm -r ./*
    cmake ..
    make -j
    ``
## Usage

### Launching the Controller

Start the Z1 arm controller node:

```bash
rosrun umi_robots z1_arm_controller.py
```

### Controlling the Arm

#### Joint Control

Send joint position commands to the `/joint_commands` topic:

```bash
# Example: Send joint positions using rostopic
rostopic pub /joint_commands std_msgs/Float64MultiArray "data: [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]"
```

The message should contain 6 values corresponding to the 6 joints of the Z1 arm (in radians). The controller ensures that joint limits are respected.

#### End Effector Pose Control

Send end effector pose commands to the `/endpose_command` topic:

```bash
# Example: Send end effector pose
rostopic pub /endpose_command geometry_msgs/Pose "position:
  x: 0.3
  y: 0.0
  z: 0.4
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0"
```

The controller uses inverse kinematics to calculate the required joint positions to achieve the desired end effector pose.

## Features

- Robust quaternion to Euler angle conversion using SciPy's Rotation class
- Joint limit enforcement
- Inverse kinematics support for end effector pose control
- Clean shutdown procedure that sets the arm to a safe "PASSIVE" mode
- Logging of current state and commanded positions

## Notes

- The arm is initialized in "JOINTCTRL" mode
- Default joint speed is set to 0.5 rad/s
- Default cartesian speed is set to 0.1 m/s 
- When using inverse kinematics, the controller warns if a requested pose is unreachable

## Troubleshooting

If the arm does not respond to commands, check:
1. The robot is powered on and properly connected
2. The controller node is running and not reporting errors
3. The commanded positions are within the joint limits
4. For pose commands, verify that the requested pose is within the reachable workspace

