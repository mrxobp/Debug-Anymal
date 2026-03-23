# ANYmal Oscillator Controller

A simple example controller for ANYmal that creates oscillating joint motions to produce bouncing and pitching behaviors.

## Overview

The Oscillator Controller makes ANYmal perform oscillating movements by applying sinusoidal oscillations to the knee joints. It supports two modes:
1. Bounce mode - creates vertical bouncing motion with opposite knee movements
2. Pitch mode - creates body pitching motion with synchronized knee movements

The oscillation frequency is controlled by the forward/backward joystick command.

## Structure

The controller is organized into two packages:

1. **anymal_ctrl_oscillator** - Core controller implementation
   - OscillatorController - Base controller class with oscillation logic

2. **anymal_ctrl_oscillator_ros** - ROS interface for the controller
   - OscillatorControllerRos - ROS wrapper with parameter loading and twist subscription
   - Plugin files (`robot_control_plugin.xml` and `.cpp`) register the ROS wrapper for dynamic loading

## Setup and Build Instructions

1. Create a catkin workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
```

2. Add the packages to the `src` directory
```bash
# Make sure both anymal_ctrl_oscillator and anymal_ctrl_oscillator_ros packages
# are placed in the ~/catkin_ws/src directory
```

3. Build with catkin-tools
```bash
cd ~/catkin_ws
catkin config --install
catkin build anymal_ctrl_oscillator anymal_ctrl_oscillator_ros
```

## Running on Robot

To run the controller on the robot, we recommend running the motion control manager with the stack launcher system using a custom stack.

1. Configure the OPC to your robot

```
anymal_setup -a <YOUR_ANYMAL>
```
```
```


1. Open a terminal in LPC: `anymal_ssh -x -c lpc`. In LPC, stop the software stack

```
anymal_software stop
```
```
```

2. In LPC, update your custom anymal configuration file (`~/.ros/config.yaml`) in LPC, and append the content from the [oscillator_controller.yaml](./oscillator_controller.yaml).

3. In the OPC, run again the software stack overlaying with your custom catkin-ws install space:
```bash

```bash
anymal_software start --remote -t robot -a <YOUR_ANYMAL> --catkin-ws-overlay ~/catkin_ws
```
```
```


3. After use, we recommend removing what you added in `~/.ros/config.yaml` in the robot. You can reboot the robot afterward.

## Running in Simulation

1. Source the workspace
```bash
source ~/catkin_ws/install/setup.bash
```

2. Run `sim.py` directly with the oscillator controller configuration:
```bash
rosrun anymal_d sim.py --extension-config-file oscillator_controller.yaml
```