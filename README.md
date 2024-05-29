# MTC Tutorial

## Overview
The `mtc_tutorial` package demonstrates the use of the MoveIt Task Constructor (MTC) with ROS 2. This package includes example launch files and a C++ source file to help you get started with MTC.


## Features
- Demonstrates basic usage of MoveIt Task Constructor in ROS 2
- Example launch files for running the tutorial


## Dependencies
the following repositories are needed for the configuration of the UR5e robot with the gripper:
- [robot_moveit_config](https://github.com/Sohaib-Snouber/robot_moveit_config.git)
- [UR5e_robotiq_gripper_RViz](https://github.com/Sohaib-Snouber/UR5e_robotiq_gripper_RViz.git)


## Installation

### Prerequisites
Ensure you have the following dependencies installed:
- ROS 2 (humble)
- MoveIt 2 (Installation instructions for Humble: [MoveIt 2 Humble Installation Guide](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html))

### Building the Package
Clone the repository into your ROS 2 workspace along with the necessary dependencies and build it using `colcon`:

```bash
cd ~/ros2_ws/src
git clone https://github.com/yourusername/mtc_tutorial.git
git clone https://github.com/Sohaib-Snouber/robot_moveit_config.git
git clone https://github.com/Sohaib-Snouber/UR5e_robotiq_gripper_RViz.git
cd ~/ros2_ws
colcon build
```


## Usage

### Initial Configuration
To configure the necessary components for the UR5e robot with the gripper, run the `mtc_demo.launch.py` file first:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch mtc_tutorial mtc_demo.launch.py
```
This launch file sets up the planning context, loads the MoveIt capabilities, starts the move_group node, RViz, static and robot state publishers, and configures the ros2_control for the robot.

### Run the MTC Tutorial
After the initial configuration is complete, run the tutorial example using the `mtc_tutorial.launch.py` file:

```bash
ros2 launch mtc_tutorial mtc_tutorial.launch.py
```


## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.


## Acknowledgements
Special thanks to the MoveIt and ROS 2 community for their continuous support and development of these powerful tools.

