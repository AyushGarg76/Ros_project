# Final Robot ROS

A ROS 2 Humble package for simulating a custom diff-drive robot in Ignition Gazebo (Fortress).

This package includes a URDF description of the robot composed of SolidWorks-exported STL meshes, fully configured for physical simulation with proper collision geometry, inertia, and visual materials.

## Features
- **Accurate Physics**: Properly tuned mass and inertia values to ensure stable simulation.
- **Ignition Diff-Drive Plugin**: Ready for standard ROS 2 `cmd_vel` control.
- **ROS-Ignition Bridge**: Includes `ros_gz_bridge` configuration in the launch file to seamlessly connect ROS 2 topics to Ignition Gazebo.
- **Custom STLs**: Scaled and rotated SolidWorks STL meshes for accurate visual representation.

## Prerequisites
- ROS 2 Humble
- Ignition Gazebo Fortress
- `ros-humble-ros-gz` (ROS/Ignition Bridge)
- `ros-humble-teleop-twist-keyboard` (for teleoperation)

Install dependencies:
```bash
sudo apt update
sudo apt install ros-humble-ros-gz ros-humble-teleop-twist-keyboard
```

## Installation
Clone this repository into the `src` folder of your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/AyushGarg76/Ros_project.git final_robot_ros
```

Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select final_robot_ros
source install/setup.bash
```

## Usage

### 1. Launch the Simulation
This launch file will start Ignition Gazebo, load the URDF, spawn the robot, and establish the ROS-Ignition bridge for the `cmd_vel` topic.
```bash
ros2 launch final_robot_ros gazebo.launch.py
```

### 2. Control the Robot
In a new terminal, source your workspace and run the teleop node:
```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Use `i`, `,`, `j`, `l` to move the robot!

## File Structure
- `urdf/final_robot_ros.urdf`: The main robot description file.
- `launch/gazebo.launch.py`: The launch file that starts the simulation and bridges.
- `meshes/`: Directory containing all visual and collision STL files.
