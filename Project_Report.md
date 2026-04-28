# Project Report: Custom Mobile Robot Simulation & Navigation

## 1. Executive Summary
This project focuses on the design, simulation, and autonomous navigation of a custom 4-wheeled mobile robot within a ROS 2 Humble environment. The robot was exported from SolidWorks and integrated into the Gazebo Ignition simulation platform. Key milestones achieved include stable physics simulation, sensor integration (LiDAR/Camera), Simultaneous Localization and Mapping (SLAM), and autonomous navigation using the Nav2 stack.

---

## 2. Robot Design & Modeling
The robot's physical structure and kinematics were developed through a systematic workflow:
- **SolidWorks Export**: The robot model was originally designed in SolidWorks and exported using the URDF Exporter tool.
- **URDF Refinement**: The raw URDF was heavily modified to:
    - Fix center-of-mass (CoM) offsets and inertia values for stable physics.
    - Correct visual rendering and mesh alignment.
    - Add joint state and robot state publishers for ROS 2 compatibility.
- **Visuals & Meshes**: High-fidelity STL/DAE meshes were integrated to provide a realistic appearance in both Gazebo and RViz.

## 3. Simulation Environment
The simulation is hosted in **Gazebo Ignition (Fortress)**, providing a high-performance physics and rendering engine.
- **Custom Hospital World**: A detailed hospital environment with corridors and rooms was created to test the robot's navigation capabilities.
- **ROS-GZ Bridge**: Configured a robust communication layer between ROS 2 and Gazebo for:
    - **Control**: `cmd_vel` for velocity commands.
    - **Odometry**: `/odom` and `/tf` for pose tracking.
    - **Sensors**: LiDAR scan data and Camera image streams.

## 4. Perception & Sensing
The robot is equipped with a modern sensor suite to perceive its surroundings:
- **LiDAR (Laser Scanner)**: Integrated a 2D LiDAR for obstacle detection and mapping. Issues with self-collision (robot mesh blocking rays) were resolved by fine-tuning the sensor's mounting and collision filters.
- **RGB Camera**: Added a front-facing camera for visual feedback, with optical frame transformations for correct image orientation in RViz.

## 5. SLAM (Simultaneous Localization and Mapping)
Using the `slam_toolbox` package, the robot can generate high-resolution maps of unknown environments:
- **Asynchronous Mapping**: Implemented the `mapper_params_online_async.yaml` configuration to allow for efficient real-time mapping while the robot is in motion.
- **Map Generation**: Successfully generated and saved maps of the hospital environment (stored in the `/maps` directory).

## 6. Autonomous Navigation (Nav2)
The core of the robot's intelligence is powered by the **Navigation2 (Nav2)** stack:
- **Global & Local Planning**: Configured SmacPlanner (Global) and DWB (Local) for efficient path planning and obstacle avoidance.
- **Recovery Behaviors**: Enabled wait, backup, and spin behaviors to handle tight spaces or trapped scenarios.
- **Lifecycle Management**: Integrated Nav2 launch files to handle the complex startup sequence of localization, planners, and controllers.

---

## 7. Technical Stack
- **OS**: Ubuntu 22.04 LTS (Linux)
- **Framework**: ROS 2 Humble Hawksbill
- **Simulator**: Gazebo Ignition Fortress
- **Mapping**: SLAM Toolbox (Async)
- **Navigation**: Nav2 (Navigation 2 Stack)
- **Visualization**: RViz2

## 8. Conclusion
The project has successfully moved from a static CAD design to a fully autonomous mobile platform. The robot is now capable of exploring unknown environments, building maps, and navigating to goal poses with obstacle avoidance, providing a solid foundation for further development in autonomous robotics.
