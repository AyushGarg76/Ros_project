# Final Robot ROS

A ROS 2 Humble package for simulating a custom 4-wheeled differential drive robot in Ignition Gazebo (Fortress). This project includes a complete simulation environment with custom sensors, SLAM mapping, and autonomous navigation.

---

## 🚀 Features
- **Accurate Physics**: Properly tuned mass and inertia values for stable simulation of a 4-wheeled robot.
- **Custom Sensors**: 
  - **2D LiDAR**: 360-degree scan, 10Hz update rate, 10m range.
  - **RGB Camera**: Front-facing, 30Hz, 640x480 resolution.
- **Hospital World**: A custom Gazebo Ignition SDF world simulating a hospital corridor with obstacles like beds and trolleys.
- **Velocity PID Controller**: A custom Python node for linear and angular velocity smoothing with anti-windup.
- **SLAM Integration**: `slam_toolbox` configuration for real-time 2D mapping.
- **Navigation Stack**: Nav2 configuration using `RegulatedPurePursuitController` for smooth autonomous movement.

---

## 🛠 Prerequisites
- **ROS 2 Humble**
- **Ignition Gazebo Fortress**
- **Required Packages**:
  ```bash
  sudo apt update
  sudo apt install ros-humble-ros-gz \
                   ros-humble-teleop-twist-keyboard \
                   ros-humble-slam-toolbox \
                   ros-humble-navigation2 \
                   ros-humble-nav2-bringup
  ```

---

## 📥 Installation
1. Clone this repository into your `src` folder:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/AyushGarg76/Ros_project.git final_robot_ros
   ```
2. Build and source:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select final_robot_ros --symlink-install
   source install/setup.bash
   ```

---

## 🎮 Usage

### 1. Launch Simulation & SLAM
Starts Gazebo, spawns the robot, establishes the ROS-Ignition bridge, and starts the SLAM node.
```bash
ros2 launch final_robot_ros gazebo.launch.py
```

### 2. Teleoperation
Control the robot manually to build a map:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 3. Save the Map
Once the map is visible in RViz and looks complete:
```bash
mkdir -p src/final_robot_ros/maps
ros2 run nav2_map_server map_saver_cli -f src/final_robot_ros/maps/hospital_map
```

### 4. Autonomous Navigation
Stop the SLAM node and launch the Nav2 stack:
```bash
ros2 launch final_robot_ros navigation.launch.py map:=src/final_robot_ros/maps/hospital_map.yaml
```
Use the **"Nav2 Goal"** tool in RViz to set a destination!

### 5. PID Velocity Controller (Optional)
To use the smoothed velocity controller:
```bash
ros2 run final_robot_ros pid_controller.py
```
*Note: This node listens to `/cmd_vel_raw` and publishes to `/cmd_vel`.*

---

## 📂 File Structure
- `urdf/final_robot_ros.urdf`: Main robot description with sensors and Gazebo plugins.
- `worlds/hospital.sdf`: Custom hospital environment.
- `config/`:
  - `nav2_params.yaml`: Navigation parameters (RPP controller, footprint, etc.).
  - `mapper_params_online_async.yaml`: SLAM toolbox configuration.
- `launch/`:
  - `gazebo.launch.py`: Simulation and SLAM bringup.
  - `navigation.launch.py`: Nav2 bringup.
- `scripts/pid_controller.py`: Velocity PID filter node.
- `meshes/`: Visual and collision STL files.

---

## 🔧 Maintenance
To update the project after modifications:
```bash
colcon build --packages-select final_robot_ros
source install/setup.bash
```
