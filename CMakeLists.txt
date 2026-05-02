cmake_minimum_required(VERSION 3.5)
project(final_robot_ros)

# Suppress unused variable warning for colcon compatibility
set(CATKIN_INSTALL_INTO_PREFIX_ROOT "")


# Find build system
find_package(ament_cmake REQUIRED)

# Install resource directories
install(
  DIRECTORY
    urdf
    meshes
    launch
    config
    resource
    rviz
    textures
    worlds
    scripts
    maps
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts so they are executable via ros2 run
install(
  PROGRAMS
    scripts/pid_controller.py
    scripts/nav_evaluator.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export package
ament_package()
