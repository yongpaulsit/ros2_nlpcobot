cmake_minimum_required(VERSION 3.5)
project(ros2gpt_main)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 and other dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tm_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros2gpt_interfaces REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Include C++ "include" directory
include_directories(include)

# Add your C++ executables
add_executable(demo_set_positions src/demo_set_positions.cpp)  # Example C++ file
ament_target_dependencies(demo_set_positions rclcpp tm_msgs)

add_executable(moveit_planner src/moveit_planner.cpp)  # Example C++ file
ament_target_dependencies(moveit_planner rclcpp moveit_ros_planning_interface)

# Install C++ executables
install(TARGETS
  moveit_planner
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/commander_node.py
  scripts/tm_move_client.py
  scripts/ros2gpt_node.py
  scripts/whisper_test.py
  DESTINATION lib/${PROJECT_NAME}
)

# Finalize the package
ament_package()