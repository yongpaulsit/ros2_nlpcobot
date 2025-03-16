cmake_minimum_required(VERSION 3.8)
project(nlpcobot_cpp_py)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_msgs REQUIRED)

find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(nlpcobot_interfaces REQUIRED)

### C++ ###
# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable (TEMPLATE)
# add_executable(cpp_executable src/_template_node.cpp)
# ament_target_dependencies(cpp_executable rclcpp)

# Create Cpp executable
add_executable(move_group_interface_node src/move_group_interface_node.cpp)
ament_target_dependencies(move_group_interface_node 
  rclcpp 
  rclcpp_action
  std_srvs
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  nlpcobot_interfaces
)

# Install Cpp executables
install(TARGETS
  # cpp_executable # TEMPLATE
  move_group_interface_node
  DESTINATION lib/${PROJECT_NAME}
)

### PYTHON ###
# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  # scripts/py_node.py # TEMPLATE
  scripts/nlpcobot_node.py
  scripts/speech_to_text_node.py
  scripts/detect_object_node.py
  scripts/parse_command_node.py
  scripts/image_publisher_node.py
  scripts/image_service_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
