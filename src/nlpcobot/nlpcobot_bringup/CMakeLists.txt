cmake_minimum_required(VERSION 3.8)
project(nlpcobot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nlpcobot_cpp_py REQUIRED)
find_package(nlpcobot_description REQUIRED)
find_package(nlpcobot_gazebo REQUIRED)
find_package(tm5-700_moveit_config REQUIRED)
find_package(tm_description REQUIRED)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
