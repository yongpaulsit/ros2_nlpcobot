cmake_minimum_required(VERSION 3.5)
project(nlpcobot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/AudioData.msg"
    "srv/CaptureImage.srv"
    "srv/DetectObject.srv"
    "srv/ParseCommand.srv"
    "srv/TranscribeAudio.srv"
    "action/MoveRobot.action"
    DEPENDENCIES std_msgs std_srvs sensor_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
