cmake_minimum_required(VERSION 3.5)
project(ros2gpt_interfaces)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/AudioData.msg"
    "srv/TranscribeAudio.srv"
    DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
