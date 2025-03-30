# NLPCobot - A ROS2 Natural Language Interface for Cobot Arms

```bash
.
├── realtimesst.log
└── src
    └── nlpcobot
        ├── nlpcobot_bringup
        │   ├── CMakeLists.txt
        │   ├── config
        │   │   ├── diff_drive.rviz
        │   │   ├── manipulator_bridge.yaml
        │   │   ├── ros_gz_example_bridge.yaml
        │   │   └── rrbot.rviz
        │   ├── launch
        │   │   ├── cobot.launch.py
        │   │   ├── gazebo_empty.launch.py
        │   │   ├── gazebo.launch.py
        │   │   ├── launch.py
        │   │   ├── move_group_interface.launch.py
        │   │   └── test.launch.py
        │   └── package.xml
        ├── nlpcobot_cpp_py
        │   ├── CMakeLists.txt
        │   ├── images
        │   │   ├── rosgraph.png
        │   │   ├── test_img_01.PNG
        │   │   ├── test_img_02.PNG
        │   │   └── test_img_03.PNG
        │   ├── include
        │   │   └── nlpcobot_cpp_py
        │   │       └── cpp_header.hpp
        │   ├── nlpcobot_cpp_py
        │   │   ├── detect_object.py
        │   │   ├── image_conversion.py
        │   │   ├── __init__.py
        │   │   ├── parse_command.py
        │   │   ├── speech_to_text.py
        │   │   ├── _template.py
        │   │   └── tf_camera_to_world.py
        │   ├── package.xml
        │   ├── scripts
        │   │   ├── detect_object_node.py
        │   │   ├── image_publisher_node.py
        │   │   ├── image_service_node.py
        │   │   ├── nlpcobot_node.py
        │   │   ├── parse_command_node.py
        │   │   ├── speech_to_text_node.py
        │   │   └── _template_node.py
        │   └── src
        │       ├── move_group_interface_node.cpp
        │       └── _template_node.cpp
        ├── nlpcobot_description
        │   ├── CMakeLists.txt
        │   ├── meshes
        │   │   ├── tm5-700
        │   │   │   ├── collision
        │   │   │   │   ├── tm5-700_arm1_c.stl
        │   │   │   │   └── tm5-700_arm2_c.stl
        │   │   │   └── visual
        │   │   │       ├── tm5-700_arm1.mtl
        │   │   │       ├── tm5-700_arm1.obj
        │   │   │       ├── tm5-700_arm1.stl
        │   │   │       ├── tm5-700_arm2.mtl
        │   │   │       ├── tm5-700_arm2.obj
        │   │   │       └── tm5-700_arm2.stl
        │   │   └── tm5-900
        │   │       ├── collision
        │   │       │   ├── tm5-900_arm1_c.stl
        │   │       │   ├── tm5-900_arm2_c.stl
        │   │       │   ├── tm5-base_c.stl
        │   │       │   ├── tmr_100w_01_c.stl
        │   │       │   ├── tmr_100w_02_c.stl
        │   │       │   ├── tmr_400w_01_c.stl
        │   │       │   ├── tmr_ee_c.stl
        │   │       │   └── tmr_iox_c.stl
        │   │       └── visual
        │   │           ├── tm5-900_arm1.mtl
        │   │           ├── tm5-900_arm1.obj
        │   │           ├── tm5-900_arm1.stl
        │   │           ├── tm5-900_arm2.mtl
        │   │           ├── tm5-900_arm2.obj
        │   │           ├── tm5-900_arm2.stl
        │   │           ├── tm5-base.mtl
        │   │           ├── tm5-base.obj
        │   │           ├── tm5-base.stl
        │   │           ├── tmr_100w_01.mtl
        │   │           ├── tmr_100w_01.obj
        │   │           ├── tmr_100w_01.stl
        │   │           ├── tmr_100w_02.mtl
        │   │           ├── tmr_100w_02.obj
        │   │           ├── tmr_100w_02.stl
        │   │           ├── tmr_400w_01.mtl
        │   │           ├── tmr_400w_01.obj
        │   │           ├── tmr_400w_01.stl
        │   │           ├── tmr_ee.mtl
        │   │           ├── tmr_ee.obj
        │   │           ├── tmr_ee.stl
        │   │           ├── tmr_iox.mtl
        │   │           ├── tmr_iox.obj
        │   │           └── tmr_iox.stl
        │   ├── package.xml
        │   └── urdf
        │       ├── camera.xacro
        │       └── tm5-700.urdf
        ├── nlpcobot_gazebo
        │   ├── CMakeLists.txt
        │   ├── hooks
        │   │   ├── nlpcobot_gazebo.dsv.in
        │   │   └── nlpcobot_gazebo.sh.in
        │   ├── include
        │   │   └── nlpcobot_gazebo
        │   │       ├── BasicSystem.hh
        │   │       └── FullSystem.hh
        │   ├── package.xml
        │   ├── README.md
        │   ├── src
        │   │   ├── BasicSystem.cc
        │   │   └── FullSystem.cc
        │   └── worlds
        │       ├── camera.sdf
        │       ├── nlpcobot_world.sdf
        │       └── world_joint_reference.sdf
        ├── nlpcobot_interfaces
        │   ├── action
        │   │   └── MoveRobot.action
        │   ├── CMakeLists.txt
        │   ├── msg
        │   │   └── AudioData.msg
        │   ├── package.xml
        │   └── srv
        │       ├── CaptureImage.srv
        │       ├── DetectObject.srv
        │       ├── ParseCommand.srv
        │       └── TranscribeAudio.srv
        ├── README.md
        └── requirements.txt

31 directories, 102 files
```
