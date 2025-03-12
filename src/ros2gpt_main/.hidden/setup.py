from setuptools import setup

package_name = 'ros2gpt_main'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['CMakeLists.txt']),  # Include CMakeLists.txt
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs', 'std_srvs', 'sensor_msgs', 'openai', 'ros2gpt_interfaces'],
    zip_safe=False,
    maintainer='yongp',
    maintainer_email='2102088@sit.singaporetech.edu.sg',
    description='Natural language interface for ROS2-controlled robot',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'ros2gpt = ros2gpt_main.ros2gpt_node:main',
            'whisper = ros2gpt_main.whisper_server_node:main',
            'command = ros2gpt_main.commander_node:main',
            'move = ros2gpt_main.tm_move_client:main'
        ],
    },
)