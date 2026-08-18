from setuptools import find_packages, setup

package_name = 'bot_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bot_vision.launch.py']),
        ('share/' + package_name + '/config', ['config/bot_vision_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='bramantya.25@intl.zju.edu.cn',
    description='bot_vision ROS2 package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector = bot_vision.yolo_detector:main',
            'target_selector = bot_vision.target_selector:main',
            'tracker_node = bot_vision.tracker_node:main',
            'follow_controller = bot_vision.follow_controller:main',
        ],
    },
)
