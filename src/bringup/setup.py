from setuptools import find_packages, setup

package_name = 'bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
        ('share/' + package_name + '/config', ['config/bringup_params.yaml']),
        ('share/' + package_name + '/rviz', ['rviz/bringup.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='bramantya.25@intl.zju.edu.cn',
    description='bringup ROS2 package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bringup_node = bringup.bringup_node:main',
        ],
    },
)
