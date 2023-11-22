from setuptools import find_packages, setup

package_name = 'ros2_cam_if'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karoline',
    maintainer_email='karoline@heiwolt.de',
    description='This package is ROS wrapper, containing a remote control and I/O interface for a Canon EOS R5 C camera',
    license='MIT License, Copyright 2023 Karoline Heiwolt',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = ros2_cam_if.my_node:main'
        ],
    },
)
