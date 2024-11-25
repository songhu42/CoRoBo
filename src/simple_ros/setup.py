import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'simple_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.py'))), 
        ('share/' + package_name + '/param', glob(os.path.join('param', '*.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aa',
    maintainer_email='freshmea@naver.com',
    description='simple_ros demo',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "hello = simple_ros.hello:main",
            "hello_class = simple_ros.hello_class:main",
            "hello_sub = simple_ros.hello_sub:main",
            "hello_pub = simple_ros.hello_pub:main",
            "time_pub = simple_ros.time_pub:main",
            "int_service_server = simple_ros.int_service_server:main",
            "service_server = simple_ros.service_server:main",
            "service_client = simple_ros.service_client:main",
            "simple_param = simple_ros.simple_param:main",
            "simple_param2 = simple_ros.simple_param2:main",
            "user_inter_pub = simple_ros.user_inter_pub:main",
            "move_turtle = simple_ros.move_turtle:main",
            "move_turtle_time = simple_ros.move_turtle_time:main", 
            "move_turtle_ns = simple_ros.move_turtle_ns:main",
            "move_turtle_time_ns = simple_ros.move_turtle_time_ns:main",
            "action_server = simple_ros.action_server:main",
            "action_client = simple_ros.action_client:main", 
            "logging_exam = simple_ros.logging_exam:main",
            "deadline = simple_ros.deadline:main", 
            "lifespan = simple_ros.lifespan:main", 
            "liveliness = simple_ros.liveliness:main"

            ],
    },
)
