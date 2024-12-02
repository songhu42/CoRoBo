import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'corobo_py'

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
    maintainer='song',
    maintainer_email='songhu42@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "action_server = corobo_py.action_server:main",
            "action_client = corobo_py.action_client:main",
            "crbm_center = corobo_py.crbm_center:main",
            "crbs_server = corobo_py.crbs_server:main",
            "crbs_mani = corobo_py.crbs_mani:main",
            "patrol_manipulator = corobo_py.patrol_manipulator:main",
            "teleop_keyboard = corobo_py.teleop_keyboard:main",
            
        ],
    },
)
