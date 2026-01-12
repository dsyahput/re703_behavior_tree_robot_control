from setuptools import find_packages, setup

package_name = 're703_behavior_tree_robot_control'

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
    maintainer='syahputra',
    maintainer_email='dannisyahputra36@gmail.com',
    description='Behavior Tree–based robot control developed for the RE703 Robot Control course, using BumperBot simulation as the final semester assessment.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'main_bt = re703_behavior_tree_robot_control.main_bt:main',
            'main_cam = re703_behavior_tree_robot_control.main_cam:main',
        ],
    },
)
