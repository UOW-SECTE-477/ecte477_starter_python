from setuptools import setup
import os
from glob import glob

package_name = 'ecte477_starter_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][xma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('rviz', '*.rviz')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeffm',
    maintainer_email='jeffm@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'command_server = ecte477_starter_python.command_server_node:main',
		'wall_follower = ecte477_starter_python.wall_follower_node:main',
        'maze_explorer = ecte477_starter_python.maze_explorer:main',
        'maze_explorer3 = ecte477_starter_python.maze_explorer3:main'
        ],
    },
)
