from setuptools import setup

package_name = 'robomove_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hamza',
    maintainer_email='hamza@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_node = robomove_pkg.robot_node:main',
            'lidar_obstacle_avoidance = robomove_pkg.lidar_obstacle_avoidance:main',
            'teleop_keyboard = robomove_pkg.teleop_keyboard:main'
            
        ],
    },
)
