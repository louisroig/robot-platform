from setuptools import find_packages, setup

package_name = 'drone_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/drone_mission.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Louis Roig',
    maintainer_email='louisroig@gmail.com',
    description='drone_mission_coordinator: action server + state machine for the M3 mapping mission cycle.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_mission_coordinator = drone_mission.drone_mission_coordinator:main',
        ],
    },
)
