from setuptools import find_packages, setup

package_name = 'teleop_web'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/teleop_web.launch.py']),
        ('share/' + package_name + '/web', ['web/index.html']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Louis Roig',
    maintainer_email='louisroig@gmail.com',
    description='M1 browser joystick publishing /hal/cmd_vel_raw via rosbridge.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
