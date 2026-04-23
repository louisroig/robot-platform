from setuptools import find_packages, setup

package_name = 'platform_hal'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/platform_hal.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Louis Roig',
    maintainer_email='louisroig@gmail.com',
    description='HAL nodes: motor_driver, imu_driver, safety_monitor.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver = platform_hal.motor_driver:main',
            'imu_driver = platform_hal.imu_driver:main',
            'safety_monitor = platform_hal.safety_monitor:main',
        ],
    },
)
