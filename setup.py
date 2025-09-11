from setuptools import setup

package_name = 'rqt_virtual_joystick'

setup(
    name=package_name,
    version='0.1.0',
    package_dir={'': 'src'},
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}', ['plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Abdelrahman Mahmoud',
    maintainer='Abdelrahman Mahmoud',
    maintainer_email='abdulrahman.mahmoud1995@gmail.com',
    keywords=['ROS2', 'rqt', 'joystick', 'twist', 'cmd_vel', 'teleop'],
    description='RQt plugin that simulates joystick input and publishes sensor_msgs/Joy and geometry_msgs/Twist.',
    license='BSD',
    entry_points={
        'console_scripts': [
            'rqt_virtual_joystick = ' + package_name + '.main:main',
        ],
    },
)
