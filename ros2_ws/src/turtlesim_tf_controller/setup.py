from setuptools import find_packages, setup

package_name = 'turtlesim_tf_controller'

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
    maintainer='Nomura',
    maintainer_email='s2311074@u.tsukuba.ac.jp',
    description='Set a target point by pressing a button, and the turtle will move there.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'button_control = turtlesim_tf_controller.button_control:main',
            'turtlesim_broadcaster = turtlesim_tf_controller.broadcaster:main',
            'turtle_tf2_frame_listener = turtlesim_tf_controller.listener:main',
        ],
    },
)
