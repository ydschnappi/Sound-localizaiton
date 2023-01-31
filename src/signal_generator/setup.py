from setuptools import setup
import os
from glob import glob

package_name = 'signal_generator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('maps/*pgm')),
        (os.path.join('share', package_name), glob('maps/*yaml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yan',
    maintainer_email='yan@todo.todo',
    description='signal generator',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'signal_pose_node=signal_generator.signal_pose:main',
            'predefind_signal_node=signal_generator.predefined_signal_pose:main',
            'signal_publisher_node=signal_generator.signal_publisher:main',
        ],
    },
)
