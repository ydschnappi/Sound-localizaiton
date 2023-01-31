from setuptools import setup

package_name = 'Lscan'

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
    maintainer='kaust',
    maintainer_email='kaustabh.paul@tum.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = Lscan.Measure2:main',
            'wanderer = Lscan.Explore2:main',
            'checkcol = Lscan.CollisionCheck:main'
        ],
    },
)
