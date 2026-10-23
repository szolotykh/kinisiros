"""Package the ROS 2 driver and its validated Python SDK dependency."""

from setuptools import find_packages, setup

package_name = 'roskinisi'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
         ['package.xml', 'README.md', 'CHANGELOG.rst', 'LICENSE', 'requirements.txt']),
    ],
    install_requires=['setuptools', 'pykinisi>=2.5.0,<3'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Sergey',
    maintainer_email='szolotykh88@gmail.com',
    description='ROS 2 driver for Kinisi motor controllers with timestamped odometry',
    license='MIT',
    entry_points={
        'console_scripts': [
            "kinisi_controller = roskinisi.kinisi_controller:main",
        ],
    },
)
