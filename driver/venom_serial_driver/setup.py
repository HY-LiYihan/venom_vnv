from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'venom_serial_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='venom',
    maintainer_email='liyihan.xyz@gmail.com',
    description='Serial driver for DJI C-board communication',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_node = venom_serial_driver.serial_node:main',
        ],
    },
)
