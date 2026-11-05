from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dht11'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Giacomo Picardi',
    maintainer_email='gpicardi@icm.csic.es',
    description='ROS 2 node to read and publish data from multiple DHT11 temperature and humidity sensors',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dht11_node = dht11.dht11_node:main',
        ],
    },
)