
import os
from glob import glob
from setuptools import setup

package_name = 'gate_detection'


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fatma',
    maintainer_email='fatmaaboeldahab1998@gmail.com',
    description='this is a gate_detection package that contain server and client nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'service = gate_detection.server_node:main',
        'client = gate_detection.client_node:main',
       
    ],
},
)





