import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'event_bridge_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=[
        'setuptools',
        'event_camera_msgs',
        'event_camera_codecs',
        'event_camera_legacy_tools',
        'libcaer_driver',
        'event_camera_tools',
        'event_camera_renderer',
    ],
    zip_safe=True,
    maintainer='taehun-ryu',
    maintainer_email='xogns2079@gmail.com',
    description='ROS2 Python bridge package depending on all event camera submodule packages.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
