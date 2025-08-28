from setuptools import setup
from glob import glob
import os

package_name = 'pallet_align'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # ament index 패키지 마커 (중요)
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         [os.path.join('resource', package_name)]),
        # 메타/런치
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaeseok',
    maintainer_email='jaeseok04260@gmail.com',
    description='ArUco pallet alignment and serial command sender',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'aruco_detector_node = pallet_align.aruco_detector_node:main',
            'serial_sender_node  = pallet_align.serial_sender_node:main',
        ],
    },
)

