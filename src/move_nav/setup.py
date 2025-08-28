# 변동점: launch/*.py 설치를 위한 glob 임포트
from glob import glob
from setuptools import find_packages, setup

package_name = 'move_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 변동점: launch 디렉터리 내 모든 .py를 설치 경로에 포함
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],  # 변동점: 필요시 pyserial 등 추가 가능
    zip_safe=True,
    maintainer='jaeseok',
    maintainer_email='jaeseok04260@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_imu_ekf = move_nav.encoder_imu_ekf:main'
        ],
    },
)
