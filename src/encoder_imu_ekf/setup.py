# setup.py
from setuptools import setup

package_name = 'encoder_imu_ekf'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # 변동점: ament index 리소스 등록 (ros2가 패키지 탐색 가능)
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),  # 변동점
        # 변동점: package.xml을 표준 위치에도 설치
        ('share/' + package_name, ['package.xml']),                                   # 변동점
        # 런치/설정 파일 설치
        ('share/' + package_name + '/launch', ['launch/ekf_with_serial.launch.py']),
        ('share/' + package_name + '/config', ['config/ekf.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaeseok',              # 변동점: 실제 이름으로
    maintainer_email='you@example.com',
    description='Serial IMU + Encoder fusion (robot_localization EKF)',
    license='MIT',
    entry_points={
        'console_scripts': [
            # ros2 run encoder_imu_ekf serial_imu_encoder_node
            'encoder_imu_ekf = encoder_imu_ekf.serial_imu_encoder_node:main',
        ],
    },
)
