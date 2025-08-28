from setuptools import find_packages, setup

package_name = 'tgt_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tgt_filter.launch.py']),  # 변동점: launch 포함
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaeseok',
    maintainer_email='jaeseok04260@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 변동점: 실행 엔트리
            'tgt_cmd_filter = tgt_nav.tgt_cmd_filter:main',
        ],
    },
)
