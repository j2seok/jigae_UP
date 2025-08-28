from setuptools import setup

package_name = 'static_tf_broadcaster'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/static_tf_launch.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaeseok',
    maintainer_email='your@email.com',
    description='Static TF broadcaster for base_link, laser, odom',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'static_tf_node = static_tf_broadcaster.static_tf_node:main',
        ],
    },
)
