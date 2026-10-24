from setuptools import setup

PACKAGE = 'vt_px4_bridge'

setup(
    name=PACKAGE,
    version='0.1.0',
    packages=[PACKAGE],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chilinghan',
    maintainer_email='chilinghan2007@gmail.com',
    description='Forwards VSLAM / ground-truth odometry to PX4 EKF2 as external vision.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_to_px4 = vt_px4_bridge.odom_to_px4:main'
        ],
    },
)
