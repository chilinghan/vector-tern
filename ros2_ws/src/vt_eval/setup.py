from setuptools import setup

PACKAGE = 'vt_eval'

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
    description='Trajectory recording for VSLAM evaluation against Gazebo ground truth.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_recorder = vt_eval.trajectory_recorder:main'
        ],
    },
)
