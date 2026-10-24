from glob import glob
from setuptools import setup

PACKAGE = 'vt_bringup'

setup(
    name=PACKAGE,
    version='0.1.0',
    packages=[PACKAGE],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml']),
        ('share/' + PACKAGE + '/launch', glob('launch/*.launch.py')),
        ('share/' + PACKAGE + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chilinghan',
    maintainer_email='chilinghan2007@gmail.com',
    description='Launch files and config for the vector-tern sim and onboard stack.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            
        ],
    },
)
