import os
import glob

from setuptools import setup

package_name = 'parameter_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # install launch files
        (os.path.join('share', package_name), glob.glob('launch/*.launch.py')),
        # install parameter files
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zeid',
    maintainer_email='zeidk@umd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'parameter_demo_exe = parameter_demo.parameter_demo_interface:main'
        ],
    },
)
