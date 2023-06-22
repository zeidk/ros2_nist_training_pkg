import os 
import glob
from setuptools import setup

package_name = 'first_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob.glob('launch/*.launch.py')),
        
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
            'simple_node_exe = first_package.simple_node:main',
            'minimal_publisher_exe = first_package.minimal_publisher:main',
            'advanced_publisher_exe = first_package.advanced_publisher:main',
            'subscriber_exe = first_package.subscriber:main'
        ],
    },
)
