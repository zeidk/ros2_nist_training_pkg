from setuptools import setup

package_name = 'interface_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zeid',
    maintainer_email='zeid.kootbally@nist.gov',
    description='Test custom interfaces and multi threaded executor',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'weather_demo_exe = interface_demo.weather_interface:main',
            'clients_demo_exe = interface_demo.client_server_interface:clients_main',
            'server_demo_exe = interface_demo.client_server_interface:server_main',
            'callback_group_demo_exe = interface_demo.callback_group_interface:main',
        ],
    },
)
