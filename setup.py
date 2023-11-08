from setuptools import setup

package_name = 'qr_navigation_2'
submodules ="qr_navigation_2/submodules"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iker',
    maintainer_email='iker@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['followGPS=qr_navigation_2.follow_gps:main','test_gps=qr_navigation_2.test_listener_gps:main','pub=qr_navigation_2.test_publisher:main','qos=qr_navigation_2.test_qos:main','imu=qr_navigation_2.test_imu:main','sus=qr_navigation_2.test_subscriber:main','controller=qr_navigation_2.node_controller:main'
        ],
    },
)
