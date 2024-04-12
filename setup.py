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
        'console_scripts': ['deteccion=qr_navigation_2.deteccion:main','followGPS=qr_navigation_2.followgps5:main','test_gps=qr_navigation_2.test_listener_gps:main','pub=qr_navigation_2.test_publisher:main','qos=qr_navigation_2.test_qos:main','imu=qr_navigation_2.test_imu:main','sus=qr_navigation_2.test_subscriber:main','controller=qr_navigation_2.node_controller:main'
                            ,'web=qr_navigation_2.fake_web:main',
                            'center_approach=qr_navigation_2.center_and_approach_test:main',
                            'orange_detection=qr_navigation_2.orange_ros2:main',
                            'bottle=qr_navigation_2.bottle_detection:main','bottle_ros2=qr_navigation_2.bottle_detection_test:main',
                            'node_detection_ros2=qr_navigation_2.node_detection:main',
                            'web_inter = qr_navigation_2.intermediary_web:main',
                            'searching=qr_navigation_2.searching:main'
        ],
    },
)
