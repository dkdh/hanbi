from setuptools import setup

package_name = 'hanbi'

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
    maintainer='try61',
    maintainer_email='try615@naver.com',
    description='Hanbi Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'a_star_local_path = hanbi.a_star_local_path:main',
            'a_star = hanbi.a_star:main',
            'client = hanbi.client:main',
            'load_map = hanbi.load_map:main',
            'make_path = hanbi.make_path:main',
            'odom = hanbi.odom:main',
            'path_pub = hanbi.path_pub:main',
            'path_tracking_patrol = hanbi.path_tracking_patrol:main',
            'perception = hanbi.perception:main',
            'pytorch_detector = hanbi.pytorch_detector:main',
            'socket_custom = hanbi.socket_custom:main',
            'run_mapping_custom = hanbi.run_mapping_custom:main'
        ],
    },
)
