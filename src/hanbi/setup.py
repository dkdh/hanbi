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
            'a_star_local_path = a_star_local_path:main',
            'a_star = a_star:main',
            'client = client:main',
            'load_map = load_map:main',
            'make_path = make_path:main',
            'odom = odom:main',
            'path_pub = path_pub:main',
            'path_tracking_patrol = path_tracking_patrol:main',
            'perception = perception:main',
            'pytorch_detector = pytorch_detector:main',
            'transform = transform:main'
        ],
    },
)
