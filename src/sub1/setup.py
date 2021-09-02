from setuptools import setup

package_name = 'sub1'

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
    maintainer='Administrator',
    maintainer_email='bure5kzam@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = sub1.publisher:main',
            'subscriber = sub1.subscriber:main',
            'perception = sub1.perception:main',
            'odom = sub1.odom:main',
            'controller = sub1.controller:main',
            'handcontrol = sub1.handcontrol:main',
            'make_pub = sub1.make_pub:main',
            'path_pub = sub1.path_pub:main',
            'path_tracking = sub1.path_tracking:main',
        ],
    },
)
