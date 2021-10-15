from setuptools import setup

package_name = 'hanbi_control'

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
            'path_pub = hanbi_control.path_pub:main',
            'path_via = hanbi_control.path_via:main',
            'a_star = hanbi_control.a_star:main',
            'path_tracking_detected = hanbi_control.path_tracking_detected:main',
            'path_tracking_kjh = hanbi_control.path_tracking_kjh:main',
            'odom = hanbi_control.odom:main'
        ],
    },
)
