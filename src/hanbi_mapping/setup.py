from setuptools import setup

package_name = 'hanbi_mapping'

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
            'run_mapping_custom = hanbi_mapping.run_mapping_custom:main',
            'socket_custom = hanbi_mapping.socket_custom:main',
            'client = hanbi_mapping.client:main'
        ],
    },
)
