from setuptools import find_packages, setup

package_name = 'bdrobot_zone_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuyouling',
    maintainer_email='yuyouling@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bdrobot_zone_planner = bdrobot_zone_planner.bdrobot_zone_planner:main',
            'online_map_server = bdrobot_zone_planner.online_map_server:main',
        ],
    },
)
