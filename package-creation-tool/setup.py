from setuptools import find_packages
from setuptools import setup

package_name = 'rclnodejs_pkg_creation_tool'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test', 'scripts']),
    data_files=[
        ('share/ament_index/resource_index/packages',
           ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['ros2cli'],
    zip_safe=True,
    maintainer='Wayne Parrott',
    maintainer_email='5588978+wayneparrott@users.noreply.github.com',
    description='The pkg command for ROS 2 command line tools.',
    long_description="""\
The package provides the pkg command for the ROS 2 command line tools.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2pkg.verb': [
            'create_nodejs = rclnodejs_pkg_creation_tool.verb.create_nodejs_pkg:CreateROS2NodeJsPkgVerb',
         ]
    },
    package_data={
        'rclnodejs_pkg_creation_tool': [
            'resource/**/*', 'resource/**/**/*', 'resource/**/**/**/*',
        ],
    },
)
