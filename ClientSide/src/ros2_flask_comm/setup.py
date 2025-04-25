from setuptools import setup

package_name = 'ros2_flask_comm'

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
    maintainer='aathil',
    maintainer_email='aathil@example.com',
    description='ROS2 <-> Flask bridge',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'blender_client = ros2_flask_comm.blender_client:main',
        ],
    },
)

