import os
from glob import glob
from setuptools import setup

package_name = 'interactive_transform_publisher'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
        (os.path.join('share', package_name), glob('rviz/*')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Alberto Cruz',
    author_email='alberto.cruz@upm.es',
    maintainer='Alberto Cruz',
    maintainer_email='alberto.cruz@upm.es',
    description='ROS2 package for creating interactive markers and their associated transformations frames.',
    license='GNU General Public License v3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interactive_transform_publisher = interactive_transform_publisher.interactive_transform_publisher:main'
        ],
    },
)
