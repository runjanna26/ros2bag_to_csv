from setuptools import setup
import os
from glob import glob

package_name = 'ros2bag_to_csv'
submodules = "ros2bag_to_csv/include"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='runj',
    maintainer_email='run.janna@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2bag_to_csv=ros2bag_to_csv.ros2bag_to_csv:main',
            'csv_to_json_ros1=ros2bag_to_csv.csv_to_json_ros1:main',
            'csv_to_json_ros2=ros2bag_to_csv.csv_to_json_ros2:main',
            'ros2bag_to_vdo_compressed=ros2bag_to_csv.ros2bag_to_vdo_compressed:main'

        ],
    },
)
