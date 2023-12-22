from setuptools import setup

package_name = 'ros2bag_to_csv'
submodules = "ros2bag_to_csv/include"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'ros2bag_to_csv=ros2bag_to_csv.ros2bag_to_csv:main'
        ],
    },
)
