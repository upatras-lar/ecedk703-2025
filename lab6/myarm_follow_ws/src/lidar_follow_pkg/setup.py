import os
from glob import glob
from setuptools import setup, find_packages


package_name = 'lidar_follow_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config', 'rviz'), glob(os.path.join('config', 'rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.dae'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Grog Strongjaw',
    maintainer_email='grogstrongjaw@vm.com',
    description='This is the final lab!',
    license='Yep, here is my licence',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f"radar_node = {package_name}.radar_node:main",
            f"myarm_node = {package_name}.myarm_node:main",
        ],
    },
)
