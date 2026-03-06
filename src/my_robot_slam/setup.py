from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # --- 核心：搬运 launch 文件 ---
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # --- 核心：搬运 config 文件夹里的 yaml 和 rviz 文件 ---
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wyi',
    maintainer_email='wangyi08234510@outlook.com',
    description='SLAM package for my robot',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
