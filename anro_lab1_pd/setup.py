from setuptools import setup
import os
from glob import glob

package_name = 'anro_lab1_pd'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name), glob("launch/*_launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mickol34',
    maintainer_email='mic.kol34@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "teleop_talker = anro_lab1_pd.custom_teleop:main"
        ],
    },
)
