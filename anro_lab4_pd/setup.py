import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'anro_lab4_pd'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*'))
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
        'nonkdl_node = anro_lab4_pd.NONKDL_DKIN:main',
        'kdl_node = anro_lab4_pd.KDL_DKIN:main',
        'service = anro_lab4_pd.jint:main',
        'client = anro_lab4_pd.jint_client:main',
        ],
    },
)
