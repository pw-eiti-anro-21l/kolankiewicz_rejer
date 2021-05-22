import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'anro_lab5_pd'

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
        'ointService = anro_lab5_pd.oint:main',
        'ointClient = anro_lab5_pd.oint_client:main',
        'ikinPublisher = anro_lab5_pd.ikin:main'
        ],
    },
)
