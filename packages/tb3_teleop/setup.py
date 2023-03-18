import os
from glob import glob
from setuptools import setup

package_name = 'tb3_teleop'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GNagahashi',
    maintainer_email='9bjk1111@mail.u-tokai.ac.jp',
    description='Package for remote control of the TurtleBot3(burger)',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_burger = tb3_teleop.teleop_burger:main',
        ],
    },
)
