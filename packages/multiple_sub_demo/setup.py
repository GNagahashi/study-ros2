import os
from glob import glob
from setuptools import setup

package_name = 'multiple_sub_demo'

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
    description='Demo of a single node handling multiple subscriptions',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_a = multiple_sub_demo.dummy_sensor_a:main',
            'sensor_b = multiple_sub_demo.dummy_sensor_b:main',
            'multiple_sub = multiple_sub_demo.multiple_sub:main',
        ],
    },
)
