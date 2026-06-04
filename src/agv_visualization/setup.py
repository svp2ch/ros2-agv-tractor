from setuptools import setup
import os
from glob import glob

package_name = 'agv_visualization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dan',
    maintainer_email='danik280702@gmail.com',
    description='Конфигурация RViz2 для визуализации беспилотного трактора',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
