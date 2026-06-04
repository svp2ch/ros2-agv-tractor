from setuptools import setup
import os
from glob import glob

package_name = 'agv_decision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dan',
    maintainer_email='danik280702@gmail.com',
    description='Конечный автомат состояний для беспилотного трактора',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'decision_node = agv_decision.decision_node:main',
        ],
    },
)
