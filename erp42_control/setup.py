import os
from setuptools import setup
from glob import glob

package_name = 'erp42_control'
package_name_ = 'autoware_planning_msgs'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]), 
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name+'/stanley.py']),
        (os.path.join('share', package_name_, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='gjs',
    maintainer_email='junseonggg2001@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_opener = erp42_control.path_opener:main',
            'controller_2024 = erp42_control.controller_2024:main',
            'pid_tunning = erp42_control.pid_tunning:main',
            'controller_autoware = erp42_control.controller_autoware:main',
            'encoder_imu = erp42_control.encoder_imu:main',
            'topic_name_changer = erp42_control.topic_name_changer:main',
            'bezier_curve_making = erp42_control.bezier_curve_making:main',
            'controller_obstacle = erp42_control.controller_obstacle:main',
            'control_parking = erp42_control.control_parking:main',
            'test = erp42_control.test:main',
        ],
    },
)
