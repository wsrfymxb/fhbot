from setuptools import setup  
import os  
from glob import glob  
  
package_name = 'turtlebot3_auto_manipulation'  
  
setup(  
    name=package_name,  
    version='0.0.0',  
    packages=[package_name],  
    data_files=[  
        ('share/ament_index/resource_index/packages',  
            ['resource/' + package_name]),  
        ('share/' + package_name, ['package.xml']),  
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),  
    ],  
    install_requires=['setuptools'],  
    zip_safe=True,  
    maintainer='your_name',  
    maintainer_email='your_email@example.com',  
    description='TurtleBot3 自动化机械臂操作',  
    license='Apache License 2.0',  
    tests_require=['pytest'],  
    entry_points={  
        'console_scripts': [  
            'auto_controller = turtlebot3_auto_manipulation.auto_controller:main',  
        ],  
    },  
)