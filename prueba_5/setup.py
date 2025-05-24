from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'prueba_5'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*.xml')) + glob(os.path.join('launch', '*.py'))),  # Incluye archivos .py
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='paulo',
    maintainer_email='paulo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "dead_reckoning = prueba_5.dead_reckoning_nav_original:main",
            "pose_loader = prueba_5.pose_loader:main",
            "obstacle_detector = prueba_5.obstacle_detector:main",
            "dead_reckoning_p2 = prueba_5.dead_reckoning_nav_parte_2:main",
            "teleop_turtlebot = prueba_5.teleop_turtlebot:main",
            "super_nodo_1 = prueba_5.super_nodo_1:main",
        ],
    },
)
