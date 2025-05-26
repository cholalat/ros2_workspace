from setuptools import find_packages, setup

package_name = 'lab_2_rm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/lab_2_rm/launch', ['launch/navegacion_pasillo.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='paulo',
    maintainer_email='paulo.oses@uc.cl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'seguir_pasillo = lab_2_rm.seguir_pasillo:main',
            'controlador_PID = lab_2_rm.controlador_PID:main',
            "pose_loader = lab_2_rm.pose_loader:main",
        ],
    },
)
