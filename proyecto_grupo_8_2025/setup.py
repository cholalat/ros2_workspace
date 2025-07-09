from setuptools import find_packages, setup

package_name = 'proyecto_grupo_8_2025'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/proyecto_grupo_8_2025/launch', ['launch/actividad_1.xml']),
        ('share/proyecto_grupo_8_2025/launch', ['launch/actividad_2.xml']),
        ('share/proyecto_grupo_8_2025/launch', ['launch/actividad_3.xml']),


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
        ],
    },
)
