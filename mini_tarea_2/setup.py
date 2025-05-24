from setuptools import find_packages, setup

package_name = 'mini_tarea_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch.xml']),  # Asegúrate de incluir tu archivo launch aquí

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
            'controlador = mini_tarea_2.controlador_pid:main',
            'virtual_robot = mini_tarea_2.virtual_robot:main',
        ],
    },
)
