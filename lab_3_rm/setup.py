from setuptools import find_packages, setup

package_name = 'lab_3_rm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/lab_3_rm/launch', ['launch/mapa_del_ambiente.xml']),

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
            'procesar_mapa = lab_3_rm.procesar_mapa:main',
            'leer_lidar = lab_3_rm.leer_lidar:main',
            "nodo_intento = lab_3_rm.nodo_de_intento:main",

        ],
    },
)
