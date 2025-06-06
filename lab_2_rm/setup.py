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
        ('share/lab_2_rm/launch', ['launch/navegacion_cuadrado_perfecto.xml']),
        ('share/lab_2_rm/launch', ['launch/navegacion_pasillo.xml']),
        ('share/lab_2_rm/launch', ['launch/follow_the_carrot_line.xml']),
        ('share/lab_2_rm/launch', ['launch/follow_the_carrot_sin.xml']),
        ('share/lab_2_rm/launch', ['launch/follow_the_carrot_sqrt.xml']),
        ('share/lab_2_rm/launch', ['launch/follow_the_carrot_paralelepipedo.xml']),
        ('share/lab_2_rm/launch', ['launch/avanzar_y_rotar_ctrl.xml']),
        ('share/lab_2_rm/launch', ['launch/run_all.xml']),


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
            'obstacle_detector = lab_2_rm.parte_2_obstacle_detector:main',
            'sigue_pasillo = lab_2_rm.parte_2_sigue_pasillo:main',
            "trayectory_loader = lab_2_rm.parte_3_trayectory_loader:main",
            "trayectoria = lab_2_rm.parte_3_trayectoria:main",
            "graficador = lab_2_rm.graficador:main",
            "dead_reckoning_nav_original = lab_2_rm.dead_reckoning_nav_original:main",
            "pose_loader = lab_2_rm.pose_loader:main",

        ],
    },
)


