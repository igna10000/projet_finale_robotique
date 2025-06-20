from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ex3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Esto es necesario para que ROS 2 reconozca el paquete
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        # Esto copia el package.xml
        ('share/' + package_name, ['package.xml']),

        # Esto copia los archivos de launch al share del paquete
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # Esto copia los archivos de configuración YAML
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seeker',
    maintainer_email='seeker@todo.todo',
    description='Nodo ROS 2 para detección de ArUco con cámara USB',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Aquí puedes agregar tus nodos en Python más adelante
            # 'nombre_ejecutable = paquete.modulo:funcion_main'
            # Ejemplo:
            'aruco_track = ex3.eje3:main'
        ],
    },
)

