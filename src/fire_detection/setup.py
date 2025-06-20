from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'fire_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['fire_detection', 'fire_detection.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Incluir todos los archivos dentro de fire_detection/yolov5 (por ejemplo, modelos .pt y demás)
        (os.path.join('share', package_name, 'yolov5'),
         glob(os.path.join('fire_detection', 'yolov5', '**', '*.*'), recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tu_usuario',
    maintainer_email='tu@email.com',
    description='Detección de fuego con YOLOv5',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fire_detector = fire_detection.fire_detector_node:main',
        ],
    },
)

