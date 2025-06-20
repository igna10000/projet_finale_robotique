import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Ruta al directorio del paquete 'ex3'
    pkg_ex3 = get_package_share_directory('ex3')

    # Construir rutas absolutas a los archivos YAML
    usb_cam_config = os.path.join(pkg_ex3, 'config', 'usb_cam_calibration.yaml')
    
    # Ruta del archivo de parámetros de ArUco
    aruco_config = os.path.join(pkg_ex3, 'config', 'aruco_parameters.yaml')

    # Rutas de los archivos de lanzamiento dentro del paquete 'ros2_aruco'
    aruco_launch = os.path.join(get_package_share_directory('ros2_aruco'), 'launch', 'aruco_recognition.launch.py')

    # Retornar la LaunchDescription
    return LaunchDescription([
        # Nodo para la cámara USB
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            output='screen',
            parameters=[usb_cam_config],
            remappings=[('/usb_cam/image_raw', '/camera/image_raw')]
        ),
        
        # Incluir el archivo de lanzamiento de ArUco
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(aruco_launch),
            launch_arguments={'param_file': aruco_config}.items()
        ),
    ])

