# projet_finale_robotique


#primero para todo:
#conectar a raspberry por ssh:
ssh seeker@robot.local
#la contraseña es: seeker
#debe de salir asi en la terminal => seeker@robot:~$


#Para ver los codigos (para salir es control + x):

#codigo interprete para conectarse a los motores (a la esp32)
nano proyecto_robotica/src/udp_cmd_vel_interpreter/udp_cmd_vel_interpreter/cmd_vel_udp_interpreter.py

#Para el codigo interprete de la bomba
nano proyecto_robotica/src/udp_cmd_vel_interpreter/udp_cmd_vel_interpreter/bool_udp_interpreter.py

#El codigo de iniciar nodo de camara para publicar la imagen y parametros calibrados de la camara (/image_raw  /camera_info)
nano proyecto_robotica/src/usb_cam/src/usb_cam_node.cpp


#Codigo importante para la inferencia (/fire_detections)
nano proyecto_robotica/src/fire_detection/fire_detection/fire_detector_node.py



nano proyecto_robotica/src/fire_tracking/fire_tracking/fire_tracker.py





###EJECUTAR TODO
#PASO 1
#Se necesita 5 terminales(divididas al mismo tiempo), para cada una poner lo siguiente:
#conectar a raspberry por ssh:
ssh seeker@robot.local
#la contraseña es: seeker
#debe de salir asi en la terminal => seeker@robot:~$
cd proyecto_robotica/
source install/setup.bash


#PASO 2
#Terminal 1
ros2 run udp_cmd_vel_interpreter cmd_vel_udp_interpreter

#PASO 3
#Terminal 2
ros2 run udp_cmd_vel_interpreter bomba 

#PASO 4
#Terminal 3
ros2 run usb_cam usb_cam_node_exe --ros-args   -p width:="640"   -p height:="480"   -p framerate:="25.0"

#PASO 5
#Terminal 4
ros2 run fire_detection fire_detector --ros-args -p conf_thres:=0.2

#PASO 6
#Terminal 5
ros2 run fire_tracking fire_tracker --ros-args -p image_width:=600 -p kp:=0.0007 -p kd:=0.0006






##COMO VER LOS NODOS

rqt_graph


##APAGAR SI O SI:

sudo shutdown now





###PARA VER EN TIEMPO REAL LA INFERENCIA
#nueva terminal (en el local)

cd yolov5/

python3 detect.py --source 0 --weights yolov5s_best.pt --conf 0.2 --img 50








###DEPENDENCIAS

vision_msgs
usb_cam
yolov5


