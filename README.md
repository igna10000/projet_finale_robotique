
# Projet Finale Robotique 🔥🤖

Este proyecto implementa un sistema robótico basado en ROS 2 y una Raspberry Pi, que permite la conexión con una ESP32 para control de motores, activación de una bomba, transmisión de video desde una cámara USB y detección e inferencia de fuego mediante un modelo YOLOv5.

---

## 🔧 Requisitos previos

Antes de ejecutar el sistema, asegúrate de tener instaladas las siguientes **dependencias**:

### Paquetes ROS 2 necesarios

- `vision_msgs`
- `usb_cam`

### Otros requerimientos

- `yolov5` (con modelo personalizado `yolov5s_best.pt`)
- Python ≥ 3.8
- PyTorch
- OpenCV

---

## 🔌 Conexión SSH con la Raspberry Pi

Conéctate a la Raspberry Pi mediante SSH:

```bash
ssh seeker@robot.local
```

- Contraseña: `seeker`
- Una vez conectado, deberías ver algo como esto:

```
seeker@robot:~$
```

---

## 📁 Navegación y códigos fuente

Puedes revisar los archivos fuente más importantes del proyecto con `nano`:

### 🔸 Intérprete de comandos UDP para motores (ESP32)

```bash
nano proyecto_robotica/src/udp_cmd_vel_interpreter/udp_cmd_vel_interpreter/cmd_vel_udp_interpreter.py
```

### 🔸 Intérprete de activación de la bomba

```bash
nano proyecto_robotica/src/udp_cmd_vel_interpreter/udp_cmd_vel_interpreter/bool_udp_interpreter.py
```

### 🔸 Nodo de cámara USB (publica `/image_raw` y `/camera_info`)

```bash
nano proyecto_robotica/src/usb_cam/src/usb_cam_node.cpp
```

### 🔸 Nodo de inferencia de fuego (publica `/fire_detections`)

```bash
nano proyecto_robotica/src/fire_detection/fire_detection/fire_detector_node.py
```

### 🔸 Nodo de seguimiento de fuego

```bash
nano proyecto_robotica/src/fire_tracking/fire_tracking/fire_tracker.py
```

---

## 🚀 Ejecución del sistema

> Se requieren **5 terminales** abiertas por SSH. Cada terminal debe ejecutar una parte del sistema.

### 🛠 Configuración inicial para todas las terminales

```bash
ssh seeker@robot.local
cd proyecto_robotica/
source install/setup.bash
```

---

### ▶️ Terminal 1 – Intérprete de comandos UDP

```bash
ros2 run udp_cmd_vel_interpreter cmd_vel_udp_interpreter
```

### 💧 Terminal 2 – Activación de bomba

```bash
ros2 run udp_cmd_vel_interpreter bomba
```

### 📷 Terminal 3 – Nodo de cámara USB

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args -p width:="640" -p height:="480" -p framerate:="25.0"
```

### 🔥 Terminal 4 – Nodo de detección de fuego

```bash
ros2 run fire_detection fire_detector --ros-args -p conf_thres:=0.2
```

### 🧭 Terminal 5 – Nodo de seguimiento de fuego

```bash
ros2 run fire_tracking fire_tracker --ros-args -p image_width:=600 -p kp:=0.0007 -p kd:=0.0006
```

---

## 📡 Visualización en tiempo real (opcional – desde PC local)

Si deseas ver la inferencia en vivo, abre una terminal local y ejecuta:

```bash
cd yolov5/
python3 detect.py --source 0 --weights yolov5s_best.pt --conf 0.2 --img 50
```

---

## 📊 Visualización de nodos ROS

Para ver cómo interactúan los nodos entre sí, puedes ejecutar:

```bash
rqt_graph
```

---

## ⚠️ Apagado seguro de la Raspberry Pi

Siempre apaga el sistema correctamente con:

```bash
sudo shutdown now
```

---

## 📦 Dependencias

Este proyecto requiere los siguientes paquetes y herramientas:

### ROS 2 Packages

- [`vision_msgs`](https://github.com/ros-perception/vision_msgs)
- [`usb_cam`](https://github.com/ros-drivers/usb_cam)

### Otros

- [`yolov5`](https://github.com/ultralytics/yolov5) (con modelo `yolov5s_best.pt`)
- Python 3.8+
- PyTorch
- OpenCV

---

## 👤 Autor

**Ignacio Sebastián Coriza Rondo**  
Estudiante de Ingeniería Mecatrónica  
Universidad Católica Boliviana
