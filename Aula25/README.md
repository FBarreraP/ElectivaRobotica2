<h1>Aula 25</h1>

Esta clase consiste en realizar navegación con el LoCoBot con el control del PS4 y el simulador Rviz a través de los paquetes de ROS1 para este robot

<h2>Ejemplo de navegación con control PS4 y Rviz con el LoCoBot</h2>

En una terminal conectada al locobot iniciar el nodo de navegación, a través del siguiente comando:
```
roslaunch interbotix_xslocobot_nav xslocobot_nav.launch robot_model:=locobot_px100 rtabmap_args:=-d
```

Abrir en la consola del computador el simulador Rviz, a través del siguiente comando:

```
roslaunch interbotix_xslocobot_descriptions remote_view.launch rviz_frame:=map
```

En otra consola conectada al locobot iniciar el nodo de Joystick, a través del siguiente comando:

```
roslaunch interbotix_xslocobot_joy xslocobot_joy.launch robot_model:=locobot_px100 launch_driver:=false
```

<h2>Paquete de Python para el Locobot<h2>

Los ejemplos demos en Python para el LoCoBot PX100 se encuentran <a href="https://docs.trossenrobotics.com/interbotix_xslocobots_docs/ros1_packages/python_demos.html">aquí</a> y se encuentran instalados en la siguiente ruta:

```
~/interbotix_ws/src/interbotix_ros_rovers/interbotix_ros_xslocobots/examples/python_demos
```

La información sobre los paquetes de Python en ROS1 para el LoCoBot PX100 se encuentran <a href="https://docs.trossenrobotics.com/interbotix_xslocobots_docs/python_interface.html">aquí</a>

Lanzar del nodo de control y python en una terminal del locobot a través del siguiente comando:

```
roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_px100
```

En otra terminal del locobot ejecutar el script de python: Ejemplo de mover la camara a través del siguiente comando:

```
python3 pan_tilt_control_px100.py (tener en cuenta que se debe posicionar en el directorio de python_demos de los pasos anteriores
```