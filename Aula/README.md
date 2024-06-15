<h1>Introducción a ROS</h1>

En esta clase se presenta un introducción a ROS

<h2>ROS</h2>

ROS es un sistema operativo de robots creado en el 2007 en el laboratorio de Inteligencia Artificial en la Universidad de Stanford. Actualmente, es una colección de frameworks, herramientas (visualizar datos, guardar datos sensores, depurar y simular) y librerías. Una de las principales justificaciones de ROS es crear proyectos complejos de robótica con alta compatibilidad con hardware (Raspberry, PC(Ubuntu)).

<h2>Instalación de Ubuntu 20.04</h2>

Descargar ubuntu 20.04 y Virtual Box

Crear la máquina virtual con Ubuntu 20.04 de formato .iso en el Virtual Box

<h2>Instalación de ROS Noetic</h2>

https://wiki.ros.org/noetic/Installation/Ubuntu

Abrir una terminal en Ubuntu y conceder permisos de super usuario (administrador)

```
su root
```

Inicialemnte se debe configurar el computador para que acepte software de packages.ros.org, por tanto, en un terminal en Ubuntu correr los siguientes comandos:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

sudo apt install ros-noetic-desktop-full

source /opt/ros/noetic/setup.bash

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

source ~/.bashrc

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

sudo apt install python3-rosdep

sudo rosdep init

rosdep update
```

Para validar la instalación de ROS, ejecutar los siguientes comandos en terminales independientes:

```
roscore
```

```
rosrun roscpp_tutorials talker
```


<h2>Conceptos ROS</h2>

![ROS Master and Nodes](image.png)

Fuente: https://robodev.blog/ros-basic-concepts


![ROS Topic and Message](image-1.png)

Fuente: https://robodev.blog/ros-basic-concepts


![ROS service](image-2.png)

Fuente: https://robodev.blog/ros-basic-concepts


<h2>Comandos ROS</h2>

ROS master

```
roscore
```