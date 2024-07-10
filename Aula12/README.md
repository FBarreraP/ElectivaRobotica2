<h1>Aula 12</h1>

En esta clase se presenta un introducción a ROS

<h2>Introducción a ROS</h2>

ROS es un Sistema Operativo de Robots, click para ver las diferentes <a href="https://wiki.ros.org/Distributions">versiones de ROS</a>

 https://wiki.ros.org/Distributions

<h3>Raspberry Pi :atom:</h3>

>[!WARNING]
>En algunas versiones de Raspbian (ej:buster) se presenta el siguiente error posteriormente a la instalación del Raspbian Buster en la Raspberry Pi 4

![Error de boot Raspbian](image-9.png)

Fuente: https://embarcados.com.br/raspberry-pi-corrigindo-problemas-com-o-start4-elf-e-fixup4-dat/

Para solucionar ese error, se deben tener en cuenta los siguientes pasos:

I. Descargar <a href="https://github.com/raspberrypi/firmware/tree/master/boot">aquí</a> los archivos desactualizados: fixup4.dat y start4.elf

II. Reemplazar los dos archivos (fixup4.dat y start4.elf) descargados anteriormente en la carpeta /boot de la SD Card

III. Después de inicializar el sistema operativo, actualizar el sistema operativo a través de los siguientes comandos:
```
sudo apt update
sudo apt full-upgrade
```

La instalación de ROS Melodic en RPi (Raspbian Buster) se encuentra en el siguiente link: https://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi

Abrir una terminal en RPi y conceder permisos de super usuario (usuario de administrador)

```
sudo su
```

Posteriormente, en un terminal correr los siguientes comandos:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt-get update

sudo apt-get upgrade

sudo apt install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake

sudo rosdep init

rosdep update

mkdir -p ~/ros_catkin_ws

cd ~/ros_catkin_ws

rosinstall_generator desktop --rosdistro melodic --deps --wet-only --tar > melodic-desktop-wet.rosinstall

wstool init src melodic-desktop-wet.rosinstall

rosdep install -y --from-paths src --ignore-src --rosdistro melodic -r --os=debian:buster

sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic -j4

source /opt/ros/melodic/setup.bash

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

```



<h3>Instalación de Ubuntu 20.04</h3>

Descargar ubuntu 20.04 y Virtual Box y crear la máquina virtual con Ubuntu 20.04 de formato .iso en el Virtual Box

<h3>Ubuntu 20.04 :electron:</h3>

La instalación de ROS Noetic en Ubuntu 20.04 se encuentra en el siguiente link: https://wiki.ros.org/noetic/Installation/Ubuntu

Abrir una terminal en Ubuntu y conceder permisos de super usuario (administrador)

```
su root
```

Posteriormente, en un terminal correr los siguientes comandos:

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

Para validar la instalación de ROS, ejecutar los siguientes comandos en terminales independientes, para correr ROS maestro y para correr el nodo `talker` del paquete `roscpp_tutorials`, el cual es un nodo publicador.

```
roscore
```

```
rosrun roscpp_tutorials talker
```

Así mismo, si se quiere conocer la versión instala de ROS, ejecutar el siguiente comando:

```
rosversion -d
```

<h2>ROS</h2>

ROS es un sistema operativo de robots. Actualmente, es una colección de frameworks, herramientas (visualizar datos, guardar datos sensores, depurar y simular) y librerías. Una de las principales justificaciones de ROS es crear proyectos complejos de robótica con alta compatibilidad con hardware (Raspberry, PC(Ubuntu), Intel NUC, entre otros) y que sean reutilizables entre diferentes robots, independizando la programación por subsistemas (visión, navegación (movimiento), comunicación, sensores) y que a su vez se interconecten entre ellos.

otra de las ventajas de ROS es que los subsistemas están interconectados como grafos (nodos) a través de flechas (mensajes), es decir, es posible ejecutar diferentes programas de manera simultanea (paralelo)

![ROS IMU](image-3.png)

Fuente: https://atadiat.com/en/e-ros-imu-and-arduino-how-to-send-to-ros/

<h3>catkin</h3>

`catkin` es el compilador para las versiones de ROS más recientes (ej: Noetic), así mismo, crea ejecutables de programas. Para utilizar este compilador hay que crear un área de trabajo, en donde estarán todo el código (paquetes y nodos). Se recomienda tener un área de trabajo por proyecto.

![catkin](image-6.png)

Fuente: https://blog.csdn.net/JeremyZhao1998/article/details/104470039

<h3>Packages</h3>

Los paquetes pueden contener ejecutables, librerías, códigos (scripts) y mucha más información de los subsistemas de un proyecto. Es importante tener en cuenta que un paquete debe contener su propia carpeta, en la cual se deben tener dos archivos (package.xml y CMakeLists.txt), los cuales brindan información sobre el paquete con respecto al propietario, objetivo, dependencias y librerías del paquete. Los paquetes deben estar dentro del área de trabajo.

![Packages](image-5.png)

Fuente: https://blog.csdn.net/JeremyZhao1998/article/details/104470039

Para instalar los paquetes de tutoriales ejecutar el siguiente comando: 

```
sudo apt-get install ros-noetic-ros-tutorials
```

Si se desea ver la lista de los paquetes instalados se debe ejecutar la siguiente línea de comando:

```
rosls <TAB>
```

Para encontrar la ruta de un paquete se debe ejecutar el siguiente comando:

```
rospack find roscpp
```

<h3>Nodes</h3>

Son programas ejecutables que envían o reciben información a un topic, hay dos principales tipos de nodos, publicador y suscriptor. Los nodos deben estar dentro de los paquetes.

<h3>Topics</h3>

Son el punto intermedio entre el nodo publicador y el nodo suscriptor.

<h3>Estructura de directorios</h3>

![Estructura gráfica ROS](image-4.png)

Fuente: https://blog.csdn.net/JeremyZhao1998/article/details/104470039

![Estructura nivel ROS](image-7.png)

Fuente: https://blog.csdn.net/ck784101777/article/details/106297924





![ROS Master and Nodes](image.png)

Fuente: https://robodev.blog/ros-basic-concepts


![ROS Topic and Message](image-1.png)

Fuente: https://robodev.blog/ros-basic-concepts


![ROS service](image-2.png)

Fuente: https://robodev.blog/ros-basic-concepts
