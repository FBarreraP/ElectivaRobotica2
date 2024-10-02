<h1>Aula 22</h1>

Esta clase consiste en utilizar el LoCoBot con el control del PS4 a través de los paquetes de ROS1 para este robot

<h2>ROS 1 en el LoCoBot PX100</h2>

Para ejecutar los paquetes de ROS1 del LoCoBot se debe tener instalado Ubuntu 20.04; la configuración de dichos paquetes está <a href="https://docs.trossenrobotics.com/interbotix_xslocobots_docs/ros_interface/ros1.html">aquí</a>.

Es importante tener en cuenta que el 'codename' del LoCoBot PX100 que se utilizará para la programación de dicho robot es: 

```
locobot_px100
```

Si se desea manipular el LoCoBot desde el computador personal (remotamente) se deben ejecutar los siguientes comandos; sin embargo, antes de ejecutar los comandos anteriores, se debe tener en cuenta que ROS, RViz y rosdep ya deben estar instalados en su máquina local para que la instalación remota de los paquetes del LoCoBot se realice correctamente.

```
sudo apt install curl

curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_rovers/main/interbotix_ros_xslocobots/install/xslocobot_remote_install.sh' > 

xslocobot_remote_install.sh

chmod +x xslocobot_remote_install.sh

./xslocobot_remote_install.sh -d noetic -b create3
```

* El script le pedirá que inserte el nombre de host de la computadora robot (NO del control remoto), por tanto, se debe ingresar: `locobot`

Posteriormente a la anterior instalación, se debe comentar la línea `ROS_MASTER_URI = http://<hostname>.local:11311` que se encuentra en el archivo "~/.bashrc"




<h3>Configuración</h3>



<h3>Guía de inicio</h3>


