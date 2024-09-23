<h1>Aula 13</h1>

En esta clase se crean nodos publisher y subscriber con ROS melodic en la Raspberry.

Antes de crear cualquier nodo, se debe tener el espacio de trabajo (<i>workspace</i>) y el paquete (<i>package</i>) ya creado en el PC. El <i>workspace</i> en la Raspberry se debe crear específicamente en la carpeta "root". 

Antes de crear el workspace, ingresar con permisos de super usuario en la terminal.

<h2>Ejemplo</h2>

Con la herramienta de ROS "rqtgraph"

<div align="center">
<img src="Imagenes/rosgraph.png" alt="Grafos con rtq_graph"/>
<br>
<figcaption>Fuente: Autor</figcaption>
</div>

<h3>Crear un <i>workspace</i></h3>

Ingresar en una terminal a la carpeta donde se quiere guardar el <i>workspace</i> y posteriormente ejecutar la siguiente línea de comando:

```
mkdir -p WORKSPACE_FOLDER_NAME/src (ej: mkdir -p ~/aula13_ws/src)

cd WORKSPACE_FOLDER_NAME (ej: cd ~/aula13_ws)

catkin_make
```
<h3>Crear un <i>package</i></h3>

Ingresar a la carpeta "src" del <i>workspace</i> previamente creado, posteriormente ejecutar el siguiente comando, teniendo en cuenta como recomendación que el nombre del paquete debe comenzar en minúscula.

```
catkin_create_pkg PACKAGE_NAME depend1 depend2 depend2 ... dependN (ej: catkin_create_pkg ejemplos std_msgs rospy roscpp)
```

Desde el terminal, retornar a la carpeta del <i>workspace</i> y compilarlo, a través de la siguiente línea de comando, el cual se debe ejecutar cada vez que se crea un nuevo <i>package</i>.

```
catkin_make
```

<h3>Crear un nodo <i>publisher</i></h3>

1. Crear una nueva carpeta con el nombre scripts dentro del <i>package</i> a utilizar
2. Ingresar a dicha carpeta
3. Crear un archivo de tipo python para el nodo <i>publisher</i> (ej: Nodo_Saludo_Conteo.py)
4. Abrir el archivo con un editor de texto (nano) en una terminal de super usuario para editarlo

<!--
a través del comando subl o gedit, el comando nano también abre el archivo para editarlo pero desde la terminal. Para instalar sublime text se debe ejecutar el siguiente comando:

```
snap install sublime-text --classic
```
-->

```python
#!/usr/bin/env python2
#coding=utf-8

import rospy #Crear nodos con ROS
from std_msgs.msg import String

def Nodo_Saludo_Conteo():
    rospy.init_node('Nodo_Saludo_Conteo')  #Inicializa el nodo con el nombre Nodo_conteo

    pub = rospy.Publisher('conversacion', String, queue_size=1) #Declara el nodo como publisher con los parámetros  del nombre del topic, el tipo de dato del mensaje y 

    rate = rospy.Rate(10) #Iniciaiza la frecuencia en Hertz de ejecución del nodo

    cont = 0

    while not rospy.is_shutdown(): #Mientras el nodo no esté apagado, es decir, mientras esté encendido

        mensaje = "Buen dia %d" %cont
        #rospy.loginfo(mensaje)
        pub.publish(mensaje)
        cont+=1
        rate.sleep() #Delay de 0.1s

if __name__ == '__main__':
    try:
        Nodo_Saludo_Conteo()
    except rospy.ROSInterruptException:
        pass
```

Posteriormente, en la terminal ubicada en la ruta del nodo, convertirlo en ejecutable a través de la siguiente línea de comando:

```
sudo chmod u+x NODE_FILE.py (ej: sudo chmod u+x Nodo_Saludo_Conteo.py)
```

En la misma terminal, ir a la carpeta del workspace y cargar el nodo nuevo (actualizar el package) ejecutando la siguiente línea de comando:

```
source devel/setup.bash
```

Para no tener que actualizar el paquete manualmente cada vez que se ejecute el nodo, agregar la siguiente linea de comando en el archivo ".bashrc", el cual está en la carpeta de instalación de ROS:

```
nano ~/.bashrc
```
```
source /RUTA WORKSPACE/devel/setup.bash (ej: source ~/aula13_ws/devel/setup.bash)
```

Para aplicar las modificaciones en el archivo .bashrc, se debe actualizar a través del siguiente comando:

```
source ~/.bashrc
```

Posteriormente, abrir una nueva terminal y correr el nodo a través del siguiente comando:
```
rosrun PACKAGE NODE_FILE.py (ej: rosrun ejemplos Nodo_Saludo_Conteo.py)
```

<h3>Crear un nodo <i>subscriber</i></h3>

1. Ingresar a la carpeta "scripts" dentro del <i>package</i> a utilizar
2. Crear un archivo Nodo_Recibir_Saludo.py
3. Abrir el archivo con un editor de texto (nano) en una terminal de super usuario para editarlo

```python
#!/usr/bin/env python2

import rospy #Crear nodos con ROS
from std_msgs.msg import String


def callback(mensaje):
    print(mensaje.data)

def Nodo_Recibir_Saludo():
    rospy.init_node('Nodo_Recibir_Saludo')

    sub = rospy.Subscriber('conversacion', String, callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        Nodo_Recibir_Saludo()
    except rospy.ROSInterruptException:
        pass

```
Posteriormente, en la terminal ubicada en la ruta del nodo, convertirlo en ejecutable a través de la siguiente línea de comando:

```
sudo chmod u+x NODE_FILE.py (ej: sudo chmod u+x Nodo_Recibir_Saludo.py)
```

En la misma terminal, ir a la carpeta del workspace y cargar el nodo nuevo (actualizar el package) ejecutando la siguiente línea de comando; sin embargo, si ya se cargó este comando con la ruta en el archivo ".bashrc", no hay necesidad de ejecutarlo.

```
source devel/setup.bash
```

Posteriormente, abrir una nueva terminal y correr el nodo a través del siguiente comando:

```
rosrun PACKAGE NODE_FILE.py (ej: rosrun ejemplos Nodo_Recibir_Saludo.py)
```