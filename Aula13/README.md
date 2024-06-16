<h1>Aula 13</h1>

En esta clase se crean nodos publisher y subscriber.

Antes de crear cualquier nodo, se debe tener el espacio de trabajo y el paquete ya creado en el PC. Posteriormente, dirigirse a la ruta de dicho paquete con los comandos unix a través del terminal de Ubuntu.

<h2>Crear un nodo publisher</h2>

1. Crear una nueva carpeta con el nombre scripts
2. Ingresar a dicha carpeta
3. crear un archivo topic_publisher.py
4. Abrir el archivo con un editor de texto a través del comando subl o gedit, el comando nano también abre el archivo para editarlo desde la terminal

```
#!/usr/bin/env python

import rospy #Crear nodos con ROS
from std_msgs.msg import Int32

#def Nodo_conteo():
rospy.init_node('Nodo_conteo')  #Inicializa el nodo con el nombre Nodo_conteo

pub = rospy.Publisher('saludo', Int32, queue_size=1) #Declara el nodo como publisher con los parámetros  del nombre del topic, el tipo de dato del mensaje y 

rate = rospy.Rate(10) #Iniciaiza la frecuencia en Hertz de ejecución del nodo

cont = 0

while not ropsy.is_shutdown(): #Mientras el nodo no esté apagado, es decir, mientras esté encendido

    mensaje = "Buen día %s" %cont
    #rospy.loginfo(mensaje)
    pub.publish(mensaje)
    cont+=1
    rate.sleep() #Delay de 0.1s


#if __name__ == '__main__':
#    try:
#        Nodo_conteo()
#    except rospy.ROSInterruptException:
#        pass
```

<h2>Crear un nodo subscriber</h2>