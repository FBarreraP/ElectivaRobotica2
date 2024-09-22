<h1>Aula 17</h1>

Esta clase consiste en estimar Roll y Pitch a través del filtro complementario con la IMU6050 en diferentes nodos con ROS

<h2>Conexiones de nodos</h2>


Si se desea observar de manera gráfica la conexión entre nodos y topics, ejecutar el siguiente comando:

```
rosrun rqt_graph rqt_graph 
```

Para habilitar las estadísticas de frecuencias de publicación de los nodos ejecutar el siguiente comando:

```
rosparam set enable_statistics true
```


<h3>Ejemplo</h3>

```python
#!/usr/bin/env python2
#coding=utf-8

import rospy #Crear nodos con ROS
from std_msgs.msg import Float64, String
import matplotlib.pyplot as plt
import threading
import time
import numpy
from copy import copy, deepcopy

flag1 = 0
flag2 = 0

A = 0.6
B = 0.4
dt = 0.01
rad2deg = 180/3.141592

raw = 300

datos = numpy.zeros((raw,4))
datos1 = numpy.zeros((raw,4))
datos2 = numpy.zeros((raw,4))
Roll=numpy.zeros(((raw+1),4))
Roll[raw][0] = raw

def grafica():
    global j, Roll

    fig, ax = plt.subplots()
    while j<raw:
        print(j)
        ax.clear()
        ax.set_title(u'Ángulo Roll')
        ax.set_xlabel(u'muestra')
        ax.set_ylabel(u'grados (°)')
        ax.plot(Roll[:,1],'.b', label='RA')
        ax.plot(Roll[:,2],'.g', label='RG')
        ax.plot(Roll[:,3],'.r', label='RFC')
        ax.legend(loc='best')
        plt.pause(0.01)
    print(Roll[:,:])
    print(j)
    plt.show()
    j = 0
    print(j)

def callback1(mensaje):

    global j, datos1, flag1

    temp = mensaje.data.split(",")
    datos1[j,:] = temp
    j += 1
    flag1 = 1

def callback2(mensaje):

    global k, datos2, flag2

    temp2 = mensaje.data.split(",")
    datos2[k,:] = temp2
    k += 1
    flag2 = 1

def NS_Roll():

    global j, k, datos1, datos2, flag1, flag2, datos3, Roll

    j = 0
    k = 0

    rospy.init_node('NS_Roll')

    sub1 = rospy.Subscriber('a_xyz_c', String, callback1)
    sub2 = rospy.Subscriber('g_xyz_c', String, callback2)

    #rospy.spin()

    while not rospy.is_shutdown(): #Mientras el nodo no esté apagado, es decir, mientras esté encendido

        if j == k:
            if flag1 == 1 and flag2 == 1:
                flag1 = flag2 = 0
                datos3[j-1][:] = datos1[j-1,:]
                datos3[k-1][4:6] = datos2[k-1,1:3]
                #print(datos3[j-1,:])
                #print("j = %d y k = %d" %(j-1,k-1))
                Roll[j-1][0] = k
                #Acelerómetro
                Roll[j][1] = (math.atan2(datos3[j-1,2],datos3[j-1,3]))*rad2deg
                #Giroscopio
                Roll[k][2] = Roll[k-1][3]+((datos3[k-1,5]*dt)*rad2deg)
                #Filtro complementario
                Roll[j][3] = (A*Roll[k][2])+(B*Roll[j][1])
#       rospy.sleep(0.01)

if __name__ == '__main__':

    try:
        hilo2 = threading.Thread(target=grafica)
        hilo2.start()
        NPS_Roll()
    except rospy.ROSInterruptException:
        pass
```