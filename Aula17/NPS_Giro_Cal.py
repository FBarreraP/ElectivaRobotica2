#!/usr/bin/env python2
#coding=utf-8

import rospy #Crear nodos con ROS
from std_msgs.msg import Float64, String
import matplotlib.pyplot as plt
import threading
import time
import numpy
from copy import copy, deepcopy

SENSITIVITY_GYRO = 250.0/32768.0
offsets = [176.29666666666665, -34.836666666666666, -18.206666666666667]

datos = numpy.zeros((300,4))
datos1 = numpy.zeros((300,4))


def grafica():
    global j, datos1

    fig, ax = plt.subplots()
    while j<300:
        print(j)
        ax.clear()
        ax.set_title(u'Giroscopios calibrados XYZ')
        ax.set_xlabel(u'muestra')
        ax.set_ylabel(u'velocidad angular (°/s)')
        ax.plot(datos1[:,1],'-b', label='gx')
        ax.plot(datos1[:,2],'-g', label='gy')
        ax.plot(datos1[:,3],'-r', label='gz')
        ax.legend(loc='best')
        plt.pause(0.01)
    print(datos1[:,:])
    print(j)
    plt.show()
    j = 0
    print(j)

def callback(mensaje):

    global pub, j, datos1

    temp = mensaje.data.split(",")
    datos[j][:] = temp
    datos1[j][0] = datos[j,0]
    for i in range(0,3):
        datos1[j][i+1] = ((datos[j,i+1])-offsets[i])*SENSITIVITY_GYRO
    pub.publish(str(datos1[j,0])+","+str(datos1[j,1])+","+str(datos1[j,2])+","+str(datos1[j,3]))
    j+=1

def NPS_Giro_Cal():

    global pub, j

    j = 0

    rospy.init_node('NPS_Giro_Cal')

    pub = rospy.Publisher('g_xyz_c', String, queue_size=10)
    sub = rospy.Subscriber('g_xyz_sc', String, callback)

    rospy.spin()


if __name__ == '__main__':

    try:
       hilo2 = threading.Thread(target=grafica)
       hilo2.start()
       NPS_Giro_Cal()
    except rospy.ROSInterruptException:
        pass