#!/usr/bin/env python2
#coding=utf-8

import rospy #Crear nodos con ROS
from std_msgs.msg import Float64, String
import matplotlib.pyplot as plt
import threading
import time
import numpy
from copy import copy, deepcopy
import math

flag1 = 0
flag2 = 0

A = 0.6
B = 0.4
dt = 0.01
rad2deg = 180/3.141592

raw = 300

datos1 = numpy.zeros((raw,4))
datos2 = numpy.zeros((raw,4))
datos3 = numpy.zeros((raw,7))
Pitch=numpy.zeros(((raw+1),4))
Pitch[raw][0] = raw

def grafica():
    global j, Pitch

    fig, ax = plt.subplots()
    while j<raw:
        print(j)
        ax.clear()
        ax.set_title(u'Ángulo Pitch')
        ax.set_xlabel(u'muestra')
        ax.set_ylabel(u'grados (°)')
        ax.plot(Pitch[:,1],'-b', label='PA')
        ax.plot(Pitch[:,2],'-g', label='PG')
        ax.plot(Pitch[:,3],'-r', label='PFC')
        ax.legend(loc='best')
        plt.pause(0.01)
    print(Pitch[:,:])
    print(j)
    plt.show()
    j = 0
    print(j)

def callback1(mensaje):

    global j, datos1, flag1

    temp = mensaje.data.split(",")
    datos1[j,:] = temp
    #print(datos1[j,:])
    j += 1
    flag1 = 1

def callback2(mensaje):

    global k, datos2, flag2

    temp2 = mensaje.data.split(",")
    datos2[k,:] = temp2
    k += 1
    flag2 = 1

def NS_Pitch():

    global j, k, datos1, datos2, flag1, flag2, datos3, Roll

    j = 0
    k = 0

    rospy.init_node('NS_Pitch')

    sub1 = rospy.Subscriber('a_xyz_c', String, callback1)
    sub2 = rospy.Subscriber('g_xyz_c', String, callback2)

    #rospy.spin()

    while not rospy.is_shutdown(): #Mientras el nodo no esté apagado, es decir, mientras esté encendido

        if j == k:
            if flag1 == 1 and flag2 == 1:
                flag1 = flag2 = 0
                datos3[j-1][0:4] = datos1[j-1,0:4]
                datos3[k-1][4:7] = datos2[k-1,1:4]
                #print(datos3[j-1,:])
                #print("j = %d y k = %d" %(j-1,k-1))
                Pitch[j-1][0] = k-1
                #Acelerómetro
                Pitch[j][1] = (math.atan2(-datos3[j-1,1],math.sqrt((datos3[j-1,2]*datos3[j-1,2])+(datos3[j-1,3]*datos3[j-1,3]))))*rad2deg
                #Giroscopio
                Pitch[k][2] = Pitch[k-1][3]+((datos3[k-1,5]*dt)*rad2deg)
                #Filtro complementario
                Pitch[j][3] = (A*Pitch[k][2])+(B*Pitch[j][1])
#       rospy.sleep(0.01)

if __name__ == '__main__':

    try:
        hilo2 = threading.Thread(target=grafica)
        hilo2.start()
        NS_Pitch()
    except rospy.ROSInterruptException:
        pass