#!/usr/bin/env python2
#coding=utf-8

import rospy #Crear nodos con ROS
from std_msgs.msg import Float64, String
import serial
import numpy

def callback(mensaje):
    global pub1, pub2, rate, s, datos

    value = mensaje.data
    if value == 'S' or value == 's':
        print("\nCapturando datos \n")
        s.write(b'H')
        for i in range(300):
            rec = s.readline() #byte
            #print(rec)
            rec = rec.decode("utf-8") #utf-8
            #print(rec)
            rec = rec.split() #list
            #print(rec)
            datos[i][:]=rec
            pub1.publish(str(datos[i,0])+","+str(datos[i,2])+","+str(datos[i,3])+","+str(datos[i,4]))
            pub2.publish(str(datos[i,0])+","+str(datos[i,5])+","+str(datos[i,6])+","+str(datos[i,7]))
            #rospy.loginfo(mensaje)
            #rate.sleep() #Delay de 0.1s
        print("\nTermina \n")

def NPS_IMU6050():
    global pub1, pub2, rate, s, datos

    datos=numpy.zeros((300,8)) #no calibrados

    rospy.init_node('NPS_IMU6050')  #Inicializa el nodo con el nombre Nodo_conteo

    pub1 = rospy.Publisher('a_xyz_sc', String, queue_size=10) #Declara el nodo como publisher con los pa$
    pub2 = rospy.Publisher('g_xyz_sc', String, queue_size=10)
    sub1 = rospy.Subscriber('tecla', String, callback)

    rate = rospy.Rate(10) #Inicializa la frecuencia en Hertz de ejecución del nodo

    s = serial.Serial('/dev/ttyACM0', 9600, 8, 'N', 1) #9600 8N1

    rospy.spin()

if __name__ == '__main__':
    try:
        NPS_IMU6050()
    except rospy.ROSInterruptException:
        pass