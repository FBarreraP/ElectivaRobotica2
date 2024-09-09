import serial
import numpy 
import matplotlib.pyplot as plt
import numpy.matlib as npm
from copy import copy, deepcopy
import time

SENSITIVITY_ACCEL = 2.0/32768.0
SENSITIVITY_GYRO = 250.0/32768.0
 
data = serial.Serial('/dev/ttyACM0',9600,timeout=10)
print(data)

raw = 300

datos=numpy.zeros((raw,8)) #bits
datos1=numpy.zeros((raw,8)) #no calibrados
datos2=numpy.zeros((raw,8)) # calibrados

value = input("\nQuiere adquirir los datos S/N \n\n")

if value == 'S' or value == 's':
    print("\nCapturando datos \n")
    data.write(b'H')
    for i in range(raw):
        rec=data.readline() #byte
        #print(rec)
        rec=rec.decode("utf-8") #string
        #print(rec)
        rec=rec.split() #lista
        #print(rec)
        datos[i][:]=rec
    print("\nTermina \n")
    print(datos,"\n")
    
    offsets = [numpy.mean(datos[:,2]), numpy.mean(datos[:,3]), numpy.mean(datos[:,4])-(32768/2), numpy.mean(datos[:,5]), numpy.mean(datos[:,6]), numpy.mean(datos[:,7])]
    print("... OFFSETS ... \n")
    print("ax, ay, az, gx, gy, gz \n")
    print(offsets)

else:
    print("\nAdios\n")