import serial
import numpy 
import matplotlib.pyplot as plt
import numpy.matlib as npm
from copy import copy, deepcopy

SENSITIVITY_ACCEL = 2.0/32768.0
SENSITIVITY_GYRO = 250.0/32768.0
offsets = [472.92, -150.92, 177.6800000000003, 192.1, -41.0, 32.72]
 
data = serial.Serial('/dev/ttyACM0',9600,timeout=10)
print(data)

datos=numpy.zeros((100,8)) #bits
datos1=numpy.zeros((100,8)) #no calibrados
datos2=numpy.zeros((100,8)) # calibrados

value = input("\nQuiere adquirir los datos S/N \n\n")

if value == 'S' or value == 's':
    print("\nCapturando datos \n")
    data.write(b'H')
    for i in range(100):
        rec=data.readline() #byte
        print(rec)
        rec=rec.decode("utf-8") #string
        print(rec)
        rec=rec.split() #lista
        print(rec)
        datos[i][:]=rec
    print("\nTermina \n")
    print(datos,"\n")
    print(type(datos))
    print(type(datos[0,2]),type(datos[0][2]))
    print(f'{datos[0,2]},{datos[0][2]}')
    
    datos1 = deepcopy(datos)
    #print("datos1",datos1)
    datos2 = deepcopy(datos)
    #print("datos2",datos2)
    

    for i in range(0,3):
        for j in range(0,100):
            datos2[j][i+2] = ((datos2[j,i+2])-offsets[i])*SENSITIVITY_ACCEL
            datos2[j][i+5] = ((datos2[j,i+5])-offsets[i+3])*SENSITIVITY_GYRO
    #print("...datos2 \n",datos2)
    
    h = plt.figure(3)
    ax3 = h.subplots(2,2)
    h.suptitle('Acelerómetro calibrado MPU6050')
    ax3[0,0].plot(datos2[:,0], datos2[:,2])
    ax3[0,0].set_title('ax')
    ax3[0,1].plot(datos2[:,0], datos2[:,3])
    ax3[0,1].set_title('ay')
    ax3[1,0].plot(datos2[:,0], datos2[:,4])
    ax3[1,0].set_title('az')
    ax3[1,1].plot(datos2[:,0], datos2[:,(2,3,4)])
    ax3[1,1].set_title('ax, ay y az')
    h.show()

    i = plt.figure(4)
    ax4 = i.subplots(2,2)
    i.suptitle('Giróscopio calibrado MPU6050')
    ax4[0,0].plot(datos2[:,0], datos2[:,5])
    ax4[0,0].set_title('gx')
    ax4[0,1].plot(datos2[:,0], datos2[:,6])
    ax4[0,1].set_title('gy')
    ax4[1,0].plot(datos2[:,0], datos2[:,7])
    ax4[1,0].set_title('gz')
    ax4[1,1].plot(datos2[:,0], datos2[:,(5,6,7)])
    ax4[1,1].set_title('gx, gy y gz')
    i.show()

    for i in range(0,3):
        for j in range(0,100):
            datos1[j][i+2] = (datos1[j,i+2])*SENSITIVITY_ACCEL
            datos1[j][i+5] = (datos1[j,i+5])*SENSITIVITY_GYRO
    #print("...datos1 \n",datos1)

    f = plt.figure(1)
    ax1 = f.subplots(2,2)
    f.suptitle('Acelerómetro no calibrado MPU6050')
    ax1[0,0].plot(datos1[:,0], datos1[:,2])
    ax1[0,0].set_title('ax')
    ax1[0,1].plot(datos1[:,0], datos1[:,3])
    ax1[0,1].set_title('ay')
    ax1[1,0].plot(datos1[:,0], datos1[:,4])
    ax1[1,0].set_title('az')
    ax1[1,1].plot(datos1[:,0], datos1[:,(2,3,4)])
    ax1[1,1].set_title('ax, ay y az')
    f.show()

    g = plt.figure(2)
    ax2 = g.subplots(2,2)
    g.suptitle('Giróscopio no calibrado MPU6050')
    ax2[0,0].plot(datos1[:,0], datos1[:,5])
    ax2[0,0].set_title('gx')
    ax2[0,1].plot(datos1[:,0], datos1[:,6])
    ax2[0,1].set_title('gy')
    ax2[1,0].plot(datos1[:,0], datos1[:,7])
    ax2[1,0].set_title('gz')
    ax2[1,1].plot(datos1[:,0], datos1[:,(5,6,7)])
    ax2[1,1].set_title('gx, gy y gz')
    g.show()

else:
    print("\nAdios\n")