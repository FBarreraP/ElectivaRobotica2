import serial
import numpy 
import matplotlib.pyplot as plt
import numpy.matlib as npm
from copy import copy, deepcopy
import time
import math 
import requests

# Enter Your API key here
myAPI = '65ZDEI8EDWZNK80G'
# URL where we will send the data, Don't change it
baseURL = 'https://api.thingspeak.com/update?api_key=%s' % myAPI 

SENSITIVITY_ACCEL = 2.0/32768.0
SENSITIVITY_GYRO = 250.0/32768.0
offsets = [472.92, -150.92, 177.6800000000003, 192.1, -41.0, 32.72]
A = 0.6
B = 0.4
dt = 0.01
rad2deg = 180/3.141592

data = serial.Serial('/dev/ttyACM0',9600,timeout=10)
print(data)

raw = 300

datos=numpy.zeros((raw,8)) #bits
datos1=numpy.zeros((raw,8)) #no calibrados
datos2=numpy.zeros((raw,8)) # calibrados
Roll=numpy.zeros(((raw+1),4))
Pitch=numpy.zeros(((raw+1),4))


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
    datos1 = deepcopy(datos)
    #print("datos1",datos1)
    datos2 = deepcopy(datos)
    #print("datos2",datos2)

    #No calibrados
    for i in range(0,3):
        for j in range(0,raw):
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
    ax1[1,1].plot(datos1[:,0], datos1[:,2], label='ax')
    ax1[1,1].plot(datos1[:,0], datos1[:,3], label='ay')
    ax1[1,1].plot(datos1[:,0], datos1[:,4], label='az')
    ax1[1,1].set_title('ax, ay y az')
    ax1[1,1].legend(loc='best')
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
    ax2[1,1].plot(datos1[:,0], datos1[:,5], label='gx')
    ax2[1,1].plot(datos1[:,0], datos1[:,6], label='gy')
    ax2[1,1].plot(datos1[:,0], datos1[:,7], label='gz')
    ax2[1,1].set_title('gx, gy y gz')
    ax2[1,1].legend(loc='best')
    g.show()

    #Calibrados
    for i in range(0,3):
        for j in range(0,raw):
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
    ax3[1,1].plot(datos2[:,0], datos2[:,2], label='ax')
    ax3[1,1].plot(datos2[:,0], datos2[:,3], label='ay')
    ax3[1,1].plot(datos2[:,0], datos2[:,4], label='az')
    ax3[1,1].set_title('ax, ay y az')
    ax3[1,1].legend(loc='lower right')
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
    ax4[1,1].plot(datos2[:,0], datos2[:,5], label='gx')
    ax4[1,1].plot(datos2[:,0], datos2[:,6], label='gy')
    ax4[1,1].plot(datos2[:,0], datos2[:,7], label='gz')
    ax4[1,1].set_title('gx, gy y gz')
    ax4[1,1].legend(loc='lower left')
    i.show()

    #ANGULOS
    Roll[raw][0] = raw
    Pitch[raw][0] = raw
    for i in range(0,raw):
        Roll[i][0] = i
        Pitch[i][0] = i
        #Acelerómetro
        Roll[i+1][1] = (math.atan2(datos2[i,3],datos2[i,4]))*rad2deg
        Pitch[i+1][1] = (math.atan2(-datos2[i,2],math.sqrt((datos2[i,3]*datos2[i,3])+(datos2[i,4]*datos2[i,4]))))*rad2deg
        #print("RollA[%d+1][1]: ",i,Roll[i+1][1])
        #Giroscópio
        Roll[i+1][2] = Roll[i][3]+((datos2[i,5]*dt)*rad2deg)
        Pitch[i+1][2] = Pitch[i][3]+((datos2[i,6]*dt)*rad2deg)
        #print("RollG[%d+1][2]: ",i,Roll[i+1][2])
        #Filtro complementario
        Roll[i+1][3] = (A*Roll[i+1][2])+(B*Roll[i+1][1])
        Pitch[i+1][3] = (A*Pitch[i+1][2])+(B*Pitch[i+1][1])
        #print("RollC[%d+1][3]: ",i,Roll[i+1][3])


    j = plt.figure(5)
    ax5 = j.subplots(2,2)
    j.suptitle('ÁNGULO ROLL')
    ax5[0,0].plot(Roll[:,0], Roll[:,1])
    ax5[0,0].set_title('Roll Acelerómetro')
    ax5[0,0].set_xlabel("Muestras")
    ax5[0,0].set_ylabel("Grados")
    ax5[0,1].plot(Roll[:,0], Roll[:,2])
    ax5[0,1].set_title('Roll Giroscópio')
    ax5[0,1].set_xlabel("Muestras")
    ax5[0,1].set_ylabel("Grados")
    ax5[1,0].plot(Roll[:,0], Roll[:,3])
    ax5[1,0].set_title('Roll Complementario')
    ax5[1,0].set_xlabel("Muestras")
    ax5[1,0].set_ylabel("Grados")
    ax5[1,1].plot(Roll[:,0], Roll[:,1], label='Acel')
    ax5[1,1].plot(Roll[:,0], Roll[:,2], label='Giro')
    ax5[1,1].plot(Roll[:,0], Roll[:,3], label='FC')
    ax5[1,1].set_title('Roll A, G y FC')
    ax5[1,1].set_xlabel("Muestras")
    ax5[1,1].set_ylabel("Grados")
    ax5[1,1].legend(loc='upper left')
    j.show()

    k = plt.figure(6)
    ax6 = k.subplots(2,2)
    k.suptitle('ÁNGULO PITCH')
    ax6[0,0].plot(Pitch[:,0], Pitch[:,1])
    ax6[0,0].set_title('Pitch Acelerómetro')
    ax6[0,0].set_xlabel("Muestras")
    ax6[0,0].set_ylabel("Grados")
    ax6[0,1].plot(Pitch[:,0], Pitch[:,2])
    ax6[0,1].set_title('Pitch Giroscópio')
    ax6[0,1].set_xlabel("Muestras")
    ax6[0,1].set_ylabel("Grados")
    ax6[1,0].plot(Pitch[:,0], Pitch[:,3])
    ax6[1,0].set_title('Pitch Complementario')
    ax6[1,0].set_xlabel("Muestras")
    ax6[1,0].set_ylabel("Grados")
    ax6[1,1].plot(Pitch[:,0], Pitch[:,1], label="Acel")
    ax6[1,1].plot(Pitch[:,0], Pitch[:,2], label="Giro")
    ax6[1,1].plot(Pitch[:,0], Pitch[:,3], label="FC")
    ax6[1,1].set_title('Pitch A, G y FC')
    ax6[1,1].set_xlabel("Muestras")
    ax6[1,1].set_ylabel("Grados")
    ax6[1,1].legend(loc='upper right')
    k.show()

    for i in range(raw):
        connection = requests.get(baseURL + '&field1=' + str(Roll[i,1]) + '&field2=' + str(Roll[i,2]) + '&field3=' + str(Roll[i,3]) + '&field4=' + str(Pitch[i,1]) + '&field5=' + str(Pitch[i,2]) + '&field6=' + str(Pitch[i,3]))
        print('Roll_A: %.4f, Roll_G: %.4f, Roll_FC: %.4f, Pitch_A: %.4f, Pitch_G: %.4f, Pitch_FC: %.4f,' %(Roll[i,1],Roll[i,2],Roll[i,3],Pitch[i,1],Pitch[i,2],Pitch[i,3]))
        connection.close()
        time.sleep(15)

else:
    print("\nAdios\n")