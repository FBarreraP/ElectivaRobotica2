<h1>Aula 11</h1>

Esta clase consiste en conectar la Raspberry Pi con Thinkspeak para monitorear datos en la nube.

<h2>Industria 4.0</h2>

<div align="center">
<img src="Imagenes/image.png" alt="Revoluciones industriales"/>
<br>
<figcaption>Fuente: https://wertambiental.com.br/2019/01/15/industria_4-0/</figcaption>
</div>

<div align="center">
<img src="Imagenes/image-1.png" alt="Industria 4.0"/>
<br>
<figcaption>Fuente: https://www.thesmartcityjournal.com/es/tecnologia/impacto-industria-4-0-fabricas</figcaption>
</div>

<h2>IoT</h2>

Es la interconexión de sensores, actuadores, software y otras tecnologías a través de internet con el objetivo de interaccionar con estos sin necesidad de intervención humana

<div align="center">
<img src="Imagenes/image-2.png" alt="IoT"/>
<br>
<figcaption>Fuente: https://tecrea.com.co/iot-y-las-empresas-que-generan-desarrollo-en-colombia-y-el-mundo/</figcaption>
</div>

<h3>Thinkspeak</h3>

<div align="center">
<img src="Imagenes/image-3.png" alt="Thinkspeak"/>
<br>
<figcaption>Fuente: https://www.thesmartcityjournal.com/es/tecnologia/impacto-industria-4-0-fabricas</figcaption>
</div>

<h3>Ejemplo</h3>

Adquisición de datos MPU6050 con STM32

```c++
//----------------------------------------------------------------------------
//                                BIBLIOTECAS
//----------------------------------------------------------------------------
#include "mbed.h"

// Enderecos dos escravos
#define    MPU6500_address            0xD0 // 7 bit I2C Endereço da MPU6500 (giroscópio e acelerômetro)

// Escalas do girôscopio
#define    GYRO_FULL_SCALE_250_DPS    0x00 // SCALE_250 (°/s) = 0 (0x00 = 000|00|000)
#define    GYRO_FULL_SCALE_500_DPS    0x08 // SCALE_500 (°/s) = 1 (0x08 = 000|01|000)
#define    GYRO_FULL_SCALE_1000_DPS   0x10 // SCALE_1000 (°/s) = 2 (0x10 = 000|10|000)
#define    GYRO_FULL_SCALE_2000_DPS   0x18 // SCALE_2000 (°/s) = 3 (0x18 = 000|11|000)

// Escalas do acelerômetro
#define    ACC_FULL_SCALE_2_G        0x00 // SCALE_2_G (g) = 0 (0x00 = 000|00|000)
#define    ACC_FULL_SCALE_4_G        0x08 // SCALE_4_G (g) = 1 (0x08 = 000|01|000)
#define    ACC_FULL_SCALE_8_G        0x10 // SCALE_8_G (g) = 2 (0x10 = 000|10|000)
#define    ACC_FULL_SCALE_16_G       0x18 // SCALE_16_G (g) = 3 (0x18 = 000|11|000)

// Escalas de conversao (As taxas de conversão são especificadas na documentação)
#define SENSITIVITY_ACCEL     2.0/32768.0             // Valor de conversão do Acelerômetro (g/LSB) para 2g e 16 bits de comprimento da palavra
#define SENSITIVITY_GYRO      250.0/32768.0           // Valor de conversão do Girôscopio ((°/s)/LSB) para 250 °/s e 16 bits de comprimento da palavra
#define SENSITIVITY_TEMP      333.87                  // Valor de sensitividade do Termometro (Datasheet: MPU-9250 Product Specification, pag. 12)
#define TEMP_OFFSET           21                      // Valor de offset do Termometro (Datasheet: MPU-9250 Product Specification, pag. 12)
#define SENSITIVITY_MAGN      (10.0*4800.0)/32768.0   // Valor de conversão do Magnetômetro (mG/LSB) para 4800uT, 16 bits de comprimento da palavra e conversao a Gauss (10mG = 1uT)

//----------------------------------------------------------------------------
//                           DECLARACAO DE VARIAVEIS
//----------------------------------------------------------------------------
// Offsets de calibração (AQUI DEVEM IR OS VALORES DETERMINADOS EN LA CALIBRACAO PREVIA COM O CÓDIGO "calibracao.ino")
//double offset_accelx = 334.0, offset_accely = -948.0, offset_accelz = 16252.0;
//double offset_gyrox = 111.0, offset_gyroy = 25.0, offset_gyroz = -49.0;

// Valores "RAW" de tipo inteiro
int16_t raw_accelx, raw_accely, raw_accelz;
int16_t raw_gyrox, raw_gyroy, raw_gyroz;
int16_t raw_temp;

// Valores "RAW" de tipo double 
   
// Saídas calibradas
float accelx, accely, accelz;
float gyrox, gyroy, gyroz;
float temp;

// Bytes
char cmd[2];
char data[1];
char GirAcel[14];

float buffer[500][8];
int i;
Timer t; //Cria-se o objeto do temporizador
float timer=0;
//.....................................................................
//                        Inicializacao I2C
//..................................................................... 
Serial pc(SERIAL_TX, SERIAL_RX);
I2C i2c(PB_7, PB_6 );//SDA,SCL

//DigitalOut myled(LED1);

int main(){
    // Desativa modo de hibernação do MPU6050
    cmd[0] = 0x6B;
    cmd[1] = 0x00;
    i2c.write(MPU6500_address, cmd, 2);
    
    pc.printf("TESTE DE CONEXAO PARA O GIROSCOPIO E O ACELEROMETRO \n\r");
    //.....................................................................
    //        Quem sou eu para a MPU6050 (giroscópio e acelerômetro)
    //.....................................................................
    pc.printf("1. Teste de conexao da MPU6050... \n\r"); // Verifica a conexao
    cmd[0] = 0x75;
    i2c.write(MPU6500_address, cmd, 1);
    i2c.read(MPU6500_address, data, 1);
    if (data[0] != 0x68) { // DEFAULT_REGISTER_WHO_AM_I_MPU6050 0x68
      pc.printf("Erro de conexao com a MPU6050 \n\r");
      pc.printf("Opaaa. Eu nao sou a MPU6050, Quem sou eu? :S. I am: %#x \n\r",data[0]);
      pc.printf("\n\r");
      while (1);
    }else{
      pc.printf("Conexao bem sucedida com a MPU6050 \n\r");
      pc.printf("Oi, tudo joia?... Eu sou a MPU6050 XD \n\r");
      pc.printf("\n\r");
    }
    wait(0.1);  
    // Configura o Girôscopio (Full Scale Gyro Range  = 250 deg/s)
    cmd[0] = 0x1B; //GYRO_CONFIG 0x1B //Registrador de configuracao do Girôscopio
    cmd[1] = 0x00;
    i2c.write(MPU6500_address, cmd, 2);                //gyro full scale 250 DPS
    // Configura o Acelerômetro (Full Scale Accelerometer Range  = 2g)
    cmd[0] = 0x1C; // ACCEL_CONFIG 0x1C //Registrador de configuracao do Acelerômetro
    cmd[1] = 0x00;
    i2c.write(MPU6500_address, cmd, 2);                //ACC full scale 2g
    wait(0.01);
    while(1) {
        //.................Construcción de la medición de los valores .................. 
        if(pc.getc() == 'H'){
            for(i=0; i<300; i++){
                t.reset();
                t.start();
                cmd[0]=0x3B;
                i2c.write(MPU6500_address, cmd, 1);            //Escritura del registro de inicio
                i2c.read(MPU6500_address, GirAcel, 14);    //Lectura en rafaga de los valores de la MPU
                //Dados crus
                raw_accelx = GirAcel[0]<<8 | GirAcel[1];    
                raw_accely = GirAcel[2]<<8 | GirAcel[3];
                raw_accelz = GirAcel[4]<<8 | GirAcel[5];
                raw_temp = GirAcel[6]<<8 | GirAcel[7];
                raw_gyrox = GirAcel[8]<<8 | GirAcel[9];
                raw_gyroy = GirAcel[10]<<8 | GirAcel[11];
                raw_gyroz = GirAcel[12]<<8 | GirAcel[13];
                wait_us(8380);
                t.stop();
                timer = t.read();
                pc.printf("%d %.2f %.2f %.2f %.2f %.2f %.2f %.2f \n\r", i, timer, (float)raw_accelx, (float)raw_accely, (float)raw_accelz, (float)raw_gyrox, (float)raw_gyroy, (float)raw_gyroz);
            }
        }
    }
}
```

Calibración MPU6050 en RPi

```python
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
```

Filtro complementario (Roll y Pitch) y Thinkspeak

```python
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
```