<h1>Aula 21</h1>

Esta clase consiste en realizar una introducción del LoCoBot PX100 y de Ubuntu 20.04, que es desde donde se ejecutan los comandos para el Locobot.

<h2>Instalación de Ubuntu 20.04</h2>

Descargar ubuntu 20.04 y Virtual Box y crear la máquina virtual con Ubuntu 20.04 de formato .iso en el Virtual Box

<h3>ROS en Ubuntu 20.04 :electron:</h3>

La instalación de ROS Noetic en Ubuntu 20.04 se encuentra <a href="https://wiki.ros.org/noetic/Installation/Ubuntu">aquí</a>

Abrir una terminal en Ubuntu y conceder permisos de super usuario (administrador)

```
su root
```

Posteriormente, en el terminal anterior (usuario de administrador) ejecutar los siguientes comandos uno a uno:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

sudo apt install ros-noetic-desktop-full

source /opt/ros/noetic/setup.bash

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

source ~/.bashrc

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

sudo apt install python3-rosdep

sudo rosdep init

rosdep update
```

<h2>LoCoBot PX100</h2>

Es un robot híbrido ya que integra una plataforma diferencial (Create® 3) y un brazo manipulador de 4 GDL (Pincher), además cuenta con una cámara de profundidad (Intel®) y controlado por Ubuntu en un NUC (Intel®). Toda la documentación del LoCoBot PX100 se encuentra <a href="https://docs.trossenrobotics.com/interbotix_xslocobots_docs/#:~:text=Interbotix%20X-Series%20LoCoBots.%20The%20Low%20Cost%20Robot%20(LoCoBot)%20is%20a">aquí</a>.

<div align="center">
<img src="image-1.png" alt="LoCoBot PX100" height="500"/>
<br>
<figcaption>Fuente: https://docs.trossenrobotics.com/interbotix_xslocobots_docs/specifications/locobot_px100.html</figcaption>
</div>

<div align="center">
<img src="image-2.png" alt="Planos LoCoBot PX100"/>
<br>
<figcaption>Fuente: https://docs.trossenrobotics.com/interbotix_xslocobots_docs/specifications/locobot_px100.html</figcaption>
</div>

<h3>Componentes</h3>

Las especificaciones de cada componente que integra el LoCoBot PX100 están <a href="https://docs.trossenrobotics.com/interbotix_xslocobots_docs/specifications.html">aquí</a>

<h4>PincherX-100</h4>

El PincherX-100 es un brazo manipulador de 4 GDL de tipo antropomórfico o angular que cuenta con servomotores DYNAMIXEL XL430-W250 que ofrecen una alta resolución de 4096 posiciones y los parámetros PID definibles por el usuario, el monitoreo de temperatura, la retroalimentación de posición, así como los niveles de voltaje, entre otras características (límites de las articulaciones, configuraciones de los servos, planos, entre otras) más detalladas y los archivos STEP y STL del brazo manipulador se encuentran <a href="https://docs.trossenrobotics.com/interbotix_xsarms_docs/specifications/px100.html">aquí</a>

<div align="center">
<img src="image-8.png" alt="PincherX-100"/>
<br>
<figcaption>Fuente: https://docs.trossenrobotics.com/interbotix_xsarms_docs/specifications/px100.html</figcaption>
</div>

<h4>Create® 3</h4>

La plataforma robótica móvil Create 3 integra diferentes sensores (proximidad, odometro) y actuadores (motores DC). El software de esta plataforma está basado en ROS para el uso de dichos sensores y actuadores; además, el robot también proporciona algunos comportamientos autónomos listos para usar, como el acoplamiento con la base de carga, el seguimiento de la pared y las reacciones a los obstáculos. Es posible configurar esta plataforma móvil a través de un red.

<div align="center">
<img src="image-3.png" alt="Plataforma Create 3"/>
<br>
<figcaption>Fuente: https://docs.trossenrobotics.com/interbotix_xslocobots_docs/specifications.html</figcaption>
</div>

Más información de esta plataforma Create 3 se puede encontrar <a href="https://iroboteducation.github.io/create3_docs/">aquí</a>

<h4>NUC (PC)</h4>

El NUC es un computador de Intel con procesador Dual-Core i3 de 8va generación, 8 GB de RAM DDR4, unidad de estado sólido (SSD) de 240 GB, Intel Iris Plus Graphics 655, Wifi, Bluetooth 5.0, Gigabit Ethernet, compatibilidad con 4k, lector de tarjetas, compatible con dos monitores, HDMI, USB, Thunderbolt 3, Ubuntu 20.04.

<div align="center">
<img src="image-4.png" alt="NUC" width="400"/>
<br>
<figcaption>Fuente: https://docs.trossenrobotics.com/interbotix_xslocobots_docs/specifications.html</figcaption>
</div>

<h4>Cámara de profundidad</h4>

La cámara de Intel RealSense Depth Camera D435 realiza detección de profundidad para aplicaciones de visión inteligente.

<div align="center">
<img src="image-5.png" alt="Cámara" width="300"/>
<br>
<figcaption>Fuente: https://docs.trossenrobotics.com/interbotix_xslocobots_docs/specifications.html</figcaption>
</div>

<h4>Servo de la cámara</h4>

El DYNAMIXEL 2XL puede controlar dos ejes con un solo módulo, que sirve para variar el paneo e inclinación para la cámara del LoCoBot y así realizar la detección de objetos, ya sea que la plataforma esté en movimiento o estacionaria. Esta gama de servomotores 2XL adopta nuevas características que permiten un modo de control de 360 grados con su codificador magnético sin contacto y su estructura de ensamblaje de carcasa trasera hueca y engranaje metálico (pulvimetalurgia) que se utiliza para una alta durabilidad.

<div align="center">
<img src="image-6.png" alt="Servo (DYNAMIXEL 2XL) cámara"/>
<br>
<figcaption>Fuente: https://docs.trossenrobotics.com/interbotix_xslocobots_docs/specifications.html</figcaption>
</div>

<h4>Power bank</h4>

El banco de energía MAXOAK K2 está diseñado para alimentar computadoras portátiles durante períodos prolongados. Este banco de energía de 50 kAh (50000 Ah) puede mantener el NUC y todos los periféricos funcionando durante 4 horas con un uso moderado. Tiene múltiples salidas, incluidos conectores de barril de 20V 5A y 12V 2.5 A, dos puertos USB-A de 5V y 2.1 A y dos puertos USB-A de 5V 1A.

<div align="center">
<img src="image-7.png" alt="Power bank"/>
<br>
<figcaption>Fuente: https://docs.trossenrobotics.com/interbotix_xslocobots_docs/specifications.html</figcaption>
</div>

Más información de esta power bank se puede encontrar <a href="https://maxoak.net/products/maxoak-k2-185wh-50000mah-power-bank">aquí</a>

<h4>Control PS4</h4>

El control PS4 es utilizado para manipular todas las funciones del LoCoBot PX100 de manera manual e inalámbrica.

<div align="center">
<img src="image-9.png" alt="Control PS4"/>
<br>
<figcaption>Fuente: https://docs.trossenrobotics.com/interbotix_xslocobots_docs/getting_started/pairing_controller.html</figcaption>
</div>

<h3>Uso básico del LoCoBot PX100</h3>

El uso básico del LoCoBot PX100 consiste en aprender a cargarlo, encenderlo, programarlo remotamente desde un computador personal y apagarlo. La información se puede encontrar <a href="https://docs.trossenrobotics.com/interbotix_xslocobots_docs/getting_started/user_guide.html">aquí</a>

<h4>Cargar el LoCoBot PX100</h4>

Es importante tener en cuenta que el locobot Px100 se conforma por diferentes equipos: a) plataforma móvil (Create 3 from iRobot), b) computadora (Intel NUC NUC8i3BEH Mini PC), c) Power bank (MAXOAK K2), d) cámara (Intel® RealSense™ Depth Camera D435), e) servomotores de giro e inclinación (DINAMIXEL 2XL), f) brazo robótico de 4 GDL (PincherX 100 Robot Arm); los cuales requieren para su correcto funcionamiento de unas fuentes eléctricas que se deben mantener de acuerdo a lo establecido por el fabricante, es por este motivo que es recomendable mantenerlas a un nivel de carga mayor al 50%, es decir, no se debe dejar descargar completamente las baterías de iones de litio, ni tampoco dejarlas sin uso por un tiempo mayor de 6 meses

<h5>Cargar la Create 3</h5>

La batería de la Create 3 (iRobot) está integrada a dicha plataforma y debe venir con una fuente de alimentación con adaptador de AC y la estación de carga iRobot. La batería es de iones de litio 4S de 26 Wh, con un voltaje nominal de 14.4 V (12 V mín., 16.8 Vmáx.), la cual informará un estado de carga del 0% cuando el voltaje total del paquete alcance los 12.0 V. Se autoprotegerá y desconectará de cualquier carga a 10,8 V o menos.

<div align="center">
<img src="image-11.png" alt="Carga create 3"/>
<br>
<figcaption>Fuente: https://iroboteducation.github.io/create3_docs/hw/overview/</figcaption>
</div>

Si la batería se autoprotege, su sistema de gestión interno puede negarse a cargarse hasta que se reinicie. El reinicio de la batería se logra retirándola del robot durante al menos quince minutos, momento en el cual se debe volver a instalar en el robot y colocar el robot en el cargador. 

Para garantizar un correcto uso de la batería 4S es importante tener en cuenta los siguientes aspectos de acuerdo a las indicaciones eléctricas por el fabricante:

a.	Indispensable la estación de carga iRobot, la cual se debe conectar con su cable de salida de 16.8V y 2.5A a la tomacorriente de la pared
b.	La batería se puede recargar en solo 4 horas (la batería se autoprotegerá y desactivará la capacidad de carga si se carga continuamente durante cuatro horas sin alcanzar el 100 % del estado de carga.)
c.	Admite solo carga o descarga
d.	El anillo de luz mostrará el estado de carga y se animará mientras la batería se está cargando
e.	Encienda la plataforma colocándola en la estación de carga iRobot con el sensor de acoplamiento (docking sensor) frente a dicha estación y espere unos segundos hasta que se inicie. El LED de estado se iluminará y la base reproducirá el “sonido feliz”.
f.	Apague la plataforma manteniendo presionado el botón de encendido durante 10 segundos
g.	Mantenga el robot en la estación de carga iRobot cuando no esté en uso para evitar que la batería se descargue.
h.	El botón central está marcado con un icono de encendido, mientras que los botones laterales están marcados con uno o dos puntos. El botón central también contiene un anillo de seis LED RGB que visualizan el estado del robot. 

<div align="center">
<img src="image-12.png" alt="Botones Create 3"/>
<br>
<figcaption>Fuente: https://iroboteducation.github.io/create3_docs/hw/face/</figcaption>
</div>

Los otros estados del robot a través del anillo de seis LED RGB están <a href="https://iroboteducation.github.io/create3_docs/hw/face/">aquí</a>

<div align="center">
<img src="boot.gif" alt="Encendido Create 3"/>
<img src="charged_spinning.gif" alt="Cargando Create 3"/>
<img src="white_solid.jpg" alt="Carga 100% Create 3"/>
<img src="red_pulsing.gif" alt="Carga <10% Create 3"/>
<br>
<figcaption>Fuente: https://iroboteducation.github.io/create3_docs/hw/face/</figcaption>
</div>

<h5>Cargar la power bank</h5>

Para garantizar un correcto uso de la batería K2 es importante tener en cuenta los siguientes aspectos de acuerdo al manual del uso realizado por el fabricante y el cual se encuentra <a href="https://maxoak.net/products/maxoak-k2-185wh-50000mah-power-bank">aquí</a>:

<ol type="a">
  <li>El cargador recomendable es de entrada AC 100V~230V y salida DC 16.8V/2.5A</li>
  <li>La batería se puede recargar en solo 6-8 horas</li>
  <li>1000 ciclos de vida (carga y descarga)</li>
  <li>Admite carga y descarga al mismo tiempo</li>
  <li>Presione el botón On: encendido rápido, los LED indican la energía restante</li>
  <li>Presione el botón Apagado: permanezca 2 segundos para apagar</li>
  <li>Apagado automático: el banco de energía se apaga automáticamente en 30 segundos si no hay carga</li>
  <li>Luces LED en estado de descarga</li>
  <dd>0-25%: iluminación LED1</dd>
  <dd>25-50%: iluminación LED1, LED2</dd>
  <dd>50-75%: iluminación LED1, LED2, LED3</dd>
  <dd>75-100%: iluminación LED1, LED2, LED3, LED4</dd>
  <dd>alarma de bajo voltaje: LED1 parpadeando</dd>
  <li>Luces LED en estado de carga</li>
  0-25%: iluminación LED1
25-50%: LED1 encendido, LED2 parpadeando
50-75%: LED1, LED2 encendido, LED3 parpadeando
75-99%: LED1, LED2, iluminación LED3, LED4 intermitente
100%: iluminación LED1, LED2, LED3, LED4
</ol>

El procedimiento adecuado de carga de la batería K2 consiste simplemente en conectar el puerto de salida de DC del adaptador de AC al puerto de entrada del banco de energía y conecte el otro lado al tomacorriente de pared y verifique el estado de carga de acuerdo al estado de los LED.

<div align="center">
<img src="image-10.png" alt="Carga power bank"/>
<br>
<figcaption>Fuente: https://cdn.shopify.com/s/files/1/0364/8009/0247/files/User_Manual-K2.pdf?v=1615319987</figcaption>
</div>

<h4>Encender el LoCoBot PX100</h4>

La secuencia de pasos que se debe seguir para el correcto encendido de los componentes que conforman el LoCoBot PX100 es:

<ol>
  <li>Create 3 con la estación de carga</li>
  <li>Power Bank</li>
  <li>NUC</li>
</ol>

<h4>Configuración de conexión</h4>

Hay dos formas de programar el LoCoBot PX100 de manera remota o directamente desde el NUC. Por tanto, se debe tener en cuenta que hay dos nomenclaturas para esto:

1. 'Remote' que consiste en programar el LoCoBot PX100 a través de comunicación SSH desde un computador personal.
2. 'Robot' o 'LoCoBot' que consiste en programar el LoCoBot PX100 desde el computador NUC de dicho robot.

<h5>Conexión directa con el NUC</h5>

Inicialmente, se debe contar con un monitor HDMI, teclado y mouse. Posteriormente, seguir los siguientes pasos:

1. Iniciar sesión en el usuario `locobot` en el LoCoBot usando su contraseña predeterminada, `locobot`.
2. Abrir un terminal (presionando Ctrl+Alt+T).
3. Escribir el comando `hostname` para recuperar el nombre de <i>host</i> del robot. Este es el nombre de dominio de la computadora que usará cuando acceda a ella por SSH. Por lo general es `locobot`.

* Si está utilizando varios LoCoBots, debe hacer que cada nombre de host sea único, es decir, `locobot1`, `locobot2`, etc., a través de ejecutar el siguiente comando:

```
hostnamectl set-hostname <unique_hostname>
```

<h5>Conexión remota</h5>

Para realizar la comunicación SSH en el LoCoBot desde el computador remoto:

Acceder con permisos de super usuario en todos los terminales de la máquina virtual 'su root'

En el computador remoto, instalar el software OpenSSH Client para permitir una conexión de shell segura entre el computador remoto y el computador LoCoBot, a través del siguiente comando:

```
sudo apt install openssh-client
```

1. Se debe conectar el robot a la misma red WiFi a la que está conectada el computador remoto; el NUC genera una red de conexión local (ej: locobot6). Para validar la comunicación remota ejecutar el siguiente comando:

```
ping -c 3 locobot.local
```

2. Posteriormente, ejecutar el siguiente comando. Si el nombre de usuario o el nombre de host del robot son diferentes a `locobot`, utilícelo en su lugar.

```
ssh -X locobot@locobot.local
```

3. Si es necesario ingresar la contraseña: `locobot` cuando sea solicitada.

4. Una vez que se haya iniciado sesión en el LoCoBot, se pueden abrir varios terminales SSH utilizando el siguiente comando:

```
/usr/bin/dbus-launch /usr/bin/gnome-terminal &
```

<h4>Apagar el LoCoBot PX100</h4>

La secuencia de pasos que se debe seguir para el correcto apagado de los componentes que conforman el LoCoBot PX100 es:

<ol>
  <li>Ubuntu 20.04 (NUC)</li>
  <li>Power Bank</li>
  <li>Create 3</li>
</ol>

<h2>Ejemplo ROS Ubuntu</h2>

Lista todos los puertos ejecutar el siguiente comando:

```
ls /dev
```

Muestra información sobre los dispositivos seriales (ttyS#) conectados ejecutar el siguiente comando:

```
dmesg | grep tty
```

<!--
Lista todas los puertos USB conectados ejecutar el siguiente comando:

```
ls /dev/ttyS*
```
-->

Muestra las configuraciones de los dispositivos seriales (ttyS#) ejecutar el siguiente comando:

```
stty -F /dev/ttyS1 -a
```

Configura los baudios a 9600, 8 bits, 1 bit de stop y no bit de paridad (8N1) ejecutar el siguiente comando:

```
stty -F /dev/ttyS1 9600 cs8 -cstopb -parenb
```

Lee datos del puerto serial en un primer terminal ejecutar el siguiente comando:

```
cat < /dev/ttyS1
```

Escribe datos en el puerto serial en un segundo terminal ejecutar el siguiente comando:

```
echo 'H' > /dev/ttyS1
```

En sistemas operativos basados en linux (Raspian, Ubuntu, etc.) existen algunos monitores seriales como minicom o screen.

Para instalar minicom ejecutar el siguiente comando:

```
sudo apt-get install minicom
```

Para abrir minicom ejecutar el siguiente comando:

```
minicom -s
```

![Ejemplo1](image.png)

```c
/*
 * File:   main.c
 * Author: LENOVO
 *
 * Created on 25 de Agosto de 2024, 14:56
 */

#include <xc.h>
#include <stdio.h>
#include <string.h>

#pragma config FOSC = INTIO67
#pragma config WDTEN = OFF
#pragma config LVP = OFF

#define _XTAL_FREQ 16000000
#define time 10
//LCD
#define CD 0x01 //Clear Display
#define RH 0x02 //(0x03) Return Home
#define EMS 0x06 //Entry Mode Set
#define DC 0x0F //(0x0E) Display Control
#define DSr 0x1C //Display Shift Rigth
#define DSl 0x18 //Display Shift Left
#define FS 0x28 //(0x3C) Function Set
#define RAW1 0x80 //DDRAM display
#define RAW2 0xC0 //DDRAM display
#define RS LATE1 //Register Selection
#define E LATE0 //Enable

void settings(void);
void start(void);
//LCD
void SettingsLCD(unsigned char word);
void WriteLCD(unsigned char word);
void LCD(unsigned char data);
//Interrupción
void __interrupt() RECEIVE(void);

int digital1, digital2;
float conversion1, conversion2, temperature;
unsigned char i, flag = 0, d;
char text[20];

void main(void) {
    settings();
    while (1) {
        if (flag == 1) {
            flag = 0;
            start();
        }
    }
}

void settings(void) {
    OSCCON = 0x72;
    ANSELA = 0x03;
    ANSELC = 0x00;
    ANSELD = 0x00;
    ANSELE = 0x00;
    TRISA = 0x03;
    TRISD = 0;
    TRISE = 0;
    LATD = 0;
    LATE = 0;
    //LCD
    SettingsLCD(0x02); //Iniciar la LCD con el método nibble (4 MSB y 4 LSB)
    SettingsLCD(EMS);
    SettingsLCD(DC);
    SettingsLCD(FS);
    SettingsLCD(CD);
    //ADC
    ADCON0 = 0x01;
    ADCON1 = 0x00;
    ADCON2 = 0x95;
    //Serial UART
    TRISCbits.TRISC6 = 0;
    TRISCbits.TRISC7 = 1;
    SPBRG = 0x19;
    RCSTA = 0x90;
    TXSTA = 0x20;
    //Interrupción
    GIE = 1;
    PEIE = 1;
    RCIE = 1;
    RCIF = 0;
}

void start(void) {
    ADCON0 = 0x01;
    __delay_ms(time);
    GO = 1;
    while (GO == 1);
    digital1 = ADRESH << 8 | ADRESL;
    conversion1 = (float) digital1 * (5.0 / 1023.0);
    ADCON0 = 0x05;
    __delay_ms(time);
    GO = 1;
    while (GO == 1);
    digital2 = ADRESH << 8 | ADRESL;
    conversion2 = (float) digital2 * (5.0 / 1023.0);
    temperature = conversion2 / 0.01;
    SettingsLCD(RAW2);
    sprintf(text, "%.4f,%.4f\n\r", conversion1, temperature);
    for (i = 0; i < strlen(text); i++) {
        while (TX1IF == 0);
        TXREG = text[i];
        WriteLCD(text[i]);
    }
    //while(TXIF == 0);
    //TXREG = 0x0A;//Salto de línea
    //while (TXIF == 0);
    //TXREG = 0x0D; //Retorno de carro
}

void SettingsLCD(unsigned char word) {
    RS = 0;
    LCD(word >> 4); // 4 MSB
    LCD(word & 0x0F); // 4 LSB
}

void WriteLCD(unsigned char word) {
    RS = 1;
    LCD(word >> 4);
    LCD(word & 0x0F);
}

void LCD(unsigned char data) { //Opción bits
    E = 1;
    __delay_us(time * 5);
    LATDbits.LATD0 = (data & 0x01);
    __delay_us(time * 5);
    LATDbits.LATD1 = (data & 0x02) >> 1;
    __delay_us(time * 5);
    LATDbits.LATD2 = (data & 0x04) >> 2;
    __delay_us(time * 5);
    LATDbits.LATD3 = (data & 0x08) >> 3;
    __delay_us(time * 5);
    E = 0;
    __delay_us(time * 5);
}

void __interrupt() RECEIVE(void) {
    if (RCIF == 1) {
        d = RCREG;
        if (d == 'H') {
            flag = 1;
        }
    }
}
```

Nodo publisher para adquisición de datos seriales enviados desde una terminal, de un potenciómetro y un LM35

```python
#!/usr/bin/env python3

import rospy #Crear nodos con ROS
from std_msgs.msg import Float64, String
import serial
import threading
import time

def callback(mensaje):
    print(f'Mensaje servo: {mensaje.data}')

def callback2(mensaje):
    global pub1, pub2, s

    print(f'Serial: {s}')
    print(f'Mensaje usuario: {mensaje.data}')
    value = mensaje.data
    if value == 'S' or value == 's':  
        s.write(b'H')      
        print(f'H enviada')
        #time.sleep(0.001)
        rec = s.readline() #byte
        print(rec)
        rec = rec.decode() #utf-8
        print(rec)
        print(type(rec))
        rec = rec.split(",") #list
        print(rec[0])
        print(rec[1])
        pub1.publish(float(rec[0]))
        pub2.publish(float(rec[1]))
        #rospy.loginfo(mensaje)
        #rate.sleep() #Delay de 0.1s

def Nodo_Proteus():
    global pub1, pub2, s

    rospy.init_node('Nodo_Proteus')  #Inicializa el nodo con el nombre Nodo_conteo

    pub1 = rospy.Publisher('potenciometro', Float64, queue_size=10) #Declara el nodo como publisher con los parámetros  del nombre del topic, el tipo de dato del mensaje y 
    pub2 = rospy.Publisher('LM35', Float64, queue_size=10)
    sub1 = rospy.Subscriber('servo', Float64, callback)
    sub2 = rospy.Subscriber('teclado', String, callback2)

    rate = rospy.Rate(10) #Inicializa la frecuencia en Hertz de ejecución del nodo

    s = serial.Serial('/dev/ttyS1', 9600, 8, 'N', 1) #9600 8N1

    rospy.spin()

if __name__ == '__main__':
    try:
        Nodo_Proteus()
    except rospy.ROSInterruptException:
        pass
```

Nodo subscriber para graficar de datos de un potenciómetro

```python
#!/usr/bin/env python3

import rospy #Crear nodos con ROS
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
import threading
import time

n = []

def grafica():
    fig, ax = plt.subplots()
    ax.set_title('potenciometro')
    ax.set_xlabel('muestra')
    ax.set_ylabel('voltaje')
    while len(n)<20:
        #print(len(n))  
        ax.clear()
        ax.plot(n[:],'.b')
        plt.pause(0.01)
    plt.show()

def callback(mensaje):
    
    global pub

    print(mensaje.data)
    n.append(mensaje.data)
    angulo = mensaje.data*(180.0/5.0)
    pub.publish(angulo)
    

def Nodo_Grafica_Pot():

    global pub

    rospy.init_node('Nodo_Grafica_Pot')

    pub = rospy.Publisher('servo', Float64, queue_size=10)

    sub = rospy.Subscriber('potenciometro', Float64, callback)

    rospy.spin()


if __name__ == '__main__':

    hilo2 = threading.Thread(target=grafica)
    hilo2.start()
    Nodo_Grafica_Pot()    
```

Nodo de Usuario

```python
#!/usr/bin/env python3

import rospy #Crear nodos con ROS
from std_msgs.msg import String
import serial

def Nodo_Usuario():

    rospy.init_node('Nodo_Usuario')  #Inicializa el nodo con el nombre Nodo_conteo

    pub = rospy.Publisher('teclado', String, queue_size=10) #Declara el nodo como publisher con los parámetros  del nombre del topic, el tipo de dato del mensaje y 

    rate = rospy.Rate(10) #Inicializa la frecuencia en Hertz de ejecución del nodo

    while not rospy.is_shutdown(): #Mientras el nodo no esté apagado, es decir, mientras esté encendido
        value = input("Quiere adquirir un dato? S/N")

        pub.publish(str(value))


if __name__ == '__main__':
    try:
        Nodo_Usuario()
    except rospy.ROSInterruptException:
        pass
```


Nodo subscriber para graficar de datos de un LM35