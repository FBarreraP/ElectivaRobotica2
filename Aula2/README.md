<h1>Aula 2</h1>

Esta clase consiste en comprender la IMU de 6 GDL y adquirir la información que proporciona el acelerómetro y el giroscopio.

<h2>MPU6050</h2>

Las Unidades de Medición Inercial (IMUs) son dispositivos electrónicos que proporcionan mediciones de velocidad angular y fuerza gravitacional en diferentes ejes. Algunas IMUs pueden medir campo magnético, temperatura, presión atmosférica, entre otras variables físicas.

<img src="" alt="STM32F303K8"/>
<figcaption>Fuente: https://os.mbed.com/platforms/ST-Nucleo-F303K8/</figcaption>

La MPU6050 es un dispositivo de 6 GDL que integra acelerómetro y giroscopio y cuenta con comunicación SPI e I2C.

<img src="image-1.png" alt="MPU6050"/>
<figcaption>Fuente: https://naylampmechatronics.com/blog/45_tutorial-mpu6050-acelerometro-y-giroscopio.html</figcaption>

<h3>I2C</h3>

La comunicación I2C es un protocolo de envío y recepción de datos serialmente a través de los pines SDA (datos) y SCL (reloj), en la cual se puede crear una topología de conexión entre diferentes dispositivos esclavos y maestros, direccionando la información a través de la dirección de esclavo de 7 bits.



<h3>STM32F303K8</h3>

La información de apoyo puede ser consultada en los manuales de la tarjeta, sin embargo, <a href="https://os.mbed.com/platforms/ST-Nucleo-F303K8/">aquí</a> también se puede encontrar alguna información desde la página de Mbed.

<img src="image.png" alt="STM32F303K8"/>
<figcaption>Fuente: https://os.mbed.com/platforms/ST-Nucleo-F303K8/</figcaption>

Para la versiones de Mbed 5 y 6, hay algunas APIs <a href="https://os.mbed.com/docs/mbed-os/v6.16/apis/index.html">aquí</a> 

<h3>Adquisición de datos </h3>

La adquisición de datos es realizada desde la STM32 programada en Mbed, para el ejemplo se utiliza la STM32F303K8

El siguiente código está para Mbed 2

```cpp
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
    i2c.write(MPU6500_address, cmd, 2);                //ACC fullsclae 2G
    wait(0.01);
    while(1) {
        //.................Construcción de la medición de los valores .................. 
        if(pc.getc() == 'H'){
            for(i=0; i<299; i++){
                cmd[0]=0x3B;
                i2c.write(MPU6500_address, cmd, 1);            //Escritura del registro de inicio
                i2c.read(MPU6500_address, GirAcel, 14);    //Lectura en rafaga de los valores de la MPU
                t.reset();
                t.start();
                raw_accelx = GirAcel[0]<<8 | GirAcel[1];    
                raw_accely = GirAcel[2]<<8 | GirAcel[3];
                raw_accelz = GirAcel[4]<<8 | GirAcel[5];
                raw_temp = GirAcel[6]<<8 | GirAcel[7];
                raw_gyrox = GirAcel[8]<<8 | GirAcel[9];
                raw_gyroy = GirAcel[10]<<8 | GirAcel[11];
                raw_gyroz = GirAcel[12]<<8 | GirAcel[13];
                
                accelx = raw_accelx*SENSITIVITY_ACCEL;
                accely = raw_accely*SENSITIVITY_ACCEL;
                accelz = raw_accelz*SENSITIVITY_ACCEL;
                gyrox = raw_gyrox*SENSITIVITY_GYRO;
                gyroy = raw_gyroy*SENSITIVITY_GYRO;
                gyroz = raw_gyroz*SENSITIVITY_GYRO;
                temp = (raw_temp/SENSITIVITY_TEMP)+21;
                wait_ms(9);
                wait_us(960);
                t.stop();
                timer = t.read();
                pc.printf("El tiempo es %f segundos \r", timer);
                pc.printf("%d %.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f \n\r",i+1,accelx, accely, accelz, gyrox, gyroy, gyroz, temp);
            }
        }
       /* for(int i=0; i<14; i++) {
            pc.printf("Buffer[%i] = %i \n\r",i,read_buffer[i]);
        }*/

        //myled = !myled;
    }
}
```

Adquisición de datos de la MPU9250

```cpp
//----------------------------------------------------------------------------
//                                BIBLIOTECAS
//----------------------------------------------------------------------------
#include "mbed.h"

// Enderecos dos escravos
#define    MPU6500_address            0xD0 // 7 bit I2C Endereço da MPU6500 (giroscópio e acelerômetro)
#define    MAG_ADDRESS_AK8963         0x18

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
int16_t raw_magnx, raw_magny, raw_magnz;

// Valores "RAW" de tipo double
   
// Saídas calibradas
float accelx, accely, accelz;
float gyrox, gyroy, gyroz;
float temp;
float magnx, magny, magnz;

// Bytes
char cmd[2];
char data[1];
char GirAcel[14];
char Mag[7];

float buffer[500][8];
int i;
Timer t; //Cria-se o objeto do temporizador
float timer=0,t_fin=10.0,cont_timer=0.0;
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
    if (data[0] != 0x71) { // DEFAULT_REGISTER_WHO_AM_I_MPU9250 0x71
      pc.printf("Erro de conexao com a MPU9250 \n\r");
      pc.printf("Opaaa. Eu nao sou a MPU9250, Quem sou eu? :S. I am: %#x \n\r",data[0]);
      pc.printf("\n\r");
      while (1);
    }else{
      pc.printf("Conexao bem sucedida com a MPU9250 \n\r");
      pc.printf("Oi, tudo joia?... Eu sou a MPU9250 XD \n\r");
      pc.printf("\n\r");
    }
    wait(0.1);  
    // Configura o Girôscopio (Full Scale Gyro Range  = 250 deg/s)
    cmd[0] = 0x1B; //GYRO_CONFIG 0x1B //Registrador de configuracao do Girôscopio
    cmd[1] = GYRO_FULL_SCALE_250_DPS;
    i2c.write(MPU6500_address, cmd, 2);                //gyro full scale 250 DPS
    // Configura o Acelerômetro (Full Scale Accelerometer Range  = 2g)
    cmd[0] = 0x1C; // ACCEL_CONFIG 0x1C //Registrador de configuracao do Acelerômetro
    cmd[1] = ACC_FULL_SCALE_2_G;
    i2c.write(MPU6500_address, cmd, 2);                //ACC fullsclae 2G
    wait(0.01);
    //Desativa modo de hibernação do AK8963
    cmd[0] = 0x37; //
    cmd[1] = 0x02;
    i2c.write(MPU6500_address, cmd, 2);  
    //Configura o Magnetômetro AK8963             //
    cmd[0] = 0x0A; //
    cmd[1] = 0x16;
    i2c.write(MAG_ADDRESS_AK8963, cmd, 2);
    //.....................................................................
    //        Quem sou eu para a AK8963 (magnetômetro)
    //.....................................................................
    pc.printf("1. Teste de conexao da AK8963... \n\r"); // Verifica a conexao
    cmd[0] = 0x00;
    i2c.write(MAG_ADDRESS_AK8963, cmd, 1);
    i2c.read(MAG_ADDRESS_AK8963, data, 1);
    if (data[0] != 0x48) { // DEFAULT_REGISTER_WHO_AM_I_MPU9250 0x71
      pc.printf("Erro de conexao com a AK8963 \n\r");
      pc.printf("Opaaa. Eu nao sou a AK8963, Quem sou eu? :S. I am: %#x \n\r",data[0]);
      pc.printf("\n\r");
      while (1);
    }else{
      pc.printf("Conexao bem sucedida com a AK8963 \n\r");
      pc.printf("Oi, tudo joia?... Eu sou a AK8963 XD \n\r");
      pc.printf("\n\r");
    }  
    wait(0.01);
    while(1) {
        //.................Construcción de la medición de los valores ..................
        if(pc.readable()) {
            if(pc.getc() == 'H') {
                i = 1;
                while(1) {
                    //.....................................................................
                    //                      ADQUISICAO DE DADOS
                    //.....................................................................
                    //Acelerómetro y giroscopio
                    cmd[0]=0x3B;
                    i2c.write(MPU6500_address, cmd, 1);            //Escritura del registro de inicio
                    i2c.read(MPU6500_address, GirAcel, 14);    //Lectura en rafaga de los valores de la MPU
                    t.reset();
                    t.start();
                    raw_accelx = GirAcel[0]<<8 | GirAcel[1];    
                    raw_accely = GirAcel[2]<<8 | GirAcel[3];
                    raw_accelz = GirAcel[4]<<8 | GirAcel[5];
                    raw_temp = GirAcel[6]<<8 | GirAcel[7];
                    raw_gyrox = GirAcel[8]<<8 | GirAcel[9];
                    raw_gyroy = GirAcel[10]<<8 | GirAcel[11];
                    raw_gyroz = GirAcel[12]<<8 | GirAcel[13];
                    //Magnetômetro
                    cmd[0]=0x02; //
                    i2c.write(MAG_ADDRESS_AK8963, cmd, 1);            
                    i2c.read(MAG_ADDRESS_AK8963, data, 1);  
                    if(data[0]& 0x01 == 0x01) { // wait for magnetometer data ready bit to be set
                        cmd[0]=0x03; //
                        i2c.write(MAG_ADDRESS_AK8963, cmd, 1);            
                        i2c.read(MAG_ADDRESS_AK8963, Mag, 7);  
                        if(!(Mag[6] & 0x08)) {
                            raw_magnx = (Mag[3] << 8 | Mag[2]);
                            raw_magny = (Mag[1] << 8 | Mag[0]);
                            raw_magnz = -(Mag[5] << 8 | Mag[4]);
                        }
                    }
                    //.....................................................................
                    //                      ESCALIZACAO DE DADOS
                    //.....................................................................
                    //Acelerómetro y giroscopio
                    accelx = raw_accelx*SENSITIVITY_ACCEL;
                    accely = raw_accely*SENSITIVITY_ACCEL;
                    accelz = raw_accelz*SENSITIVITY_ACCEL;
                    gyrox = raw_gyrox*SENSITIVITY_GYRO;
                    gyroy = raw_gyroy*SENSITIVITY_GYRO;
                    gyroz = raw_gyroz*SENSITIVITY_GYRO;
                    temp = (raw_temp/SENSITIVITY_TEMP)+21;
                    //Magnetômetro
                    magnx = raw_magnx*SENSITIVITY_MAGN;
                    magny = raw_magny*SENSITIVITY_MAGN;
                    magnz = raw_magnz*SENSITIVITY_MAGN;
                   
                    wait_ms(9);
                    //wait_us(962);
                    t.stop();
                    timer = t.read();
                    //pc.printf("El tiempo es %f segundos \r", timer);
                    cont_timer += timer; //cont_timer = cont_timer + timer;
                    pc.printf("%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f \n\r",i++,cont_timer,accelx, accely, accelz, gyrox, gyroy, gyroz, magnx, magny, magnz);
                    if(cont_timer >= t_fin) {
                        cont_timer=0.0;
                        pc.printf("A\n");
                        break;
                    }
                }
            }
        }
       /* for(int i=0; i<14; i++) {
            pc.printf("Buffer[%i] = %i \n\r",i,read_buffer[i]);
        }*/

        //myled = !myled;
    }
}
```