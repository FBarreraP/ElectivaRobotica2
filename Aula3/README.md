<h1>Aula 3</h1>

Esta clase consiste en calibrar los sensores (acelerómetro y giroscopio) de la MPU6050.

<h3>Precisión</h3>

La precisión permite concentrar datos con baja desviación estándar, sin embargo, dichos datos están alejados del punto exacto de medición.

<h3>Exactitud</h3>

La exactitud indica que los datos están cercanos al punto exacto de medición

<h3>Resolución</h3>

Es la mínima variación entre muestra y muestra

$$2g/32768 = 0.00006103516$$

$$16g/32768 = 0.00048828125$$

<h3>Error</h3>







Adquisición de datos de la MPU6050

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

// Valores "RAW" de tipo inteiro
int16_t raw_accelx, raw_accely, raw_accelz;
int16_t raw_gyrox, raw_gyroy, raw_gyroz;
int16_t raw_temp;
// Valores "RAW" de tipo float
float Raw_accelx, Raw_accely, Raw_accelz;
float Raw_gyrox, Raw_gyroy, Raw_gyroz;

// Valores de "offsets" de tipo float 
float offset_accelx = 208.0, offset_accely = -314.0, offset_accelz = 16166.0;
float offset_gyrox = 208.5, offset_gyroy = -30.5, offset_gyroz = -68.5;
   
// Saídas nao calibradas
float accelx, accely, accelz;
float gyrox, gyroy, gyroz;
float temp;

// Saídas calibradas
float Accelx, Accely, Accelz;
float Gyrox, Gyroy, Gyroz;

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
I2C i2c(PB_7, PB_6 );

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
            //Dados "RAW"
            accelx = raw_accelx*SENSITIVITY_ACCEL;
            accely = raw_accely*SENSITIVITY_ACCEL;
            accelz = raw_accelz*SENSITIVITY_ACCEL;
            gyrox = raw_gyrox*SENSITIVITY_GYRO;
            gyroy = raw_gyroy*SENSITIVITY_GYRO;
            gyroz = raw_gyroz*SENSITIVITY_GYRO;
            temp = (raw_temp/SENSITIVITY_TEMP)+21;
            //Dados Calibrados
            Raw_accelx = raw_accelx; Raw_accely = raw_accely; Raw_accelz = raw_accelz; 
            Raw_gyrox = raw_gyrox; Raw_gyroy = raw_gyroy; Raw_gyroz = raw_gyroz;
            Accelx = (Raw_accelx-offset_accelx)*SENSITIVITY_ACCEL;
            Accely = (Raw_accely-offset_accely)*SENSITIVITY_ACCEL;
            Accelz = (Raw_accelz-(offset_accelz-(32768/2)))*SENSITIVITY_ACCEL;
            Gyrox = (Raw_gyrox-offset_gyrox)*SENSITIVITY_GYRO;
            Gyroy = (Raw_gyroy-offset_gyroy)*SENSITIVITY_GYRO;
            Gyroz = (Raw_gyroz-offset_gyroz)*SENSITIVITY_GYRO;
            //wait_ms(9);
            //wait_us(951);
            t.stop();
            timer = t.read();
            //pc.printf("El tiempo es %f segundos \r", timer);
            pc.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f \n\r",accelx, accely, accelz, gyrox, gyroy, gyroz, Accelx, Accely, Accelz, Gyrox, Gyroy, Gyroz);
        }
       /* for(int i=0; i<14; i++) {
            pc.printf("Buffer[%i] = %i \n\r",i,read_buffer[i]);
        }*/

        //myled = !myled;
    }
}
```

Calibración de datos de la MPU6050

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

// Valores de "offsets" de tipo float 
float offset_accelx = 0.0, offset_accely = 0.0, offset_accelz = 0.0;
float offset_gyrox = 0.0, offset_gyroy = 0.0, offset_gyroz = 0.0;

// Máximos e mínimos para os offsets
float max_accelx = -32768.0, min_accelx = 32767.0, max_accely = -32768.0, min_accely = 32767.0, max_accelz = -32768.0, min_accelz = 32767.0;
float max_gyrox = -32768.0, min_gyrox = 32767.0, max_gyroy = -32768.0, min_gyroy = 32767.0, max_gyroz = -32768.0, min_gyroz = 32767.0;

// Saídas calibradas
float accelx, accely, accelz;
float gyrox, gyroy, gyroz;
float temp;

// Bytes
char cmd[2];
char data[1];
char GirAcel[14];

float buffer[500][8];
int i,offset_samples = 100;
Timer t; //Cria-se o objeto do temporizador
float timer=0;
//.....................................................................
//                        Inicializacao I2C
//..................................................................... 
Serial pc(SERIAL_TX, SERIAL_RX);
I2C i2c(PB_7, PB_6 );

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
        //  .....................................................................
        //                 Calibração do Giroscopio e do Acelerometro
        //  .....................................................................
        pc.printf("CALIBRACAO DO GIROSCOPIO E DO ACELEROMETRO \n\r");
        pc.printf("Nao mexa neles ate que voce leia a mensagem 'calibracao finalizada' \n\r");
        wait(2);
        pc.printf("Coloque a IMU em posicao horizontal com as letras viradas para cima \n\r");
        wait(2);
        pc.printf("Lendo os sensores pela primeira vez... \n\r"); //Leitura de dados (primeiras 100 amostras dos sensores descartadas)
        for(i=0; i<offset_samples; i++){
            cmd[0]=0x3B;
            i2c.write(MPU6500_address, cmd, 1);            //Escritura del registro de inicio
            i2c.read(MPU6500_address, GirAcel, 14);    //Lectura en rafaga de los valores de la MPU
            raw_accelx = GirAcel[0]<<8 | GirAcel[1];    
            raw_accely = GirAcel[2]<<8 | GirAcel[3];
            raw_accelz = GirAcel[4]<<8 | GirAcel[5];
            raw_temp = GirAcel[6]<<8 | GirAcel[7];
            raw_gyrox = GirAcel[8]<<8 | GirAcel[9];
            raw_gyroy = GirAcel[10]<<8 | GirAcel[11];
            raw_gyroz = GirAcel[12]<<8 | GirAcel[13];
            wait(0.01);
        }
        pc.printf("Lendo os sensores e calculando os offsets... \n\r"); 
        for(i=0; i<offset_samples; i++){
            cmd[0]=0x3B;
            i2c.write(MPU6500_address, cmd, 1);            //Escritura del registro de inicio
            i2c.read(MPU6500_address, GirAcel, 14);    //Lectura en rafaga de los valores de la MPU
            raw_accelx = GirAcel[0]<<8 | GirAcel[1];    
            raw_accely = GirAcel[2]<<8 | GirAcel[3];
            raw_accelz = GirAcel[4]<<8 | GirAcel[5];
            raw_temp = GirAcel[6]<<8 | GirAcel[7];
            raw_gyrox = GirAcel[8]<<8 | GirAcel[9];
            raw_gyroy = GirAcel[10]<<8 | GirAcel[11];
            raw_gyroz = GirAcel[12]<<8 | GirAcel[13];
            // Máximos e mínimos para o acelerômetro em cada um dos seus eixos (X,Y e Z)
            //Eixo X
            if (raw_accelx >= max_accelx){
                max_accelx = raw_accelx;
            }
            if (raw_accelx <= min_accelx){
                min_accelx = raw_accelx;
            }
            //Eixo Y
            if (raw_accely >= max_accely){
                max_accely = raw_accely;
            }
            if (raw_accely <= min_accely){
                min_accely = raw_accely;
            }
            //Eixo Z
            if (raw_accelz >= max_accelz){
                max_accelz = raw_accelz;
            }
            if (raw_accelz <= min_accelz){
                min_accelz = raw_accelz;
            }
                    
            //Maximos e minimos para o Giroscopio em cada um dos seus eixos (X,Y e Z)
            //Eixo X
            if (raw_gyrox >= max_gyrox){
                max_gyrox = raw_gyrox;
            }
            if (raw_gyrox <= min_gyrox){
                min_gyrox = raw_gyrox;
            }
            //Eixo Y
            if (raw_gyroy >= max_gyroy){
                max_gyroy = raw_gyroy;
            }
            if (raw_gyroy <= min_gyroy){
                min_gyroy = raw_gyroy;
            }
            //Eixo Z
            if (raw_gyroz >= max_gyroz){
                max_gyroz = raw_gyroz;
            }
            if (raw_gyroz <= min_gyroz){
                min_gyroz = raw_gyroz;
            }
            wait(0.01);
        }
        //Calculando offset (bias) de calibração
        //Acelerômetro
        offset_accelx = (max_accelx + min_accelx) / 2;
        offset_accely = (max_accely + min_accely) / 2;
        offset_accelz = (max_accelz + min_accelz) / 2;
        //Giroscópio
        offset_gyrox = (max_gyrox + min_gyrox) / 2;
        offset_gyroy = (max_gyroy + min_gyroy) / 2;
        offset_gyroz = (max_gyroz + min_gyroz) / 2;
        pc.printf("Calibracao finalizada do acelerometro e do giroscopio \n\r");
        pc.printf("offset_accelx = %.2f, offset_accely = %.2f, offset_accelz = %.2f, offset_gyrox = %.2f, offset_gyroy = %.2f, offset_gyroz = %.2f \n\r",offset_accelx, offset_accely, offset_accelz, offset_gyrox, offset_gyroy, offset_gyroz);
        pc.printf("CALIBRACAO ENCERRADA \n\r");
        wait(100);
    }
}

```