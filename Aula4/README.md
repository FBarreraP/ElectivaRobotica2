<h1>Aula 4</h1>

Esta clase consiste en realizar la comunicación serial entre la RPi (Raspbian Buster), la STM32 y la MPU6050.

<h2>Instalación Ubuntu 22.04 server en RPi</h2>


Posteriormente a la instalación del Ubuntu 22.04 server, se recomiendan seguir los pasos a través del terminal:

Mostrar interfaces de red del sistema y su estado

```
ip link
```
Para revisar el estado de las redes del sistema  herramienta en terminal con NetworkManager

```
nmcli device status
```
Para mostrar la información detallada de las IP de las redes del sistema

```
ip a
```

Para verificar que la RPi esté conectada a internet
```
ping -c 4 8.8.8.8
```

Para actualizar la lista de paquetes disponibles desde los repositorios.

```
sudo apt update
```

<h3>Instalar escritorio gráfico LXDE</h3>

```
sudo apt install -y lxde-core  lxappearance lightdm 
```
Posteriormente seleccionar lightdm y en la siguiente ventana marcar las 2 opciones con *

Finalmente reiniciar el sistema operativo

```
sudo reboot
```

<h3>Activar la opción WiFi en LXDE</h3> 

Para administrar de forma gráfica las redes del sistema
```
sudo apt install network-manager
```

Instalar otros complementos:

- Chromium
- Visual studio code
- minicom

<h2>Comunicación RPi, STM32 y MPU6050</h2>

Lista todos los puertos ejecutar el siguiente comando:

```
ls /dev
```
<!--
Muestra información sobre los dispositivos COM conectados ejecutar el siguiente comando:

```
dmesg | grep tty
```
-->

Lista todas los puertos USB conectados ejecutar el siguiente comando:

```
ls /dev/ttyACM*
```

Muestra las configuraciones de los dispositivos COM ejecutar el siguiente comando:

```
stty -F /dev/ttyACM0 -a
```

Configura los baudios a 9600, 8 bits, 1 bit de stop y no bit de paridad (8N1) ejecutar el siguiente comando:

```
stty -F /dev/ttyACM0 9600 cs8 -cstopb -parenb
```

Lee datos del puerto serial en un primer terminal ejecutar el siguiente comando:

```
cat < /dev/ttyACM0
```

Escribe datos en el puerto serial en un segundo terminal ejecutar el siguiente comando:

```
echo 'H' > /dev/ttyACM0
```

En sistemas operativos basados en linux (Raspian, Ubuntu, etc.) existen algunos monitores seriales como minicom o screen. Para instalar minicom ejecutar el siguiente comando:

```
sudo apt-get install minicom
```

Para abrir minicom ejecutar el siguiente comando:

```
minicom -s
```

<h3>Adquisición de datos MPU6050 en la STM32F767ZI</h3>

```c++
//Ejemplo I2C
//Fabián Barrera Prieto
//Universidad ECCI
//STM32F767ZIT6U
//operation 'or' (|) for set bit and operation 'and' (&) for clear bit

#include <stdio.h>
#include "stm32f7xx.h"
#include <string.h>

//MPU6050
#define MPU6500_address 0x68 // Endereço da MPU6500 (giroscópio e acelerômetro)

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
#define TEMP_OFFSET           21                      // Valor de offset do Termometro (Datasheet: MPU-6050 Product Specification, pag. 12)

// Offsets de calibração (AQUI DEVEM IR OS VALORES DETERMINADOS EN LA CALIBRACAO PREVIA COM O CÓDIGO "calibracao.ino")
//double offset_accelx = 334.0, offset_accely = -948.0, offset_accelz = 16252.0;
//double offset_gyrox = 111.0, offset_gyroy = 25.0, offset_gyroz = -49.0;

// Valores "RAW" de tipo inteiro
int16_t raw_accelx, raw_accely, raw_accelz;
int16_t raw_gyrox, raw_gyroy, raw_gyroz;
int16_t raw_temp;

// Saídas calibradas
float accelx, accely, accelz;
float gyrox, gyroy, gyroz;
float temp;

uint8_t data[1];
uint8_t GirAcel[14];

uint8_t flag = 0, j, cont = 0;
int i;
unsigned char d;
char text[100], text1[60]={"TESTE DE CONEXAO PARA O GIROSCOPIO E O ACELEROMETRO \n\r"}; 
char text2[35]={"Erro de conexao com a MPU6050 \n\r"};
char text3[55]={"Opaaa. Eu nao sou a MPU6050, Quem sou eu? :S. I am:"};
char text4[40]={"Conexao bem sucedida com a MPU6050 \n\r"};
char text5[45]={"Oi, tudo joia?... Eu sou a MPU6050 XD \n\r"};
unsigned char cmd[1];

float timer = 0.0, t_fin = 1.0, cont_timer = 0.0;
char text6[40];
char text7[5]={"A\n"};

//I2C
void ReadI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes);
void WriteI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes);

void Print(char *data, int n);
void delay(void);

void SysTick_Wait(uint32_t n){
    SysTick->LOAD = n - 1; //15999
    SysTick->VAL = 0; //Clean the value of Systick counter
    while (((SysTick->CTRL & 0x00010000) >> 16) == 0); //Check the count flag until it's 1 
}

void SysTick_ms(uint32_t x){
    for (uint32_t i = 0; i < x; i++){//x ms
        SysTick_Wait(16000); //1ms
    }
}

extern "C"{
    void EXTI15_10_IRQHandler(void){
        EXTI->PR |= 1; //Down flag
        if(((GPIOC->IDR & (1<<13)) >> 13) == 1){
            flag = 1;
        }
    }

    void USART3_IRQHandler(void){ //Receive interrupt
        if(((USART3->ISR & 0x20) >> 5) == 1){//Received data is ready to be read (flag RXNE = 1)
            d = USART3->RDR;//Read the USART receive Data 
            if(d == 'H'){
                flag = 1;
            }
        }
    }
}

int main(){
    //----------------------------------------------------------------------------
    //                        					GPIOs
    //----------------------------------------------------------------------------
    RCC->AHB1ENR |= ((1<<1)|(1<<2)); 

    GPIOB->MODER &= ~((0b11<<0)|(0b11<<14));
    GPIOB->MODER |= ((1<<0)|(1<<14)); 
    GPIOC->MODER &= ~(0b11<<26);

    GPIOB->OTYPER &= ~((1<<0)|(1<<7));
    GPIOB->OSPEEDR |= (((1<<1)|(1<<0)|(1<<15)|(1<<14)));
    GPIOC->OSPEEDR |= ((1<<27)|(1<<26));
    GPIOB->PUPDR &= ~((0b11<<0)|(0b11<<14));
    GPIOC->PUPDR &= ~(0b11<<26);
    GPIOC->PUPDR |= (1<<27);

    //----------------------------------------------------------------------------
    //                        				Systick
    //----------------------------------------------------------------------------
    SysTick->LOAD = 0x00FFFFFF; 
    SysTick->CTRL |= (0b101);

    //----------------------------------------------------------------------------
    //                        				Interrupt
    //----------------------------------------------------------------------------
    RCC->APB2ENR |= (1<<14); 
    SYSCFG->EXTICR[3] &= ~(0b1111<<4); 
    SYSCFG->EXTICR[3] |= (1<<5); 
    EXTI->IMR |= (1<<13); 
    EXTI->RTSR |= (1<<13);
    NVIC_EnableIRQ(EXTI15_10_IRQn); 
        
    //----------------------------------------------------------------------------
    //                        					UART
    //----------------------------------------------------------------------------
    RCC->AHB1ENR |= (1<<3); 
    GPIOD->MODER |= (1<<19)|(1<<17); 
    GPIOD->AFR[1] |= (0b111<<4)|(0b111<<0); 
    RCC->APB1ENR |= (1<<18); 
    USART3->BRR = 0x683; 
    USART3->CR1 |= ((1<<5)|(0b11<<2)); 
    NVIC_EnableIRQ(USART3_IRQn);

    //----------------------------------------------------------------------------
    //                        					I2C
    //----------------------------------------------------------------------------
    RCC->AHB1ENR |= (1<<1); //Enable GPIOB clock (PB9=I2C1_SDA and PB8=I2C1_SCL)
    GPIOB->MODER |= (1<<19)|(1<<17); //Set (10) pins PB9 (bits 19:18) and PB8 (bits 17:16) as alternant function
    GPIOB->OTYPER |= (1<<9)|(1<<8); //Set (1) pin PB9 (bit 9) and pin PB8 (bit 8) as output open drain (HIGH or LOW)
    GPIOB->OSPEEDR |= (0b11<<18)|(0b11<<16); //Set (11) pin PB9 (bits 19:18) and pin PB8 (bits 17:16) as Very High Speed
    GPIOB->PUPDR|= (1<<18)|(1<<16); //Set (01) pin PB9 (bits 19:18) and pin PB8 (bits 17:16) as pull up
    GPIOB->AFR[1] |= (1<<6)|(1<<2); //Set the I2C1 (AF4) alternant function for pins PB9=I2C1_SDA (bits 7:4) and PB8=I2C1_SCL (bits 3:0)
    RCC->APB1ENR |= (1<<21); //Enable I2C1 clock
    RCC->DCKCFGR2 |= (1<<17); //Set (10) bits 17:16 as HSI clock is selected as source I2C1 clock
    I2C1->CR1 &= ~(1<<0);// Clear the enable I2C1
    I2C1->TIMINGR |= 0x30420F13;// Table 207 of reference manual
    I2C1->CR1 |= (1<<0);// Enable I2C1
		
		//----------------------------------------------------------------------------
    //                        					TIMER
    //----------------------------------------------------------------------------
		//TIMER
    RCC->APB1ENR |= (1<<1); //Enable the TIMER3 clock 
    TIM3->PSC = 24; // Prescale factor 25 for 100ms of time
    TIM3->ARR = 63999; // Maximum count value
		
		RCC->APB1ENR |= (1<<3); //Enable the TIMER5 clock 
    TIM5->PSC = 24; // Prescale factor 25 for 100ms of time
    TIM5->ARR = 10000000; // Maximum count value
    

    USART3->CR1 |= (1<<0);
    
    SysTick_ms(1000);

    //----------------------------------------------------------------------------
    //                        				MPU6050
    //----------------------------------------------------------------------------
    cmd[0] = 0x00;	
    WriteI2C1(MPU6500_address, 0x6B, cmd, 1); // Desativa modo de hibernação do MPU6050
    Print(text1, strlen(text1));
    //.....................................................................
    //        Quem sou eu para a MPU6050 (giroscópio e acelerômetro)
    //.....................................................................
    ReadI2C1(MPU6500_address, 0x75, data, 14);
    if (data[0] != 0x68) { // DEFAULT_REGISTER_WHO_AM_I_MPU6050 0x68
    Print(text2, strlen(text2));
    sprintf(text3,"%s %#x \n\r",data[0]);
    Print(text3, strlen(text3));
    while (1);
    }else{
        Print(text4, strlen(text4));
        Print(text5, strlen(text5));
    }
    SysTick_ms(100);
    //.....................................................................
    //        Configuracao dos sensores giroscópio e acelerômetro
    //.....................................................................
    cmd[0] = 0x00;
    WriteI2C1(MPU6500_address, 0x1B, cmd, 1);	
    WriteI2C1(MPU6500_address, 0x1C, cmd, 1);	
    SysTick_ms(10);
    
    while(1){
        if(flag == 1){
            flag = 0;
            i = 1;
            while(1){
								TIM5->CNT = 0;
								TIM5->CR1 |= (1<<0); // Enable Counting										
                ReadI2C1(MPU6500_address, 0x3B, GirAcel, 14);
                raw_accelx = GirAcel[0]<<8 | GirAcel[1];    
                raw_accely = GirAcel[2]<<8 | GirAcel[3];
                raw_accelz = GirAcel[4]<<8 | GirAcel[5];
                raw_temp = GirAcel[6]<<8 | GirAcel[7];
                raw_gyrox = GirAcel[8]<<8 | GirAcel[9];
                raw_gyroy = GirAcel[10]<<8 | GirAcel[11];
                raw_gyroz = GirAcel[12]<<8 | GirAcel[13];
                //SysTick_ms(1);	
                delay();
                //Dados escalados
                //accelx = raw_accelx*SENSITIVITY_ACCEL;
                //accely = raw_accely*SENSITIVITY_ACCEL;
                //accelz = raw_accelz*SENSITIVITY_ACCEL;
                //gyrox = raw_gyrox*SENSITIVITY_GYRO;
                //gyroy = raw_gyroy*SENSITIVITY_GYRO;
                //gyroz = raw_gyroz*SENSITIVITY_GYRO;
                //temp = (raw_temp/SENSITIVITY_TEMP)+21;
                TIM5->CR1 &= ~(1<<0); // Disable Counting			
                timer = TIM5->CNT*0.0000000625;
                cont_timer += timer;
                //sprintf(text6,"El tiempo es %f segundos \n", timer);
//Print(text6, strlen(text6));
                sprintf(text,"%d %.4f %.2f %.2f %.2f %.2f %.2f %.2f \n\r", i++, timer, (float)raw_accelx, (float)raw_accely, (float)raw_accelz, (float)raw_gyrox, (float)raw_gyroy, (float)raw_gyroz);
                //sprintf(text,"%d \t %.2f \t %.2f \t %.2f \t %.2f \t %.2f \t %.2f \t %.2f \n\r",i+1,accelx, accely, accelz, gyrox, gyroy, gyroz, temp);
                Print(text, strlen(text));
                if(cont_timer >= t_fin){
                    cont_timer = 0;
                    //Print(text7, strlen(text7));
                    break;
                }
								
            }
        }
    }
}

void WriteI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes){
    uint8_t n; // Count for data read
    
    I2C1->CR2 &= ~(0x3FF<<0);// Clear the slave address
    I2C1->CR2 |= (Address<<1);// Set the 7-bit slave address to be sent

    // i2c modo escritura
    I2C1->CR2 &= ~(1<<10);// Master requests a write transfer
    I2C1->CR2 &= ~(0xFF<<16);// Clear the number of bytes to be transmitted
    I2C1->CR2 |= ((bytes+1)<<16);// Set the number of bytes to be transmitted
    I2C1->CR2 |= (1<<25);// Set automatic end mode

    I2C1->CR2 |= (1<<13);// Generate START

    while (((I2C1->ISR) & (1<<1)) != (0b10)){}// Wait the (TXIS) Transmit interrupt status

    I2C1->TXDR = Register;// Transmit the register

    n = bytes;
    while(n>0){
        while (((I2C1->ISR) & (1<<1)) != (0b10)){}// Wait the (TXIS) Transmit interrupt status
        I2C1->TXDR = *Data;// Data to be sent
        Data++;
        n--;
    }
    while (((I2C1->ISR) & (1<<5)) != (0b100000)){}// Wait the STOPF detection flag
}

void ReadI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes ) {
    
    uint8_t n; // Contador para la lectura de los bytes

    // Dirección del dispositivo
    I2C1->CR2 &= ~(0x3FF<<0);// Clear the slave address
    I2C1->CR2 |= (Address<<1);// Set the 7-bit slave address to be sent

// i2c modo escritura
    I2C1->CR2 &= ~(1<<10);// Master requests a write transfer
    I2C1->CR2 &= ~(0xFF<<16);// Clear the number of bytes to be transmitted
    I2C1->CR2 |= (1<<16);// Set the number of bytes to be transmitted
    I2C1->CR2 &= ~(1<<25);// Set software end mode

    I2C1->CR2 |= (1<<13);// Generate START
    
    while (((I2C1->ISR) & (1<<1)) != (0b10)){}// Wait the (TXIS) Transmit interrupt status

    I2C1->TXDR = Register;// Transmit the register

    while (((I2C1->ISR) & (1<<6)) != (0b1000000)){}// Wait a (TC) Transfer complete

    // i2c en modo lectura
    I2C1->CR2 |= (1<<10);// Master requests a read transfer
    I2C1->CR2 &= ~(0xFF<<16);// Clear the number of bytes to be transmitted
    I2C1->CR2 |= (bytes<<16);// Set the number of bytes to be received
    I2C1->CR2 &= ~(1<<25);// Set software end mode

    I2C1->CR2 |= (1<<13);// Generate RE-START

    n = bytes;
    while (n>0){
        while (((I2C1->ISR) & (1<<2)) != (0b100)){}// Wait (RXNE) that the received data is copied into the I2C_RXDR register
        *Data = I2C1->RXDR;// Receive the register
        Data++;
        n--;
    }

    I2C1->CR2 |= (1<<14);// I2C stop

    while (((I2C1->ISR) & (1<<5)) != (0b100000)){}// Wait the STOPF detection flag
}

void Print(char *data, int n){
    for(j=0; j<n; j++){
        USART3->TDR = *data; 
        data++;
        while(((USART3->ISR & 0x80) >> 7) == 0){} 
    }
    //USART3->TDR = 0x0A; 
    //while((USART3->ISR & 0x80)==0){};
    USART3->TDR = 0x0D; 
    while(((USART3->ISR & 0x80) >> 7) == 0){}
}

void delay(void){
	TIM3->CNT = 0;
	TIM3->CR1 |= (1<<0); // Enable Counting
	//while(TIM5->CNT < 16000); //1ms	
	while(TIM3->CNT < 8000); //0.5ms
	//while(TIM5->CNT < 128000); //8.51ms=8150us
	TIM3->CR1 &= ~(1<<0); // Disable Counting	
	
	for(j=0; j<=7; j++){
		TIM3->CNT = 0;
		TIM3->CR1 |= (1<<0); // Enable Counting
		while(TIM3->CNT < 16000); //1ms	
		TIM3->CR1 &= ~(1<<0); // Disable Counting	
	}
}
```

<h3>Adquisición de datos MPU6050 en la STM32F303K8</h3>

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
    i2c.write(MPU6500_address, cmd, 2);                //ACC fullsclae 2G
    wait(0.01);
    while(1) {
        //.................Construcción de la medición de los valores .................. 
        if(pc.getc() == 'H'){
            for(i=0; i<100; i++){
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

<h3>Calibración MPU6050 en RPi</h3>

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

datos=numpy.zeros((100,8)) #bits
datos1=numpy.zeros((100,8)) #no calibrados
datos2=numpy.zeros((100,8)) # calibrados

value = input("\nQuiere adquirir los datos S/N \n\n")

if value == 'S' or value == 's':
    print("\nCapturando datos \n")
    data.write(b'H')
    for i in range(100):
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

<h3>Gráficas MPU6050 en RPi</h3>

```python
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
    #h.show()

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
    #i.show()

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
    #f.show()

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
    #g.show()

    plt.show()

else:
    print("\nAdios\n")
```