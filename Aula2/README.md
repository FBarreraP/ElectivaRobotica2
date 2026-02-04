<h1>Aula 2</h1>

Esta clase consiste en comprender la IMU de 6 GDL y adquirir la información que proporciona el acelerómetro y el giroscopio.

<h2>IMU</h2>

Las Unidades de Medición Inercial (IMUs) son dispositivos electrónicos que proporcionan mediciones de velocidad angular y fuerza gravitacional en diferentes ejes. Algunas IMUs pueden medir campo magnético, temperatura, presión atmosférica, entre otras variables físicas.

<div align="center">
<img src="Imagenes/image-2.png" alt="IMU"/>
</div>

Una IMU detecta las variaciones de fuerza gravitacional ($g=m/s^2$) y velocidad angular  ($°/s$), además del campo magnético terrestre (T o G), a través de los acelerómetros, giroscopios y magnetómetros. Los ángulos de Euler pueden ser estimados por fusión sensorial. Estos sensores en una IMU son de tecnología MEMS.

<div align="center">
<img src="Imagenes/image-3.png" alt="IMU"/>
<br>
<figcaption>Fuente: </figcaption>
</div>

<h3>Giroscopio</h3>

<div align="center">
<img src="Imagenes/image-8.png" alt="Giroscopio MEMS"/>
<br>
<figcaption>Fuente: https://cursos.mcielectronics.cl/2019/06/18/giroscopio/</figcaption>
</div>

<h3>Acelerómetro</h3>

<div align="center">
<img src="Imagenes/image-9.png" alt="Funcionamiento del Acelerómetro"/>
<br>
<figcaption>Fuente: http://www.prometec.net/imu-mpu6050/</figcaption>

<img src="Imagenes/image-10.png" alt="Acelerómetros MEMS"/>
</div>

Además, los acelerómetros permiten detectar diferentes movimientos de un dispositivo

<div align="center">
<img src="Imagenes/image-12.png" alt="Movimientos acelerómetros"/>
</div>

<h3>Magnetómetro</h3>

<div align="center">
<img src="Imagenes/image-11.png" alt="Magnetómetro MEMS"/>
<br>
<figcaption>Fuente: </figcaption>
</div>

<h3>MPU6050</h3>

La MPU6050 es un dispositivo de 6 GDL que integra acelerómetro y giroscopio y cuenta con comunicación SPI e I2C.

<img src="Imagenes/image-1.png" alt="MPU6050 y MPU9250"/>
<figcaption>Fuente: Datasheet MPU6050 y MPU9250</figcaption>

<img src="Imagenes/image-4.png" alt="MPU6050 y MPU9250"/>

<h3>I2C</h3>

I2C (Inter Integrated Circuits) es una comunicación tipo ‘half duplex’ que puede ser realizada entre uno o más maestros y uno o más esclavos, a partir de una señal de reloj (SCL) y una señal de datos (SDA). El direccionamiento de los datos se realiza a través de la dirección de esclavo de 7 bits.

<img src="Imagenes/image-5.png" alt="Conexión I2C"/>
<figcaption>Fuente: https://howtomechatronics.com/tutorials/arduino/how-i2c-communication-works-and-how-to-use-it-with-arduino/
</figcaption>
<br>

<h4>Escribir I2C</h4>

<img src="Imagenes/image-6.png" alt="Escribir I2C"/>
<figcaption>Fuente: https://howtomechatronics.com/tutorials/arduino/how-i2c-communication-works-and-how-to-use-it-with-arduino/
</figcaption>
<br>

<h4>Leer I2C</h4>

<img src="Imagenes/image-7.png" alt="Leer I2C"/>
<br>

<h3>STM32F767ZI</h3>

La información de apoyo puede ser consultada en los manuales de la tarjeta, sin embargo, <a href="https://os.mbed.com/platforms/ST-Nucleo-F767ZI/">aquí</a> también se puede encontrar alguna información desde la página de Mbed.

<div align="center">
<img src="image-4.png" alt="CN8 y CN9"/>
<br>
<img src="image-5.png" alt="CN7 y CN10"/>
<br>
Fuente: https://os.mbed.com/platforms/ST-Nucleo-F767ZI/
</div>

<h4>Pines Morphos</h4>

<div align="center">
<img src="image-6.png" alt="CN11"/>
<br>
<img src="image-7.png" alt="CN12"/>
<br>
Fuente: https://os.mbed.com/platforms/ST-Nucleo-F767ZI/
</div>

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
double offset_accelx = 334.0, offset_accely = -948.0, offset_accelz = 16252.0;
double offset_gyrox = 111.0, offset_gyroy = 25.0, offset_gyroz = -49.0;

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
char text[50], text1[60]={"TESTE DE CONEXAO PARA O GIROSCOPIO E O ACELEROMETRO \n\r"}; 
char text2[35]={"Erro de conexao com a MPU6050 \n\r"};
char text3[55]={"Opaaa. Eu nao sou a MPU6050, Quem sou eu? :S. I am:"};
char text4[40]={"Conexao bem sucedida com a MPU6050 \n\r"};
char text5[45]={"Oi, tudo joia?... Eu sou a MPU6050 XD \n\r"};
unsigned char cmd[1];

float timer = 0.0;
char text6[40];

//I2C
void ReadI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes);
void WriteI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes);

void Print(char *data, int n);

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
    TIM5->ARR = 63999; // Maximum count value
    

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
            for(i=0; i<=299; i++){
								TIM3->CNT = 0;
								TIM3->CR1 |= (1<<0); // Enable Counting										
                ReadI2C1(MPU6500_address, 0x3B, GirAcel, 14);
                raw_accelx = GirAcel[0]<<8 | GirAcel[1];    
                raw_accely = GirAcel[2]<<8 | GirAcel[3];
                raw_accelz = GirAcel[4]<<8 | GirAcel[5];
                raw_temp = GirAcel[6]<<8 | GirAcel[7];
                raw_gyrox = GirAcel[8]<<8 | GirAcel[9];
                raw_gyroy = GirAcel[10]<<8 | GirAcel[11];
                raw_gyroz = GirAcel[12]<<8 | GirAcel[13];
                //SysTick_ms(1);	
//							TIM5->CNT = 0;
//							TIM5->CR1 |= (1<<0); // Enable Counting
//							while(TIM5->CNT < 33000); //2ms
//							//while(TIM5->CNT < 136160); //8150us
//							TIM5->CR1 &= ~(1<<0); // Disable Counting	
                //Dados escalados
                //accelx = raw_accelx*SENSITIVITY_ACCEL;
                //accely = raw_accely*SENSITIVITY_ACCEL;
                //accelz = raw_accelz*SENSITIVITY_ACCEL;
                //gyrox = raw_gyrox*SENSITIVITY_GYRO;
                //gyroy = raw_gyroy*SENSITIVITY_GYRO;
                //gyroz = raw_gyroz*SENSITIVITY_GYRO;
                //temp = (raw_temp/SENSITIVITY_TEMP)+21;
								TIM3->CR1 &= ~(1<<0); // Disable Counting			
								timer = TIM3->CNT*0.0000000625;
								//timer = TIM3->CNT;
								sprintf(text6,"El tiempo es %f segundos \n", timer);
                sprintf(text,"%d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \n",i+1,raw_accelx, raw_accely, raw_accelz, raw_gyrox, raw_gyroy, raw_gyroz, raw_temp);
                //sprintf(text,"%d \t %.2f \t %.2f \t %.2f \t %.2f \t %.2f \t %.2f \t %.2f \n\r",i+1,accelx, accely, accelz, gyrox, gyroy, gyroz, temp);
                Print(text6, strlen(text6));
								Print(text, strlen(text));
								
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
```

<h3>STM32F303K8</h3>

La información de apoyo puede ser consultada en los manuales de la tarjeta, sin embargo, <a href="https://os.mbed.com/platforms/ST-Nucleo-F303K8/">aquí</a> también se puede encontrar alguna información desde la página de Mbed.

<img src="Imagenes/image.png" alt="STM32F303K8"/>
<figcaption>Fuente: https://os.mbed.com/platforms/ST-Nucleo-F303K8/</figcaption>
<br>

Para la versiones de Mbed 5 y 6, hay algunas APIs <a href="https://os.mbed.com/docs/mbed-os/v6.16/apis/index.html">aquí</a> 

<h3>Ejercicio 1</h3>

Adquirir los datos del acelerómetro y giroscopio de la IMU (MPU6050) con la STM32.

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
            for(i=0; i<299; i++){
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
                //Dados escalados
                // accelx = raw_accelx*SENSITIVITY_ACCEL;
                // accely = raw_accely*SENSITIVITY_ACCEL;
                // accelz = raw_accelz*SENSITIVITY_ACCEL;
                // gyrox = raw_gyrox*SENSITIVITY_GYRO;
                // gyroy = raw_gyroy*SENSITIVITY_GYRO;
                // gyroz = raw_gyroz*SENSITIVITY_GYRO;
                // temp = (raw_temp/SENSITIVITY_TEMP)+21;
                // wait_us(8363);
                t.stop();
                timer = t.read();
                pc.printf("El tiempo es %f segundos \r", timer);
                pc.printf("%d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \n\r",i+1,raw_accelx, raw_accely, raw_accelz, raw_gyrox, raw_gyroy, raw_gyroz, raw_temp);
                //pc.printf("%d \t %.2f \t %.2f \t %.2f \t %.2f \t %.2f \t %.2f \t %.2f \n\r",i+1,accelx, accely, accelz, gyrox, gyroy, gyroz, temp);
            }
        }
    }
}
```