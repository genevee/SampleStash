/*
 * Serial_mega.cpp
 *
 * Created: 1/24/2014 9:36:21 PM
 *  Author: Genevieve
 */ 

//Can be found in the AVR/Arduino firmware downloads
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include "pins_arduino.h"
#include <inttypes.h>
#include "twi.h"
#include <math.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <compat/twi.h>

//My own files, must be manually downloaded
#include "i2c.h"

#ifndef Printing_h
#define Printing_h
#include "Printing.h"
#endif

#define FOSC 1843200// Clock Speed

#define BAUD1 9600 //Desired USART baud rate
#define MYUBRR1 ((F_CPU/16)/BAUD1-1) //

#define BAUD2 57600 //Secondary baud RATE
#define MYUBRR2 ((F_CPU/16)/BAUD2-1)

//I2C addresses for sensors
#define ACC_W (0xA6) //Acceleromter ADXL345
#define ACC_R (0xA7)
#define MAG_W (0x3C) //Magnetometer HCM5883L
#define MAG_R (0x3D)
#define ALT_W (0xC0) //Altimeter MPL3115A2
#define ALT_R (0xC1)
#define GYRO_W (0xD0)//0x68 //Gyropscope ITG-3200
#define GYRO_R (0xD1) //0x69

//Specific, useful addresses to be written at sensor initilizatoin 
#define ACC_FORMAT (0x31) //Write 0x01 to put in the 4G range
#define ACC_POWER_CONTROL (0x2D) // Write 0x08 to put in measurement mode

//Addresses of the accelerator measurement registers 
#define ACC_X1 (0x32)
#define ACC_X2 (0x33)
#define ACC_Y1 (0x34)
#define ACC_Y2 (0x35)
#define ACC_Z1 (0x36)
#define ACC_Z2 (0x37)

//Addresses of the magnetometer measurement and initilization registers 
#define MAG_CONFIG1 (0x00)
#define MAG_CONFIG2 (0x01)
#define MAG_MODE (0x02)
#define MAG_X1 (0x03)
#define MAG_X2 (0x04)
#define MAG_Y1 (0x05)
#define MAG_Y2 (0x06)
#define MAG_Z1 (0x07)
#define MAG_Z2 (0x08)

#define GYRO_X1 (0x1D)
#define GYRO_X2 (0x1E)
#define GYRO_Y1 (0x1F)
#define GYRO_Y2 (0x20)
#define GYRO_Z1 (0x21)
#define GYRO_Z2 (0x22)
#define GYRO_POWER (0x3E)

#define ALT_BARO_ENABLE (0x26) //ALT is calculated from BARO, with default atmostpheric preSerialure of 101,326 Pa
#define ALT_ENABLE (0x26) //Set to value 0xB8
#define ALT_ENABLE_FLAGS (0x13) //Set to value 0x07
#define ALT_STATUS (0x00)
#define ALT_P1 (0x01)
#define ALT_P2 (0x02)
#define ALT_P3 (0x03)
#define ALT_T1 (0x04)
#define ALT_T2 (0x05)

//Functions to initialize USART1 and 2, and read and write to both
void USART_Init2( unsigned int ubrr);
void USART_Init1( unsigned int ubrr);
unsigned char USART1_Read(void);
void USART1_Write(unsigned char data);

//Functions to read and write in Master Mode to I2C
void MASTER_write(int device_w, int reg, int data);
int MASTER_read(int device_r, int device_w, int reg);
int BITWISE_rect(int value1, int value2);
int* MASTER_multiple_read(int* bytes, int device_r, int device_w, int reg_start, int reg_to_read);

//Debugging function, to print an array of 6 values to a serial port
void BITWISE_print(int* read_ptr);

//Initializes and read from ADC1
void ADC_init();
uint8_t ADC_read(int channel);

//Initializes the sets time on Real Time Clock sensor
int RTC_init();
int SetTimeDate(int d, int mo, int y, int h, int mi, int s);
char* ReadTimeDate();

//Blinks on board LED for debugging purposes
void Blink_LED(void);

//Character array for debugging purpouses
char stringing[256];

int main(void)
{
	sei(); //Enable global interupt
	
	//Initizlize int arrays to hold the measured values
	char GPS_read [256];
	char* GPS_read_ptr = &(GPS_read[0]);
	
	int mag_read[6];
	int* mag_read_ptr = &(mag_read[0]);
	
	int acc_read[6];
	int* acc_read_ptr = &(acc_read[0]);
	
	int gyro_read[6];
	int* gyro_read_ptr = &(gyro_read[0]);
	
	int alt_baro_read[5];
	int* alt_baro_read_ptr = &(alt_baro_read[0]);	
	
	DS3234_DATE read_date;
	DS3234_TIME read_time;
	
	char str[255]; //Initialized debug string for UART
	
	USART_Init1(MYUBRR2); //Initilize UART periferhal
	USART_Init2(MYUBRR2); //Initilize UART periferhal
		
	debug_statement("Initializing USART and I2C...");
	
	DDRB |= (1 <<5 ); //Set Port D as digital output	
	i2c_init(); //Initialize I2C peripheral

	debug_statement("...done \r\n");
		
	//debug_statement("-----------Initializing ADC---------\r\n");
	ADC_init(); //Initializing ADC on Channel 0
	ADC_init(1); //Initializing ADC on Channel 0

	//debug_statement("-----------Initiailzing IO---------\r\n");
	//PORTF Pin 0 analog input
	DDRF = 0x00;
	DDRA |= (1 << DDA0);
		 
	/*debug_statement("-----------Writing MAG Format---------\r\n");
	MASTER_write(MAG_W, MAG_MODE, 0x00);
	
	debug_statement("-----------Writing GYRO Format---------\r\n");*/
	MASTER_write(GYRO_W, GYRO_POWER, 0x00);
		
	//debug_statement("-----------Writing ACC Format---------\r\n");
	MASTER_write(ACC_W, ACC_FORMAT, 0x01);
	MASTER_write(ACC_W, ACC_POWER_CONTROL, 0x08);

	/*debug_statement("-----------Writing ALT/BARO Format---------\r\n");
	MASTER_write(ALT_W, ALT_ENABLE, 0xB8);
	MASTER_write(ALT_W, ALT_ENABLE_FLAGS, 0x07);
	MASTER_write(ALT_W, ALT_BARO_ENABLE, 0xB9);*/

	//debug_statement("-----------Initiazling RTC---------\r\n");
	SPI_init();
	
	read_time.seconds = 15;
	read_time.minutes = 13;
	read_time.hours = 11;
	read_time.ampm_mask = 0;
	SPI_write_time(&read_time);
	
	debug_statement("Blinking");
	Blink_LED();
	
	debug_statement("Entering Loop");
	
	
	while(1)
	{			
		Blink_LED();
		
		//debug_statement("-----------Entering reading loop---------\r\n");
		
		//debug_statement("-----------Reading RTC---------\r\n");	
		/*SPI_read_time(&read_time);
		uint8_t seconds = read_time.seconds;
		uint8_t minutes = read_time.minutes;
		uint8_t hours = read_time.hours;
		uint8_t am_pm = read_time.ampm_mask;
		sprintf(str, "     %d : %d : %d in the ( %d) AM/PM" , hours, minutes, seconds, am_pm);
		debug_statement(str);
		debug_statement("\r\n");*/
				
		
		
		/*
		//debug_statement("-----------Reading MAG---------\r\n");		
		mag_read_ptr = MASTER_multiple_read(mag_read, MAG_R, MAG_W, MAG_X1, 6);*/
		
		//debug_statement("-----------Reading ACC---------\r\n");
		//acc_read_ptr = MASTER_multiple_read(acc_read, ACC_R, ACC_W, ACC_X1, 6);
		
		//debug_statement("-----------Reading GYRO-----------");	
		/*gyro_read_ptr = MASTER_multiple_read(gyro_read, GYRO_R, GYRO_W, GYRO_X1, 6);
	
		//debug_statement("-----------Reading ALT-----------");			
		int alt_status_read = MASTER_read(ALT_R, ALT_W, ALT_STATUS);
		
		while (alt_status_read & 0x08 != true)
			alt_status_read = MASTER_read(ALT_R, ALT_W, ALT_STATUS);
			
		alt_baro_read_ptr = MASTER_multiple_read(alt_baro_read, ALT_R, ALT_W, ALT_P1, 5);
			*/
			
		//debug_statement("-----------Reading AMBIENT LIGHT-----------");	
		//uint8_t ambient_light = ADC_read(0);
		//uint8_t ambient_light = 0;
		
		
		//debug_statement("-----------Reading MICROPHONE LEVEL-----------");
		//uint8_t sound_level = ADC_read(1);
		//uint8_t sound_level = 0;
		
		//Printing Values
		debug_statement("MAG: ");
		BITWISE_print(mag_read_ptr);*/
		
		//debug_statement("ACC: ");
		//BITWISE_print(acc_read_ptr);
		
		/*debug_statement("GYRO: ");
		BITWISE_print(gyro_read_ptr);	
		
		debug_statement("ALT: ");
		unsigned long int PRES = (int)(((*(alt_baro_read_ptr+2)) << 16) | ((*(alt_baro_read_ptr+1)) << 8) | (*(alt_baro_read_ptr+0)));
		unsigned int TEMP = (int)(((*(alt_baro_read_ptr+4)) << 8) | (*(alt_baro_read_ptr+3)));
		
		sprintf(str, "PRES : %d ", PRES);
		debug_statement(str);
		sprintf(str, "TEMP : %d ", TEMP);
		debug_statement(str);
		
	//	sprintf(str, "AMB LT : %d ", ambient_light);
	//	debug_statement(str);
	
		//if (ambient_light < 75)
			//PORTB = (0 << 5);
		//sprintf(str, "SOUND LEVEL : %d ", sound_level);
		//debug_statement(str);
		

		//debug_statement("\r\n");
		_delay_ms(100);
		
	}//End while
}//End main


void USART_Init1( unsigned int ubrr){
	
	/* Set baud rate */
	UBRR1H = (uint8_t)(ubrr>>8);
	UBRR1L = (uint8_t) ubrr;
	
	/* Enable receiver and transmitter */
	UCSR1B = (1<<RXEN1)|(1<<TXEN1)|(1<< RXCIE1);
	
	/* Set frame format: 8data, 2stop bit */
	UCSR1C = (1 << USBS1)| (1 << UCSZ10) | (1 << UCSZ11);
	
} // USART_Init


void Blink_LED(void)
{
	
	DDRA = (1 << DDA0);
	
	PORTA = (1 << DDA0);
	_delay_ms(100);
	PORTA = (0 << DDA0);
	_delay_ms(100);
		
}
void USART_Init2( unsigned int ubrr){
	
	/* Set baud rate */
	UBRR2H = (uint8_t)(ubrr>>8);
	UBRR2L = (uint8_t) ubrr;
	
	/* Enable receiver and transmitter */
	UCSR2B = (1<<RXEN2)|(1<<TXEN2);
	
	/* Set frame format: 8data, 2stop bit */
	UCSR2C = (1 << USBS2)| (1 << UCSZ20) | (1 << UCSZ21);
	
} // USART_Init

unsigned char USART1_Read(void)
{
	while(!(UCSR1A & (1 << RXC1)));
	
	return UDR1;
}

void USART1_Write(unsigned char data)
{
	while ( !( UCSR1A & (1<<UDRE1))){};
	
	UDR1 = data;
}

//Initializes ADC on Channel 0
void ADC_init(void)
{
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescaler to 128 - 125KHz sample rate @ 16MHz

	ADMUX |= (1 << REFS0); // Set ADC reference to AVCC
	ADMUX |= (1 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading

	// No MUX values needed to be changed to use ADC0
	ADCSRA |= (1 << ADEN);  // Enable ADC

	//ADCSRA |= (1 << ADIE);  // Enable ADC Interrupt
	//sei();   // Enable Global Interrupts	
}

//Starts ADC conversions, and returns the value in the initialized ADC channel
uint8_t ADC_read(int channel)
{	
		//if (channel == 0){}
			
		//else if (channel == 1)
		//ADMUX |= (1 << 1);
	
		
		ADCSRA |= (1 << ADSC);  // Start A2D Conversions
		while (ADCSRA & (1 << ADSC)){} //Wait for conversion to finish
		
		return ADCH;
}

void MASTER_write(int device_w, int reg, int data)
{
	i2c_start(device_w);//Enters Master Transmiter mode
	i2c_write(reg);
	i2c_write(data);
	i2c_stop();
}
int MASTER_read(int device_r, int device_w, int reg)
{
	i2c_start(device_w);//Enters Master Transmiter mode
	i2c_write(reg);
	i2c_start(device_r);
	int value = i2c_read_nack();
	i2c_stop();
	
	return value;
}

int BITWISE_rect(int value1, int value2)
{
		return(((int)value2) << 8) | value1;;
}

void BITWISE_print(int* read_ptr)
{	
	char str[256];          //string buffer to transform data before sending it to the serial port
	int x = (int)(((*(read_ptr+1)) << 8) | (*(read_ptr+0)));
	int y = (int)(((*(read_ptr+3)) << 8) | (*(read_ptr+2)));
	int z = (int)(((*(read_ptr+5)) << 8) | (*(read_ptr+4)));
	
	sprintf(str, "X : %d ", x);
	debug_statement(str);
	sprintf(str, "Y : %d ", y);
	debug_statement(str);
	sprintf(str, "Z : %d    |", z);
	debug_statement(str);
	
	return;
	
}

int* MASTER_multiple_read(int* bytes, int device_r, int device_w, int reg_start, int reg_to_read)
{
	
	i2c_start(device_w);//Enters Master Transmiter mode
	i2c_write(reg_start);
	i2c_start(device_r);
	
	for(int i = 0; i < reg_to_read-1; ++i)
		*(bytes+i) = i2c_read_ack();
		
	*(bytes+reg_to_read) = i2c_read_nack();
	i2c_stop();
	
	return bytes;
}







