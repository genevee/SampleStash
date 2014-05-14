#ifndef Printing_h
	#define Printing_h
	#include "Printing.h"
#endif




#define START 		0x08
#define START_REP 	0x10
#define MT_SLA_ACK	0x18
#define MT_DATA_ACK 	0x28
#define MR_SLA_ACK  	0x40

void i2c_init(void)
{
	//debug_statement("Begin I2C Initialized \r\n");
	//Set SCL to 400 kHz
	
	TWSR = 0x00;
	TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;
	
	//Enable TWI
	TWCR = (1 << TWEN);
	ERROR((int)TWSR);
	//debug_statement("End I2C Initialized \r\n");
}


unsigned char i2c_start (unsigned char address)		/* i2c_start */
{
	
	//debug_statement("Beginning of I2C start\r\n");

	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);		//Send START condition
	while (!(TWCR & (1<<TWINT)));					//Wait for TWINT flag set. This indicates that the START condition has been transmitted
	ERROR((int)TWSR);
	
	if ((TWSR & 0xF8) != START) //Status code should be 0x08
		ERROR();
	
	TWDR = address;								//Load SLA_W into TWDR Register.
	TWCR = (1<<TWINT) | (1<<TWEN);				//Clear TWINT bit in TWCR to start transmission of address
	
	while (!(TWCR & (1<<TWINT)));					//Wait for TWINT flag set. This indicates that the SLA+W has been transmitted, and ACK/NACK has been received.
	ERROR((int)TWSR);
	
	if ((TWSR & 0xF8) != MT_SLA_ACK)				//Check value of TWI Status Register. Mask prescaler bits. If status different from
		ERROR();
		
	//debug_statement("End of Start I2C \r\n");									//MT_SLA_ACK go to ERROR
	return TWDR;
	
}

unsigned char i2c_start_rep (unsigned char address)
{
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);		//Start again
	while (!(TWCR & (1<<TWINT)));
	ERROR((int)TWSR);
	
	if ((TWSR & 0xF8) != START_REP)
	return 0;
	
	TWDR = address;								//Load SLA_W into TWDR Register.
	TWCR = (1<<TWINT) | (1<<TWEN);

	while (!(TWCR & (1<<TWINT)));

	if ((TWSR & 0xF8) != MR_SLA_ACK)				//Check value of TWI Status Register. Mask prescaler bits. If status different from
	return 0;

	return 1;
}

void i2c_stop(void)
{
	//debug_statement("Beginning of I2C stop\r\n");
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);

	while(TWCR & (1<<TWSTO));

	//debug_statement("End of I2C stop\r\n");
	
	return;
}


unsigned char i2c_write (unsigned char data)
{
	//debug_statement("Begenning of I2C send data\r\n");
	
	TWDR = data;								//Load Pointer into TWDR Register. Clear
	
	TWCR = (1<<TWINT) | (1<<TWEN);				//TWINT bit in TWCR to start transmission of data

	while (!(TWCR & (1<<TWINT))){}
	ERROR((int)TWSR);


	if ((TWSR & 0xF8) != MT_DATA_ACK)
		ERROR();
	
	//debug_statement("End of I2C send data\r\n");
	
	return 1;
	
}

unsigned char i2c_read_ack(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while (!(TWCR & (1<<TWINT)));
	ERROR((int)TWSR);
	

	return TWDR;
}

unsigned char i2c_read_nack(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
	ERROR((int)TWSR);

	return TWDR;
}
