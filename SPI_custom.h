#ifndef Printing_h
	#define Printing_h
	#include "Printing.h"
#endif 

#define SPI_MOSI (DDB2)
#define SPI_MISO (DDB3)
#define SPI_SCK (DDB1)
#define SPI_SS (DDB0)

#define BCD_TO_INT(X) ((X) & 0x0F) + (10*((X) >> 4))
#define INT_TO_BCD(X) ((X) - ((X)/10)*10) | (((X) / 10) << 4)

/**
 * A structure holding time in a format with which the DS3234 is working.
 * ampm_mask holds information about whether the clock is in 12/24 mode (bit 0)
 * (1 if 12 mode, 0 if 24 mode)
 * and if in 12 mode bit 1 denotes AM/PM mode (0: AM, 1: PM)
 */
typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t ampm_mask;
} DS3234_TIME;

/**
 * A structure holding time in a format with which the DS3234 is working.
 * control holds a centuSry bit (bit 0). If bit 1 is set only one of day_of_week
 * or day_of_month values are valid, and in that case bit 2 denotes which one
 * (0: day_of_week, 1: day_of_month)
 */
typedef struct {
	uint8_t day_of_week;
	uint8_t day_of_month;
	uint8_t month;
	uint8_t year;
	uint8_t control;
} DS3234_DATE;


void SPI_init() {
	DDRB |= (1<<SPI_MOSI) | (1<<SPI_SCK) | (1<<SPI_SS);
	DDRB &= ~(1<<SPI_MISO);

	SPCR = (1<<MSTR) | (1<<CPOL) | (1<<CPHA);
	SPCR |= (1<<SPE);

	PORTB &= ~(1<<DDB0);
	PORTB |= (1<<DDB0);
}

uint8_t SPI_data_transfer(uint8_t data) {
	SPDR = data;
	while(!(SPSR & (1 <<SPIF)));
	return SPDR;
}

void SPI_slave_select() {
	PORTB &= ~(1<<SPI_SS);
}

void SPI_slave_unselect() {
	PORTB |= (1<<SPI_SS);
}

void SPI_read_time(DS3234_TIME *time) 
{
	SPI_slave_select();
	
	SPI_data_transfer(0x00); //Seconds register
	uint8_t data = SPI_data_transfer(0x00); //Dummy byte to receive seconds register
	time->seconds = BCD_TO_INT(data);
	data = SPI_data_transfer(0x00); //Dummy byte to burst receive minutes register
	time->minutes = BCD_TO_INT(data);
	data = SPI_data_transfer(0x00); //Dummy byte to burst reveive hours register
	
	if (data & (1 << 6)) {
		//Register is in 12 hour mode
		//If bit 4 is set (10hr bit) increment hour by 10 
		time->hours = (data & (1 << 4)) ? (data & 0x0F) + 10 : (data & 0x0F);
		time->ampm_mask = 1;
		time->ampm_mask |= (data & (1 << 5) >> 4); //bit 5 (AM/PM) of date shifted to bit 1 of ampm_mask
		} else {
		//Register is in 24 hour mode
		//Bit 6 and 7 are guaranteed to be zero in this mode
		time->hours = BCD_TO_INT(data);
		time->ampm_mask = 0;
	}
	
	SPI_slave_unselect();
}

void SPI_write_time(DS3234_TIME *time) 
{
	SPI_slave_select();
	SPI_data_transfer(0x80); //Write seconds register address
	SPI_data_transfer(INT_TO_BCD(time->seconds)); //Write seconds register
	SPI_data_transfer(INT_TO_BCD(time->minutes)); //Burst-write minutes register
	
	if (time->ampm_mask) {
		//The clock is in 12-hour mode;
		SPI_data_transfer(INT_TO_BCD(time->hours) | (1 << 6) | ((time->ampm_mask & 2) << 5));
		//Burst-write hours register, bit 6 meaning we are in 12 hours mode and bit 5 AM/PM value
		} else {
		//The clock is in 24-hour mode
		SPI_data_transfer(INT_TO_BCD(time->hours)); //Burst-write hours register
	}
	SPI_slave_unselect();
}

void SPI_read_date(DS3234_DATE *date) {
	uint8_t data;
	SPI_slave_select();
	SPI_data_transfer(0x03); //Day register address
	data = SPI_data_transfer(0x00); //Read day register
	date->day_of_week = BCD_TO_INT(data);
	data = SPI_data_transfer(0x00); //Burst-read day register
	date->day_of_month = BCD_TO_INT(data); //Burst-read date register
	data = SPI_data_transfer(0x03); //Burst-read month and century register
	date->month = BCD_TO_INT(data & 0x7F); //Ommit the century bit
	date->control = 0;
	if (data & 0x80) date->control |= 1;
	data = SPI_data_transfer(0x00); //Burst-read year register
	date->year = BCD_TO_INT(data);
	SPI_slave_unselect();
}

void SPI_write_date(DS3234_DATE *date) {
	SPI_slave_select();
	SPI_data_transfer(0x83); //Write day register address
	SPI_data_transfer(INT_TO_BCD(date->day_of_week));
	SPI_data_transfer(INT_TO_BCD(date->day_of_month));
	SPI_data_transfer(INT_TO_BCD(date->month));
	SPI_data_transfer(INT_TO_BCD(date->year));
	SPI_slave_unselect();
}
