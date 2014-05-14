void debug_statement(int print)
{
	// Wait for empty transmit buffer
	while ( !( UCSR1A & (1<<UDRE1))){};
	
	// Put data into buffer, sends the data
	UDR1 = print;

	return;
}


void debug_statement(char print[])
{
	char *s = print;
	
	while(*s)
	{
		// Wait for empty transmit buffer
		while ( !( UCSR1A & (1<<UDRE1))){};
		
		// Put data into buffer, sends the data
		UDR1 = *s;
		++s;
	}
	return;
}

void ERROR(int error_code)
{
	char check_buffer[256];
	
	switch(error_code)
	{
		case 0x08: debug_statement("0x08: Start condition has been transmitted \r\n"); break;
		case 0x10: debug_statement("0x10: Repeated start condition has been transmitted \r\n"); break;
		case 0x18: debug_statement("0x18: SLA+W has been transmitted, ACK has been recived \r\n"); break;
		case 0x20: debug_statement("0x20: SLA+W has been transmitted, NO ACK was recieved \r\n"); break;
		case 0x28: debug_statement("0x28: Data byte has been transmitted, ACK was recived \r\n"); break;
		case 0x30: debug_statement("0x30: Data byte has been transmitted, NO ACK was recived \r\n");break;
		case 0x38: debug_statement("0x38: Arbitition lost in SLA+R or NO ACK bit \r\n"); break;
		case 0x40: debug_statement("0x40: The SLA+R has been trnasmitted and ACK has been recieved \r\n"); break;
		case 0x48: debug_statement("0x48: The SLA+R has been transmitted but NO ACK was recieved \r\n"); break;
		case 0x50: debug_statement("0x50: Data bye has been recived, ACK has been returned \r\n"); break;
		case 0x58: debug_statement("0x58: Data byte has been recived, NOT ACK has been returned \r\n"); break;
		default: 
		{
				sprintf(check_buffer, "Unknown error: %x \r\n", error_code);
				debug_statement(check_buffer);
		}
	}
	return;
}

void ERROR(void)
{
	debug_statement("Housten, we have an error!\r\n");
	return;
}
