/*
 * Serial_Italo.h
 *
 * Created: 6/2/2016 5:00:35 PM
 *  Author: KevinDustinSalvatore
 */ 


#ifndef SERIAL_ITALO_H_
#define SERIAL_ITALO_H_

#include <avr/io.h>
#include <avr/pgmspace.h>

//Para configuracion de USART:
void Serial_begin();
void Serial1_begin();
void Serial2_begin();
void Serial3_begin();

//Para envio de caracteres de una cadena
void Serial_write(unsigned char Cadena[]);
void Serial1_write(unsigned char Cadena[]);
void Serial2_write(unsigned char Cadena[]);
void Serial3_write(unsigned char Cadena[]);

//Para impresion de caracteres de una cadena FLASH
void Serial_print(const unsigned char cadena[]);
void Serial1_print(const unsigned char cadena[]);
void Serial2_print(const unsigned char cadena[]);
void Serial3_print(const unsigned char cadena[]);

//Para recepcion de una cadena de caracteres que termina en enter
unsigned char Serial_read();
unsigned char Serial1_read();
unsigned char Serial2_read();
//void Serial3_read(unsigned char Cadena[]);



void Serial_begin(unsigned int BaudRate)
{
	unsigned int UBRR=((F_CPU/BaudRate)>>3) -1;
	UBRR0H=UBRR>>8;
	UBRR0L=UBRR&0xFF;
	UCSR0A=1<<U2X0;
	UCSR0C=(0<<UMSEL00)|(0<<UPM01)|(0<<UPM00)|(0<<USBS0)|(1<<UCSZ01)|(1<<UCSZ00); //Sin paridad un bit de parada y 8bits de datos
	UCSR0B=(1<<RXEN0)|(1<<TXEN0)|(0<<UCSZ02);
}

void Serial1_begin(unsigned int BaudRate)
{
	unsigned int UBRR=((F_CPU/BaudRate)>>3) -1;
	UBRR1H=UBRR>>8;
	UBRR1L=UBRR&0xFF;
	UCSR1A=1<<U2X1;
	UCSR1C=(0<<UMSEL10)|(0<<UPM11)|(0<<UPM10)|(0<<USBS1)|(1<<UCSZ11)|(1<<UCSZ10); //Sin paridad un bit de parada y 8bits de datos
	UCSR1B=(1<<RXEN1)|(1<<TXEN1)|(0<<UCSZ12);
}

void Serial2_begin(unsigned int BaudRate)
{
	unsigned int UBRR=((F_CPU/BaudRate)>>3) -1;
	UBRR2H=UBRR>>8;
	UBRR2L=UBRR&0xFF;
	UCSR2A=1<<U2X2;
	UCSR2C=(0<<UMSEL20)|(0<<UPM21)|(0<<UPM20)|(0<<USBS2)|(1<<UCSZ21)|(1<<UCSZ20); //Sin paridad un bit de parada y 8bits de datos
	UCSR2B=(1<<RXEN2)|(1<<TXEN2)|(0<<UCSZ22);
}

void Serial3_begin(unsigned int BaudRate)
{
	unsigned int UBRR=((F_CPU/BaudRate)>>3) -1;
	UBRR3H=UBRR>>8;
	UBRR3L=UBRR&0xFF;
	UCSR3A=1<<U2X3;
	UCSR3C=(0<<UMSEL30)|(0<<UPM31)|(0<<UPM30)|(0<<USBS3)|(1<<UCSZ31)|(1<<UCSZ30); //Sin paridad un bit de parada y 8bits de datos
	UCSR3B=(1<<RXEN3)|(1<<TXEN3)|(0<<UCSZ32);
}


void Serial_write(unsigned char Cadena[])
{
	unsigned char i;
	for(i=0; Cadena[i]; i++) {
		while( !(UCSR0A & (1<<UDRE0)) ); // Espera que se libere el buffer de transmisión
		UDR0 = Cadena[i];
	}
}

void Serial1_write(unsigned char Cadena[])
{
	unsigned char i;
	for(i=0; Cadena[i]; i++) {
		while( !(UCSR1A & (1<<UDRE1)) ); // Espera que se libere el buffer de transmisión
		UDR1 = Cadena[i];
	}
}

void Serial2_write(unsigned char Cadena[])
{
	unsigned char i;
	for(i=0; Cadena[i]; i++) {
		while( !(UCSR2A & (1<<UDRE2)) ); // Espera que se libere el buffer de transmisión
		UDR2 = Cadena[i];
	}
}

void Serial3_write(unsigned char Cadena[])
{
	unsigned char i;
	for(i=0; Cadena[i]; i++) {
		while( !(UCSR3A & (1<<UDRE3)) ); // Espera que se libere el buffer de transmisión
		UDR3 = Cadena[i];
	}
}


void Serial_print(const unsigned char cadena[])
{
	unsigned char i;
	for (i = 0; pgm_read_byte(&cadena[i]); i++)
	{
		while( !(UCSR0A & (1<<UDRE0)) ); // Espera que se libere el buffer de transmisión
		UDR0 = pgm_read_byte(&cadena[i]);
	}
}

void Serial1_print(const unsigned char cadena[])
{
	unsigned char i;
	for (i = 0; pgm_read_byte(&cadena[i]); i++)
	{
		while( !(UCSR1A & (1<<UDRE1)) ); // Espera que se libere el buffer de transmisión
		UDR1 = pgm_read_byte(&cadena[i]);
	}
}

void Serial2_print(const unsigned char cadena[])
{
	unsigned char i;
	for (i = 0; pgm_read_byte(&cadena[i]); i++)
	{
		while( !(UCSR2A & (1<<UDRE2)) ); // Espera que se libere el buffer de transmisión
		UDR2 = pgm_read_byte(&cadena[i]);
	}
}

void Serial3_print(const unsigned char cadena[])
{
	unsigned char i;
	for (i = 0; pgm_read_byte(&cadena[i]); i++)
	{
		while( !(UCSR3A & (1<<UDRE3)) ); // Espera que se libere el buffer de transmisión
		UDR3 = pgm_read_byte(&cadena[i]);
	}
}

unsigned char Serial_read()
{
	unsigned char datoRX=0;
	//unsigned char i=0;
	//while (datoRX!=13) {
	while((UCSR0A&(1<<RXC0))==0);
	datoRX=UDR0;
	return datoRX;
	//Cadena[i++]=datoRX;
	//}
	//Cadena[i]=0;
}

unsigned char Serial1_read()
{
	unsigned char datoRX=0;
	//unsigned char i=0;
	//while (datoRX!=13)
	while((UCSR1A&(1<<RXC1))==0);
	
	datoRX=UDR1;
	return datoRX;
	//Cadena[i++]=datoRX;
	//}
	//Cadena[i]=0;
}

unsigned char Serial2_read(void)
{
	unsigned char datoRX=0;
	//unsigned char i=0;
	//while (datoRX!=13) {
	while((UCSR2A&(1<<RXC2))==0);
	datoRX=UDR2;
	return datoRX;
	//Cadena[i++]=datoRX;
	//}
	//Cadena[i]=0;
}

void Serial3_read(unsigned char Cadena[])
{
	unsigned char datoRX=0;
	unsigned char i=0;
	while (datoRX!=13) {
		while((UCSR3A&(1<<RXC3))==0);
		datoRX=UDR3;
		Cadena[i++]=datoRX;
	}
	Cadena[i]=0;
}

void TxByte(unsigned char dato){
	while ((UCSR0A & (1<<UDRE0)) == 0);
	UDR0 = dato;
}
void TxBCD(int numero){
	TxByte(numero/1000 + '0');
	TxByte((numero/100)%10 + '0');
	TxByte((numero/10)%10 + '0');
	TxByte(numero%10 + '0');
}




#endif /* SERIAL_ITALO_H_ */