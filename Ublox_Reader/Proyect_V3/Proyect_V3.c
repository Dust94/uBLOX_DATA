/*
 * Proyect_V3.c
 *
 * Created: 02/06/2016 03:41:52 p.m.
 *  Author: u_mecatronica
 */ 


 #define sbi(x,y) x |= _BV(y)
 #define cbi(x,y) x &= ~(_BV(y))

#define _OPEN_SYS_ITOA_EXT
#define F_CPU 16000000UL
#define MAX_LENGTH 512


#define  POSLLH_MSG  0x02
#define  SBAS_MSG    0x32
#define  VELNED_MSG  0x12
#define  STATUS_MSG  0x03
#define  SOL_MSG     0x06
#define  DOP_MSG     0x04
#define  DGPS_MSG    0x31
#define	 TRUE		 1
#define	 FALSE		 0

#define LONG(X)    *(long*)(&data[X])
#define ULONG(X)   *(unsigned long*)(&data[X])
#define INT(X)     *(int*)(&data[X])
#define UINT(X)    *(unsigned int*)(&data[X])


#include <avr/interrupt.h>
#include <math.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include "Uart_Lord_Anthony.h"
//#include "PString.h"





unsigned char  state, lstate, code, id, chk1, chk2, ck1, ck2;
unsigned int  length, idx, cnt, len_cmdBuf;

unsigned char data[MAX_LENGTH];

long lastTime = 0;
int checkOk = 0;


/*
 * uBlox UBX Protocol Reader (runs on Arduino Leonardo, or equivalent)
 *
 * Note: RX pad on 3DR Module is output, TX is input
 */
	
	
void enableMsg (unsigned char id, uint8_t enable) {

  //               MSG   NAV   < length >  NAV
  if (enable == 1){
  uint8_t cmdBuf[] = {0x06, 0x01, 0x03, 0x00, 0x01, id, 0x1};
  len_cmdBuf = sizeof(cmdBuf) / sizeof(cmdBuf[0]);
  sendCmd(len_cmdBuf, cmdBuf);
  
  }
  else{ 
  uint8_t cmdBuf[] = {0x06, 0x01, 0x03, 0x00, 0x01, id, 0x0};
  len_cmdBuf = sizeof(cmdBuf) / sizeof(cmdBuf[0]);
  sendCmd(len_cmdBuf, cmdBuf);
  }
  
}

void setup() {
	
  Serial_begin(9600);
  Serial2_begin(38400);	
  
  if (FALSE) {
    while(!(UCSR2A&(1<<RXC2)));
	lstate = state = 0;
  }

  // Modify these to control which messages are sent from module
  enableMsg(POSLLH_MSG, TRUE);    // Enable position messages
  enableMsg(SBAS_MSG, FALSE);      // Enable SBAS messages
  enableMsg(VELNED_MSG, FALSE);    // Enable velocity messages
  enableMsg(STATUS_MSG, FALSE);    // Enable status messages
  enableMsg(SOL_MSG, TRUE);       // Enable soluton messages
  enableMsg(DOP_MSG, FALSE);       // Enable DOP messages
  enableMsg(DGPS_MSG, FALSE);     // Disable DGPS messages
}




  // Convert 1e-7 value packed into long into decimal format
void printLatLon (long val) {
  char buffer[14];
  //PString str(buffer, sizeof(buffer));
  ltoa(val,buffer,10);
  //str.print(val, DEC);
  //char len = str.length();
  int len = sizeof(buffer) / sizeof(buffer[0]);
  char ii = 0;
  while (ii < (len - 7)) {
    Serial_write(buffer[ii++]);
  }
  Serial_write(".");
  while (ii < len) {
    Serial_write(buffer[ii++]);
  }
}

void printHex (unsigned char val) {
  if (val < 0x10)
  Serial_write("0");
  Serial_print(val);
}

void sendCmd (unsigned char len, uint8_t data[]) {
  TxByte2(0xB5);
  TxByte2(0x62);
  unsigned char chk1 = 0, chk2 = 0;
  for (unsigned char ii = 0; ii < len; ii++) {
    unsigned char cc = data[ii];
    //TxByte(cc);
	TxByte2(cc);
    chk1 += cc;
    chk2 += chk1;
  }
  TxByte2(chk1);
  TxByte2(chk2);
}




int main(void)
{
	unsigned char cc;

	DDRB = (1<<PB7);
	cbi(PORTB,PB7) ;
	_delay_ms(1000);
	sbi(PORTB,PB7);
	_delay_ms(1000);
	cbi(PORTB,PB7) ;
	_delay_ms(1000); 
	/* Replace with your application code */	
	setup();
	Serial_write(" acaba DE TERMINAR EL SETUP\r\n");

	state = 0;

	while(1) {
		
		
		//Serial_write("00\r\n");
		if(!(UCSR2A && (1<<RXC2))==0); {
			cc = UDR2;
			//while ((UCSR0A & (1<<UDRE0)) == 0);
			//Serial_write("SE HA ASIGNADO EL UDR2 \r\n");
			//TxBCD(cc);
			//Serial_write("A\r\n");
			TxBCD(state);
			Serial_write("\r\n");

			switch (state) {
				case 0:    // wait for sync 1 (0xB5)
				ck1 = ck2 = 0;
				//Serial_write("ENTRO AL CASE 0 \r\n");
				if (cc == 0xB5) //0xB5
				state++;
				break;
				
				case 1:    // wait for sync 2 (0x62)
				if (cc == 0x62) state++; //0x62
				else state = 0;
				//Serial_write("ENTRO AL CASE 1 \r\n");
				break;

				case 2:    // wait for class code
				code = cc;
				ck1 += cc;
				ck2 += ck1;
				state++;
				//Serial_write("ENTRO AL CASE 2 \r\n");
				break;

				case 3:    // wait for Id
				id = cc;
				ck1 += cc;
				ck2 += ck1;
				state++;
				//Serial_write("ENTRO AL CASE 3 \r\n");
				break;

				case 4:    // wait for length byte 1
				length = cc;
				ck1 += cc;
				ck2 += ck1;
				state++;
				/*
				Serial_write("ENTRO AL CASE 4 \r\n");
				TxBCD(length);
				Serial_write("\r\n");

				*/
				break;

				case 5:    // wait for length byte 2
				TxBCD(length);
				Serial_write("\r\n");
				TxBCD(cc);
				Serial_write("\r\n");
				//length |= (unsigned int) cc << 8;  //probar esto en ambos casos
				length |= ((unsigned int) cc << 8);

				TxBCD(length);
				Serial_write("\r\n");
				ck1 += cc;
				ck2 += ck1;
				idx = 0;
				/*Serial_write("ENTRO AL CASE 5 \r\n");
				TxBCD(length);
				Serial_write("\r\n");
				*/
				state++;
				if (length > MAX_LENGTH){
				state= 0;
				}
				
				break;

				case 6:    // wait for <length> payload bytes
				//Serial_write("ENTRO AL CASE 6 \r\n");
				data[idx++] = cc;
				ck1 += cc;
				ck2 += ck1;
				
				if (idx >= length) {
					//Serial_write("ENTRO AL CASE 6_IF \r\n");
					state++;
				}
				break;

				case 7:    // wait for checksum 1
				chk1 = cc;
				//_delay_ms(100);
				
				//Serial_write("ENTRO AL sCASE 7 \r\n");
				Serial_write("\r\n  chk1: ");
				TxBCD(chk1);
				Serial_write("\r\n  ck1: ");
				TxBCD(ck1);
				Serial_write("\r\n lenght: ");
				TxBCD(length);
				
				
				state++;
							
				break;
				case 8:    // wait for checksum 2
				
				chk2 = cc;
				
				Serial_write("ENTRO AL CASE 8 \r\n");
				Serial_write("\r\n  chk1: ");
				TxBCD(chk1);
				Serial_write("\r\n  ck1: ");
				TxBCD(ck1);
				Serial_write("\r\n lenght: ");
				TxBCD(length);
				Serial_write("\r\n ck2: ");
				TxBCD(ck2);
				Serial_write("\r\n chk2: ");
				TxBCD(chk2);
				
				if ((ck1 == chk1)  &&  (ck2 == chk2))
				{	checkOk  = 1;	}

				else {checkOk = 0;}

				checkOk = 1;
				code= 0x02;
																
				Serial_write("\r\nCHEOK: ");
				TxBCD(checkOk);
				Serial_write("\r\nCODE: ");
				TxBCD(code);
				Serial_write("\r\n");

				if (checkOk == TRUE) {
/*
				Serial_write("ingreso al selector \r\n");
				Serial_write("PRECISION: ");
				TxBCD(ULONG(24));
				Serial_write("LONGITUD: ");
				printLatLon(LONG(4));
				Serial_write("\r\n");
				//Serial_print(F(", lat = "));
				Serial_write("LATITUD: ");
				printLatLon(LONG(8));
				Serial_write("\r\n");

				*/
					switch (code) {		
												
						case 0x01:      // NAV-
						// Add blank line between time groups
						if (lastTime != ULONG(0)) {
							lastTime = ULONG(0);
							//Serial_print(F("\nTime: "));
							//Serial_print(ULONG(0)"\n");
												}
						Serial_print("NAV-");
						switch (id) {
							case 0x02:  // NAV-POSLLH
							//Serial_print(F("POSLLH: lon = "));
							Serial_write("LONGITUD: ");
							printLatLon(LONG(4));
							Serial_write("\r\n");
							//Serial_print(F(", lat = "));
							Serial_write("LATITUD: ");
							printLatLon(LONG(8));
							Serial_write("\r\n");
							/*Serial_print(F(", vAcc = "));
							Serial_print(ULONG(24), DEC);
							Serial_print(F(" mm, hAcc = "));
							Serial_print(ULONG(20), DEC);
							Serial_print(F(" mm"));
							*/
							break;
							case 0x03:  // NAV-STATUS
							Serial_write("STATUS: gpsFix = ");
							Serial_print(data[4]);
							if (data[5] & 2) {
								Serial_write(", dgpsFix");
							}
							
							break;
							
							case 0x04:  // NAV-DOP
							
							Serial_write("DOP:    gDOP = ");
							/*Serial_print((float) UINT(4) / 100, 2);
							Serial_print(", tDOP = ");
							Serial_print((float) UINT(8) / 100, 2);
							Serial_print(", vDOP = ");
							Serial_print((float) UINT(10) / 100, 2);
							Serial_print(", hDOP = ");
							Serial_print((float) UINT(12) / 100, 2);
							*/
							
							break;
							
							
							case 0x06:  // NAV-SOL
							/*
							Serial_print(F("SOL:    week = "));
							Serial_print(UINT(8), DEC);
							Serial_print(F(", gpsFix = "));
							Serial_print(data[10], DEC);
							Serial_print(F(", pDOP = "));
							Serial_print((float) UINT(44) / 100.0, 2);
							*/
							//Serial_write(", pAcc = ");
							Serial_write("PRECISION: ");							
							Serial_print(ULONG(24));
							//Serial_write(" cm")
							/*Serial_print(F(" cm, numSV = "));
							Serial_print(data[47], DEC);
							*/
														
							break;
							
							case 0x12:  // NAV-VELNED
							
							Serial_print("VELNED: gSpeed = ");
							/*
							Serial_print(ULONG(20), DEC);
							Serial_print(F(" cm/sec, sAcc = "));
							Serial_print(ULONG(28), DEC);
							Serial_print(F(" cm/sec, heading = "));
							Serial_print((float) LONG(24) / 100000, 2);
							Serial_print(F(" deg, cAcc = "));
							Serial_print((float) LONG(32) / 100000, 2);
							Serial_print(F(" deg"));
							*/
							break;
							case 0x31:  // NAV-DGPS
							
							Serial_write("DGPS:   age = ");
							//Serial_print(LONG(4));
							Serial_write(", baseId = ");
							//Serial_print(INT(8));
							Serial_write(", numCh = ");
							//Serial_print(INT(12));
							
							break;
							case 0x32:  // NAV-SBAS
							
							Serial_write("SBAS:   geo = ");
							switch (data[4]) {
								
								case 133:
								Serial_write("Inmarsat 4F3");
								break;
								case 135:
								Serial_write("Galaxy 15");
								break;
								case 138:
								Serial_write("Anik F1R");
								break;
								default:
								Serial_write(data[4]);
							
								break;
							}
							
							//Serial_print(", mode = ");
							
							switch (data[5]) {
								
								case 0:
								Serial_write("disabled");
								break;
								case 1:
								Serial_write("enabled integrity");
								break;
								case 2:
								Serial_write("enabled test mode");
								break;
								default:
								Serial_write(data[5]);
							
							}
							
							Serial_print(", sys = ");
							switch (data[6]) {
								case 0:
								Serial_write("WAAS");
								break;
								case 1:
								Serial_write("EGNOS");
								break;
								case 2:
								Serial_write("MSAS");
								break;
								case 16:
								Serial_write("GPS");
								break;
								default:
								Serial_write(data[6]);
								
							}
							break;
							default:
							printHex(id);
						}
						Serial2_print("\n ");
						break;
						case 0x05:      // ACK-
						Serial_print("ACK-");
						
						switch (id) {
							case 0x00:  // ACK-NAK
							Serial_print("NAK: ");
							break;
							case 0x01:  // ACK-ACK
							Serial_print("ACK: ");
							break;
						}
						
						printHex(data[0]);
						Serial_print(" ");
						printHex(data[1]);
						Serial_print("\n ");
						break;
					}
				}
				state = 0;
				break;
			}
		}
		
	}
}