/*
 * CarESD.c
 *
 * Created: 2012-05-03 20:10:39
 *  Author: Neil
 */ 
#include <avr/io.h>
#include <inttypes.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <inttypes.h>

#include <avr/wdt.h>

#define F_CPU 8000000UL
#include <util/delay.h>

#define Pin(x)            (1 << (x))
#define SetPin(addr,x)    (addr |= Pin(x))
#define ClearPin(addr,x)  (addr &= ~Pin(x))

#define IsPinClear(addr,x) ((addr & Pin(x)) == 0)
#define IsPinSet(addr,x) ((addr & Pin(x)) != 0)

#define TDriverPinLeft 0
#define TDriverPinRight 1
#define TDriverPinPower 2

#define DriverPinLeft 4
#define DriverPinPower 2
#define DriverPinRight 3

#define DriverPortLeft PORTB
#define DriverPortRight PORTB
#define DriverPortPower PORTB

#define DDriverPortLeft DDRB
#define DDriverPortRight DDRB
#define DDriverPortPower DDRB

#define FALSE 0
#define TRUE 1

uint8_t PortTmp = 0xFF;
uint8_t PortTmpPrevious = 0xFF;

uint8_t tmp = 0xE0;
uint8_t tmp2 = 0;

uint8_t fail = TRUE;

uint16_t ov_counter = 0;

#define MINONWIDTH 950
#define MAXONWIDTH 2075
#define MINOFFWIDTH 12000
#define MAXOFFWIDTH 24000

typedef struct {
  uint16_t  riseTime;
  uint16_t  fallTime;
  uint16_t  lastGoodWidth;
} tPinTimingData;

tPinTimingData pin = {0,0,1500};


void InitDriver() {
	//set as out ports
	DDriverPortLeft |= 1<<DriverPinLeft;
	DDriverPortRight |= 1<<DriverPinRight;
	DDriverPortPower |= 1<<DriverPinPower;
	
	//Set them to 1. (off)
	DriverPortLeft |= 1<<DriverPinLeft;
	DriverPortRight |= 1<<DriverPinRight;
	DriverPortPower |= 1<<DriverPinPower;	
}

void DriveForward() {
	PortTmp &= ~_BV(TDriverPinLeft);
	PortTmp |= _BV(TDriverPinRight);
	PortTmp &= ~_BV(TDriverPinPower);
}

void DriveBack() {
	PortTmp |= _BV(TDriverPinLeft);
	PortTmp &= ~_BV(TDriverPinRight);
	PortTmp &= ~_BV(TDriverPinPower);
}

void Break() {
	PortTmp |= _BV(TDriverPinLeft);
	PortTmp |= _BV(TDriverPinRight);
	PortTmp |= _BV(TDriverPinPower);
}

void DriverOff() {
	PortTmp |= _BV(TDriverPinLeft);
	PortTmp |= _BV(TDriverPinRight);
	PortTmp |= _BV(TDriverPinPower);
}


void Forward() {
	DriverOff();
	DriveForward();
}

void Backward() {
	DriverOff();
	DriveBack();
}

void AllOff() {
	PortTmp |= _BV(TDriverPinLeft);
	PortTmp |= _BV(TDriverPinRight);
	PortTmp |= _BV(TDriverPinPower);
}

void UpdatePort() {
	//DRIVE
	
	uint32_t tmp = (pin.lastGoodWidth - 1000);
	
	if (IsPinSet(PortTmp, TDriverPinPower)) {
		OCR0A = 0xFF;
	} else {
		if (IsPinClear(PortTmp, TDriverPinLeft) && IsPinSet(PortTmp, TDriverPinRight)) {
			OCR0A = (500-tmp)/2;
		} else if (IsPinSet(PortTmp, TDriverPinLeft) && IsPinClear(PortTmp, TDriverPinRight)) {
			OCR0A = (tmp-500)/2;
		}		
	}
	
	/*if ((PortTmpPrevious & TDriverPinRight) == (PortTmp & TDriverPinRight) &&
		(PortTmpPrevious & TDriverPinLeft) == (PortTmp & TDriverPinLeft))
		return;
	*/		

	_delay_ms(100);
	
	if (IsPinClear(PortTmp, TDriverPinLeft) && IsPinClear(PortTmp, TDriverPinRight)) {
		//break
		//ClearPin(DriverPortLeft, DriverPinLeft);
		//ClearPin(DriverPortRight, DriverPinRight);
	} else if (IsPinClear(PortTmp, TDriverPinLeft) && IsPinSet(PortTmp, TDriverPinRight)) {
		//Forward
		ClearPin(DriverPortLeft, DriverPinLeft); 
		SetPin(DriverPortRight, DriverPinRight);
		//OCR0A = (tmp-500)/2;
		
	} else if (IsPinSet(PortTmp, TDriverPinLeft) && IsPinClear(PortTmp, TDriverPinRight)) {
		//Backward
		SetPin(DriverPortLeft, DriverPinLeft);
		ClearPin(DriverPortRight, DriverPinRight);
		//OCR0A = (500-tmp)/2;
		
	} else if (IsPinSet(PortTmp, TDriverPinLeft) && IsPinSet(PortTmp, TDriverPinRight)) {
		SetPin(DriverPortLeft, DriverPinLeft);
		SetPin(DriverPortRight, DriverPinRight);		
	}
		
	PortTmpPrevious = PortTmp;
}

ISR(TIMER1_OVF_vect) {
/*	
if (IsPinSet(PIND, PIND0)) {
		fail = TRUE;		
	} else {
		fail = FALSE;		
	} 
*/
	ov_counter++;
	
	if (ov_counter > 3) {
		fail = TRUE;		
	} else {
		fail = FALSE;		
	}
}

ISR(TIMER1_CAPT_vect){
/*
	if (IsPinSet(PIND, PIND0)) {
		fail = TRUE;		
	} else {
		fail = FALSE;		
	}	
*/
	
	uint32_t time;
	if (IsPinSet(PIND, PIND6) > 0) {		
		time = (((uint32_t)ov_counter)<<16) + pin.riseTime;
		if ((time >= MINOFFWIDTH) && (time <= MAXOFFWIDTH)) {
			ov_counter = 0;
		} 
		
		pin.riseTime = ICR1;
		
		TCCR1B &= ~(1<<ICES1); 
		ov_counter=0; 
		
	} else { 
		pin.fallTime = ICR1;

		TCCR1B|=1<<ICES1;
		time=(uint32_t)pin.fallTime-(uint32_t)pin.riseTime+(((uint32_t)ov_counter)<<16); 
		if ((time >= MINONWIDTH) && (time <= MAXONWIDTH)) {
			pin.lastGoodWidth = time;
		} 
	} 
} 

int main(void)
{	
	wdt_enable(WDTO_500MS);
	
	PortTmp = 0xFF;
	PortTmpPrevious = 0xFF;
	
	DDRB = 0xFC;
	PORTB= 0xFF;
	
	DDRD = 0x00;
	PORTD= 0xFF;
	
	InitDriver();
	DriverOff();
	
	_delay_ms(250);
	wdt_reset();
	_delay_ms(250);
	wdt_reset();
	_delay_ms(250);
	wdt_reset();
	_delay_ms(250);
	wdt_reset();
	_delay_ms(250);
	wdt_reset();
	_delay_ms(250);
	wdt_reset();
	_delay_ms(250);
	wdt_reset();
	_delay_ms(250);
	wdt_reset();
	
	OCR0A = 0xFF;
	TCCR0A=0b10000111; //Clear OC0A on Compare Match // Fast PWM
	TCCR0B=0b00000010; //clk/8 // clk=8mhz/8= 1mhz
	
	TCCR1A=0; //Normal port operation // WGM - Normal
	TCCR1B=(1<<ICNC1)|(1<<ICES1)|(1<<CS11); 
	//Bit 7 – ICNC1: Input Capture Noise Canceler
	//Bit 6 – ICES1: Input Capture Edge Select
	//Bit 2:0 - CS12/CS11/CS10 - Clock select bits - 1Mhz (clk 8Mhz/8)
	
	TIMSK = 1<<TOIE1|1<<ICIE1;
	//TOIE1: Timer/Counter1, Overflow Interrupt Enable
	//ICIE1: Timer/Counter1, Input Capture Interrupt Enable
	
	sei();

	while(1) {
		wdt_reset();
		if (fail == FALSE) { //if there is signal
			if (pin.lastGoodWidth > 1550) { 
				Forward();
			} else if (pin.lastGoodWidth < 1450) {
				Backward();
			} else {
				DriverOff();	
			}	
		} else {
			DriverOff();	
		}
		UpdatePort(); 
	}
}