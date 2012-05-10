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

uint16_t t = 0;
uint8_t PortTmp = 0xFF;
uint8_t PortTmpPrevious = 0xFF;

volatile uint8_t timerL = 0;
volatile uint8_t timerH = 0;
volatile uint8_t inittemp = 0;
volatile uint8_t countDown = 0;

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
	if (IsPinSet(PortTmp, TDriverPinPower)) 
		OCR0A = 0xFF;
	else
		OCR0A = 0x0;	
	
	/*if ((PortTmpPrevious & TDriverPinRight) == (PortTmp & TDriverPinRight) &&
		(PortTmpPrevious & TDriverPinLeft) == (PortTmp & TDriverPinLeft))
		return;
	*/		
	SetPin(DriverPortLeft, DriverPinLeft);
	SetPin(DriverPortRight, DriverPinRight);
	_delay_ms(100);
	
	if (IsPinClear(PortTmp, TDriverPinLeft) && IsPinClear(PortTmp, TDriverPinRight)) {
		//break
		//ClearPin(DriverPortLeft, DriverPinLeft);
		//ClearPin(DriverPortRight, DriverPinRight);
	} else if (IsPinClear(PortTmp, TDriverPinLeft) && 
			   IsPinSet(PortTmp, TDriverPinRight)) {
		//Forward
		ClearPin(DriverPortLeft, DriverPinLeft); 
		SetPin(DriverPortRight, DriverPinRight);
		
	} else if (IsPinSet(PortTmp, TDriverPinLeft) && IsPinClear(PortTmp, TDriverPinRight)) {
		//Turn right
		SetPin(DriverPortLeft, DriverPinLeft);
		ClearPin(DriverPortRight, DriverPinRight);
		
	} else if (IsPinSet(PortTmp, TDriverPinLeft) && IsPinSet(PortTmp, TDriverPinRight)) {
		SetPin(DriverPortLeft, DriverPinLeft);
		SetPin(DriverPortRight, DriverPinRight);		
	}
		
	PortTmpPrevious = PortTmp;
}

uint8_t tmp = 0xE0;
uint8_t tmp2 = 0;

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

tPinTimingData pin;

ISR(TIMER1_OVF_vect) {
	ov_counter++;
}

ISR(TIMER1_CAPT_vect){
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
	
	
	DDRB = 0xFC;
	PORTB= 0x0;
	
	DDRD = 0x00;
	PORTD= 0xFF;
	
	InitDriver();
	
	OCR0A = 0xFF;
	TCCR0A=0b11110111;
	TCCR0B=0b00000010;	
	
	TCCR1A=0;
	TCCR1B=(1<<ICNC1)|(1<<ICES1)|(1<<CS11);
	TIMSK = 1<<TOIE1|1<<ICIE1;
	sei();

	while(1) {
		if (pin.lastGoodWidth > 1600) { 
			Forward();
		} else if (pin.lastGoodWidth < 1400) {
			Backward();
		} else {
			DriverOff();	
		}
		
		UpdatePort();
		_delay_ms(100);		
	}
}