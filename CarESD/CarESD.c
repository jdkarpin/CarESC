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
#define DriverPinPower 3
#define DriverPinRight 2

#define DriverPortLeft PINB
#define DriverPortRight PINB
#define DriverPortPower PINB

uint16_t t = 0;
uint8_t PortTmp = 0xFF;
uint8_t PortTmpPrevious = 0xFF;

volatile uint8_t timerL = 0;
volatile uint8_t timerH = 0;
volatile uint8_t inittemp = 0;
volatile uint8_t countDown = 0;

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
	//PortTmp |= _BV(TDriverPinLeft);
	//PortTmp |= _BV(TDriverPinRight);
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
	//PortTmp |= _BV(DriverPinLeft);
	//PortTmp |= _BV(DriverPinRight);
	PortTmp |= _BV(TDriverPinPower);
}

void UpdatePort() {
	//DRIVE
	/*if (IsPinSet(PortTmp, TDriverPinPower)) 
		SetPin(DriverPortPower, DriverPinPower);
	else
		ClearPin(DriverPortPower, DriverPinPower);
	*/
	
	if ((PortTmpPrevious & DriverPinRight) == (PortTmp & TDriverPinRight) &&
		(PortTmpPrevious & DriverPinLeft) == (PortTmp & TDriverPinLeft))
		return;
			
	ClearPin(DriverPortLeft, DriverPinLeft);
	ClearPin(DriverPortRight, DriverPinRight);
	_delay_ms(100);
	
	if (IsPinClear(PortTmp, TDriverPinLeft) && IsPinClear(PortTmp, TDriverPinRight)) {
		ClearPin(DriverPortLeft, DriverPinLeft);
		ClearPin(DriverPortRight, DriverPinRight);
	} else if (IsPinClear(PortTmp, TDriverPinLeft) && IsPinSet(PortTmp, TDriverPinRight)) {
		//Turn left
		ClearPin(DriverPortLeft, DriverPinLeft); 
		SetPin(DriverPortRight, DriverPinRight);
		
	} else if (IsPinSet(PortTmp, TDriverPinLeft) && IsPinClear(PortTmp, TDriverPinRight)) {
		//Turn right
		SetPin(DriverPortRight, DriverPinRight);
		ClearPin(DriverPortLeft, DriverPinLeft);
	} else if (IsPinSet(PortTmp, TDriverPinLeft) && IsPinSet(PortTmp, TDriverPinRight)) {
		ClearPin(DriverPortLeft, DriverPinLeft);
		ClearPin(DriverPortRight, DriverPinRight);		
		ClearPin(DriverPortPower, DriverPinPower);
	}
		
	PortTmpPrevious = PortTmp;
}

uint8_t tmp = 0xE0;

uint8_t tmp2 = 0;
unsigned long milis = 0;


unsigned long long fallingEdgeTime = 0;
unsigned long long risingEdgeTime = 0;

ISR(TIMER1_COMPA_vect)
{	
	milis++;
	if (milis%100000) {
		if (tmp2 == 0) {
			PORTB &= ~1<<4; 
			tmp2 = 1;
		} else {
			PORTB |= 1<<4; 
			tmp2 = 0; 
		}
	}
	
}

#define RISING_EDGE 1
#define FALLING_EDGE 0

#define MINONWIDTH 950
#define MAXONWIDTH 2075
#define MINOFFWIDTH 12000
#define MAXOFFWIDTH 24000

typedef struct {
  uint8_t edge;
  unsigned long  riseTime;
  unsigned long  fallTime;
  unsigned long  lastGoodWidth;
} tPinTimingData;

tPinTimingData pin;

ISR(PCINT_vect) {
	unsigned long time;
	if (IsPinSet(PINB, PINB0) > 0) {

		time = milis - pin.fallTime;
		pin.riseTime = milis;
		if ((time >= MINOFFWIDTH) && (time <= MAXOFFWIDTH))
			pin.edge = RISING_EDGE;
		else
			pin.edge = FALLING_EDGE; // invalid rising edge detected
	} else {
		pin.fallTime = milis;
		
        if ((time >= MINONWIDTH) && (time <= MAXONWIDTH) && (pin.edge == RISING_EDGE)) {
			pin.lastGoodWidth = time;
			pin.edge = FALLING_EDGE;

        }
	}
	

	
	
		
}

int main(void)
{	
	DDRB = 0xFC;
	PORTB = 0x00;
	OCR0A=0x0;
	TCCR0A=0b11110111;
	TCCR0B=0b00000010;
	
	
	
	OCR1A = 800;
	
	TCCR1A=0;//0b00001010;
	TCCR1B=(1 << WGM12)|(1 << CS12);
	TIMSK=1<<OCIE1A;
	
	GIMSK = 0x30; //wlaczenie przerwan PCINT 
	PCMSK = 0x01; //wlaczenie przerwania tylko dla PB0 
	
	sei();

	while(1) {
		_delay_ms(100);
		/*OCR0A = 0;
		_delay_ms(100);
		OCR0A = 0x10;
		_delay_ms(100);
		OCR0A = 0x20;
		_delay_ms(100);
		OCR0A = 0x30;
		_delay_ms(100);
		OCR0A = 0x40;
		_delay_ms(100);
		OCR0A = 0x50;
		_delay_ms(100);
		OCR0A = 0x50;
		_delay_ms(100);
		OCR0A = 0x60;
		_delay_ms(100);
		OCR0A = 0x70;
		_delay_ms(100);
		OCR0A = 0x80;
		_delay_ms(100);
		OCR0A = 0x90;
		_delay_ms(100);
		OCR0A = 0xA0;*/
		
		
	}
}