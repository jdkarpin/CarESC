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

uint16_t ov_counter = 0; 



#define RISING_EDGE 1
#define FALLING_EDGE 0

#define MINONWIDTH 950
#define MAXONWIDTH 2075
#define MINOFFWIDTH 12000
#define MAXOFFWIDTH 24000

typedef struct {
  uint8_t edge;
  uint16_t  riseTime;
  uint16_t  fallTime;
  uint16_t  lastGoodWidth;
} tPinTimingData;

tPinTimingData pin;

uint16_t rising, falling; 
uint32_t counts;

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
	
	OCR0A= 0x0;
	TCCR0A=0b11110111;
	TCCR0B=0b00000010;	
	
	TCCR1A=0;//0b00001010;
	TCCR1B=(1<<ICNC1)|(1<<ICES1)|(1<<CS11);
	TIMSK = 1<<TOIE1|1<<ICIE1;
	//TIMSK=1<<OCIE1B;//|1<<ICIE1;
	
	counts = 0;

	
	sei();

	while(1) {
		_delay_ms(100);
	}
}