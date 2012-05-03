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
	
	if ((PortTmpPrevious&0x7) == (PortTmp &0x7))
		return
	
		
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

int main(void)
{
	DDRD = 0x3;
	PORTD = 0xFF;
	DDRB = 0xFF;
	PORTB = 0xFF;
	
	OCR0A=0x1; // przyk³adowe wartoœci 
	OCR0B=0x50; 
	TCCR0A=0b11110111; 
	TCCR0B=0b00000010;
	
	
	TCCR0B=0b00000010;
	
	
	while(1);

}