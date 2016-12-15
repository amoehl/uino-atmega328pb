/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

  $Id: wiring.h 249 2007-02-03 16:52:51Z mellis $
*/


/* *uino Superstick  Atmega328PB based Board with 20MHz 
* by A. Moehl, 2016
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

#define NUM_DIGITAL_PINS            18
#define NUM_ANALOG_INPUTS           6


/*pins which can deliver PWM */
#define digitalPinHasPWM(p)  ((p) == 1 || (p) == 4 || (p) >= 12  || (p) == 14 || (p) == 16 || (p) == 17 || (p) == 18)

/* First SPI */
static const uint8_t MOSI = 1;
static const uint8_t MISO = 2;
static const uint8_t SCK  = 3;
static const uint8_t SS   = 4;

/* 2nd SPI */
static const uint8_t MOSI1 = 5;
static const uint8_t MISO1 = 8;
static const uint8_t SCK1  = 7;
static const uint8_t SS1   = 6;

/* Default I2C is I2C1, because I2C0 isn't available on the *uino Superstick*/
static const uint8_t SDA = 13;	// PE0
static const uint8_t SCL = 11;	// PE1

static const uint8_t SDA0 = 13;	//PC4
static const uint8_t SCL0 = 11; //PC5

/* Second I2C */
static const uint8_t SDA1 = 13;	// PE0
static const uint8_t SCL1 = 11;	// PE1

#undef LED_BUILTIN
#define LED_BUILTIN 	0	// PD7 - only for the LED



/*Pin Change Interrupt Control Register */
#define digitalPinToPCICR(p)    (((p) >= 0 && (p) <= NUM_DIGITAL_PINS) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p) \
	( \
		(p) == 0 ? 2 : \
		((p) == 1 ? 0 : \
	 	((p) == 2 ? 0 : \
		((p) == 3 ? 0 : \
		((p) == 4 ? 0 : \
		((p) == 5 ? 3 : \
		((p) == 6 ? 3 : \
		((p) == 7 ? 1 : \
		((p) == 8 ? 2 : \
		((p) == 9 ? 2 : \
		((p) == 10 ? 1 : \
		((p) == 11 ? 3 : \
		((p) == 12 ? 2 : \
		((p) == 13 ? 3 : \
		((p) == 14 ? 2 : \
		((p) == 15 ? 0 : \
		((p) == 16 ? 1 : \
		((p) == 17 ? 0 : \
		((p) == 18 ? 1 :  0) \
	))))))))))))))))))

/* Pin Change Mask Register 0*/
// Pin Change interrupts
// PCINT0 - PCINT7 = PB0-PB7
// PCINT8 - PCINT15 = PC0-PC7
// PCINT16 - PCINT23 = PD0-PD7
// PCINT24 - PCINT27 = PE0-PE3

/* map digital pin to pin change mask register */
#define digitalPinToPCMSK(p)  (((p) == 1 || (p) == 2 || (p) == 3 || (p) == 4 || (p) == 15 || (p) == 17 ) ? &PCMSK0: \
								(((p) == 7 || (p) == 8 || (p) == 9 || (p) == 10 ) ? &PCMSK1 : \
								 (((p) == 0 || (p) == 12 || (p) == 14 || (p) == 16 || (p) == 18 ) ? &PCMSK2: \
								  (((p) == 5 || (p) == 6 || (p) == 11 || (p) == 13 ) ? &PCMSK3: ((uint8_t *)0) \
								))))

/* map arduino pin to bit of the PCMSK Register */
#define digitalPinToPCMSKbit(p) \
	( \
		(p) == 0 ? 7 : \
		((p) == 1 ? 3 : \
	 	((p) == 2 ? 4 : \
		((p) == 3 ? 5 : \
		((p) == 4 ? 2 : \
		((p) == 5 ? 3 : \
		((p) == 6 ? 2 : \
		((p) == 7 ? 1 : \
		((p) == 8 ? 0 : \
		((p) == 9 ? 3 : \
		((p) == 10 ? 2 : \
		((p) == 11 ? 1 : \
		((p) == 12 ? 2 : \
		((p) == 13 ? 0 : \
		((p) == 14 ? 1 : \
		((p) == 15 ? 0 : \
		((p) == 16 ? 0 : \
		((p) == 17 ? 1 : \
		((p) == 18 ? 6 : 0) \
	))))))))))))))))))




/* external interrupt */
#define digitalPinToInterrupt(p)  ((p) == 12 ? 0 :  NOT_AN_INTERRUPT))

/* Analog Pins*/
#define analogInputToDigitalPin(p)  ((p < NUM_ANALOG_INPUTS) ? (p) + 6 : -1)
static const uint8_t A0 = 0;
static const uint8_t A1 = 1;
static const uint8_t A2 = 2;
static const uint8_t A3 = 3;
static const uint8_t A4 = 4;
static const uint8_t A5 = 5;


/* Analog Channel Pin Assignment */
extern const uint8_t PROGMEM analog_pin_to_channel_PGM[];
#define analogPinToChannel(P)  ( pgm_read_byte( analog_pin_to_channel_PGM + (P) ) )

#ifdef ARDUINO_MAIN


// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
	(uint16_t) &DDRE,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
	(uint16_t) &PORTE,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
	(uint16_t) &PINE,
};

/* Port Settings per externel IO Pin*/
const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	PD, // PD7 - LED
	PB, // PB3 
	PB, // PB4
	PB, // PB5
	PB, // PB6
	PE, // PE3
	PE, // PE2
	PC, // PC1
	PC, // PC0
	PC, // PC3
	PC, // PC2
	PE, // PE1 
	PD, // PD2 
	PE, // PE0
	PD, // PD1
	PB, // PB0 
	PD, // PD0
	PB, // PB1
	PD  // PD6
};

/* Pin - Port Setting*/
const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	_BV(7), 
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(6),
	_BV(3),
	_BV(2),
	_BV(1),
	_BV(0), 
	_BV(3),
	_BV(2),
	_BV(1),
	_BV(2),
	_BV(0),
	_BV(1), 
	_BV(0),
	_BV(0),
	_BV(1),
	_BV(6)
};

/* Pin-Timer Assignment */
const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	NOT_ON_TIMER, 
	TIMER2A,
	NOT_ON_TIMER,
	TIMER1B,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER, 
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	TIMER4B,
	NOT_ON_TIMER,
	TIMER4A,
	NOT_ON_TIMER, 
	TIMER3A,
	TIMER1A,
	TIMER0A
};


const uint8_t PROGMEM analog_pin_to_channel_PGM[] = {
    7,  // A0               ADC7
    6,  // A1               ADC6    
    1,  // A2               ADC1    
    0,  // A3               ADC0
    3,  // A4               ADC3    
    2,  // A5               ADC2    
};



#endif /* ARDUINO_MAIN */


#define SERIAL_PORT_MONITOR   		Serial
#define SERIAL_PORT_HARDWARE  		Serial
#define SERIAL_PORT_HARDWARE1       Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial1

#endif
