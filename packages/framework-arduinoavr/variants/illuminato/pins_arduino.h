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

  Changelog
  -----------
  11/25/11  - ryan@ryanmsutton.com - Add pins for Sanguino 644P and 1284P
  07/15/12  - ryan@ryanmsutton.com - Updated for arduino0101
  05/30/17  - william@welliver.org - Updated for Illuminato Genesis

  Improvements by Kristian Sloth Lauszus, lauszus@gmail.com
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

static const uint8_t SS   = 4;
static const uint8_t MOSI = 5;
static const uint8_t MISO = 6;
static const uint8_t SCK  = 7;

static const uint8_t SDA = 17;
static const uint8_t SCL = 16;

#define LED_BUILTIN 0

static const uint8_t A0 = 31;
static const uint8_t A1 = 30;
static const uint8_t A2 = 29;
static const uint8_t A3 = 28;
static const uint8_t A4 = 27;
static const uint8_t A5 = 26;
static const uint8_t A6 = 25;
static const uint8_t A7 = 24;

// ATMEL ATMEGA644/ATMEGA1284 / SANGUINO
//
//                        +---\/---+
//            (D 0) PB0  1|        |40  PA0 (AI 0 / D31)
//            (D 1) PB1  2|        |39  PA1 (AI 1 / D30)
//       INT2 (D 2) PB2  3|        |38  PA2 (AI 2 / D29)
//        PWM (D 3) PB3  4|        |37  PA3 (AI 3 / D28)
//     SS PWM (D 4) PB4  5|        |36  PA4 (AI 4 / D27)
//       MOSI (D 5) PB5  6|        |35  PA5 (AI 5 / D26)
//       MISO (D 6) PB6  7|        |34  PA6 (AI 6 / D25)
//        SCK (D 7) PB7  8|        |33  PA7 (AI 7 / D24)
//                  RST  9|        |32  AREF
//                  VCC 10|        |31  GND
//                  GND 11|        |30  AVCC
//                XTAL2 12|        |29  PC7 (D 23)
//                XTAL1 13|        |28  PC6 (D 22)
//       RX0 (D 8)  PD0 14|        |27  PC5 (D 21) TDI
//       TX0 (D 9)  PD1 15|        |26  PC4 (D 20) TDO
//  INT0 RX1 (D 10) PD2 16|        |25  PC3 (D 19) TMS
//  INT1 TX1 (D 11) PD3 17|        |24  PC2 (D 18) TCK
//       PWM (D 12) PD4 18|        |23  PC1 (D 17) SDA
//       PWM (D 13) PD5 19|        |22  PC0 (D 16) SCL
//       PWM (D 14) PD6 20|        |21  PD7 (D 15) PWM
//                        +--------+
//
#define NUM_DIGITAL_PINS            42
#define NUM_ANALOG_INPUTS           6

#define analogInputToDigitalPin(p)  ((p < 6) ? 36 + (p): -1)
#define analogPinToChannel(p)       ((p < 6) ? (p) : 36 + (p))

#define digitalPinHasPWM(p)         ((p) == 15 || (p) == 16 || (p) == 17 || (p) == 42 )

#if 0

#define digitalPinToPCICR(p)        ( (((p) >= 0) && ((p) <= 31)) ? (&PCICR) : ((uint8_t *)0) )

#define digitalPinToPCICRbit(p)     ( (((p) >= 24) && ((p) <= 31)) ? 0 : \
                                    ( (((p) >=  0) && ((p) <=  7)) ? 1 : \
                                    ( (((p) >= 16) && ((p) <= 23)) ? 2 : \
                                    ( (((p) >=  8) && ((p) <= 15)) ? 3 : \
                                    0 ) ) ) )

#define digitalPinToPCMSK(p)        ( (((p) >= 24) && ((p) <= 31)) ? (&PCMSK0) : \
                                    ( (((p) >=  0) && ((p) <=  7)) ? (&PCMSK1) : \
                                    ( (((p) >= 16) && ((p) <= 23)) ? (&PCMSK2) : \
                                    ( (((p) >=  8) && ((p) <= 15)) ? (&PCMSK3) : \
                                    ((uint8_t *)0) ) ) ) )


#define digitalPinToPCMSKbit(p)     ( (((p) >= 24) && ((p) <= 31)) ? (31 - (p)) : \
                                    ( (((p) >=  0) && ((p) <=  7)) ? (p) : \
                                    ( (((p) >= 16) && ((p) <= 23)) ? ((p) - 16) : \
                                    ( (((p) >=  8) && ((p) <= 15)) ? ((p) - 8) : \
                                    0 ) ) ) )

#endif
																		
#define digitalPinToInterrupt(p)    ((p) == 21 ? 0	 : NOT_AN_INTERRUPT)))

#ifndef PA
	#define PA 1
	#define PB 2
	#define PC 3
	#define PD 4
	#define PE 5
	#define PF 6
	#define PG 7
#endif	

#ifdef ARDUINO_MAIN
// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] =
{
	NOT_A_PORT,
	(uint16_t)&DDRA,
	(uint16_t)&DDRB,
	(uint16_t)&DDRC,
	(uint16_t)&DDRD,
	(uint16_t)&DDRE,
	(uint16_t)&DDRF,
	(uint16_t)&DDRG,
};

const uint16_t PROGMEM port_to_output_PGM[] =
{
	NOT_A_PORT,
	(uint16_t)&PORTA,
	(uint16_t)&PORTB,
	(uint16_t)&PORTC,
	(uint16_t)&PORTD,
	(uint16_t)&PORTE,
	(uint16_t)&PORTF,
	(uint16_t)&PORTG,
};
const uint16_t PROGMEM port_to_input_PGM[] =
{
	NOT_A_PIN,
	(uint16_t)&PINA,
	(uint16_t)&PINB,
	(uint16_t)&PINC,
	(uint16_t)&PIND,
	(uint16_t)&PINE,
	(uint16_t)&PINF,
	(uint16_t)&PING,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] =
{
	PE,		//*	PE 0	**	0	USART0_RX
	PE,		//*	PE 1	**	1	USART0_TX
	PE,		//*	PE 2	**	2
	PE,		//*	PE 3	**	3
	PE,		//*	PE 4	**	4
	PE,		//*	PE 5	**	5
	PE,		//*	PE 6	**	6
	PE,		//*	PE 7	**	7

	PD,		//*	PD 2	**	8
	PD,		//*	PD 3	**	9
	PD,		//*	PD 4	**	10
	PD,		//*	PD 6	**	11
	PD,		//*	PD 7	**	12

	PG,		//*	PG 0	**	13

	PB,		//*	PB 0	**	14
	PB,		//*	PB 4	**	15
	PB,		//*	PB 5	**	16
	PB,		//*	PB 6	**	17

	PG,		//*	PG 3	**	18
	PG,		//*	PG 4	**	19

	PD,		//*	PD 0	**	20
	PD,		//*	PD 1	**	21

	PG,		//*	PG 1	**	22

	PC,		//*	PC 0	**	23
	PC,		//*	PC 1	**	24
	PC,		//*	PC 2	**	25
	PC,		//*	PC 3	**	26
	PC,		//*	PC 4	**	27
	PC,		//*	PC 5	**	28
	PC,		//*	PC 6	**	29

	PA,		//*	PA 4	**	30
	PA,		//*	PA 5	**	31
	PA,		//*	PA 6	**	32
	PA,		//*	PA 7	**	33
	PG,		//*	PG 2	**	34
	PC,		//*	PC 7	**	35

	PF,		//*	PF 0	**	36
	PF,		//*	PF 1	**	37
	PF,		//*	PF 2	**	38
	PF,		//*	PF 3	**	39
	PF,		//*	PF 4	**	40
	PF,		//*	PF 5	**	41

	PB		//*	PB 7	**	42
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] =
{
    /* 0 */		_BV(0),
    /* 1 */		_BV(1),
    /* 2 */		_BV(2),
    /* 3 */		_BV(3),
    /* 4 */		_BV(4),
    /* 5 */		_BV(5),
    /* 6 */		_BV(6),
    /* 7 */		_BV(7),

    /* 8 */		_BV(2),
    /* 9 */		_BV(3),
    /* 10 */	_BV(4),
    /* 11 */	_BV(6),
    /* 12 */	_BV(7),

    /* 13 */	_BV(0),

    /* 14 */	_BV(0),
    /* 15 */	_BV(4),
    /* 16 */	_BV(5),
    /* 17 */	_BV(6),

    /* 18 */	_BV(3),
    /* 19 */	_BV(4),

    /* 20 */	_BV(0),
    /* 21 */	_BV(1),

    /* 22 */	_BV(1),

    /* 23 */	_BV(0),
    /* 24 */	_BV(1),
    /* 25 */	_BV(2),
    /* 26 */	_BV(3),
    /* 27 */	_BV(4),
    /* 28 */	_BV(5),
    /* 29 */	_BV(6),
    /* 30 */	_BV(4),
    /* 31 */	_BV(5),
    /* 32 */	_BV(6),
    /* 33 */	_BV(7),
    /* 34 */	_BV(2),
    /* 35 */	_BV(7),

    /* 36 */	_BV(0),
    /* 37 */	_BV(1),
    /* 38 */	_BV(2),
    /* 39 */	_BV(3),
    /* 40 */	_BV(4),
    /* 41 */	_BV(5),

    /* 42 */	_BV(7),
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] =
{
    /* 0 */  NOT_ON_TIMER,	//*	PE 0	**	0	USART0_RX
    /* 1 */  NOT_ON_TIMER,	//*	PE 1	**	1	USART0_TX
    /* 2 */  NOT_ON_TIMER,	//*	PE 2	**	2
    /* 3 */  NOT_ON_TIMER,	//*	PE 3	**	3
    /* 4 */  NOT_ON_TIMER,	//*	PE 4	**	4
    /* 5 */  NOT_ON_TIMER,	//*	PE 5	**	5
    /* 6 */  NOT_ON_TIMER,	//*	PE 6	**	6
    /* 7 */  NOT_ON_TIMER,	//*	PE 7	**	7

    /* 8 */  NOT_ON_TIMER,	//*	PD 2	**	8
    /* 9 */  NOT_ON_TIMER,	//*	PD 3	**	9
    /* 10 */ NOT_ON_TIMER,	//*	PD 4	**	10
    /* 11 */ NOT_ON_TIMER,	//*	PD 6	**	11
    /* 12 */ NOT_ON_TIMER,	//*	PD 7	**	12

    /* 13 */ NOT_ON_TIMER,	//*	PG 0	**	13

    /* 14 */ NOT_ON_TIMER,	//*	PB 0	**	14
    /* 15 */ TIMER0A,		//*	PB 4	**	15
    /* 16 */ TIMER1A,		//*	PB 5	**	16
    /* 17 */ TIMER1B,		//*	PB 6	**	17

    /* 18 */ NOT_ON_TIMER,	//*	PG 3	**	18
    /* 19 */ NOT_ON_TIMER,	//*	PG 4	**	19
	
    /* 20 */ NOT_ON_TIMER,	//*	PD 0	**	20
    /* 21 */ NOT_ON_TIMER,	//*	PD 1	**	21
	
    /* 22 */ NOT_ON_TIMER,	//*	PG 1	**	22
    /* 23 */ NOT_ON_TIMER,	//*	PC 0	**	23
    /* 24 */ NOT_ON_TIMER,	//*	PC 1	**	24
    /* 25 */ NOT_ON_TIMER,	//*	PC 2	**	25
    /* 26 */ NOT_ON_TIMER,	//*	PC 3	**	26
    /* 27 */ NOT_ON_TIMER,	//*	PC 4	**	27
    /* 28 */ NOT_ON_TIMER,	//*	PC 5	**	28
    /* 29 */ NOT_ON_TIMER,	//*	PC 6	**	29
    /* 30 */ NOT_ON_TIMER,	//*	PA 4	**	30
    /* 31 */ NOT_ON_TIMER,	//*	PA 5	**	31
    /* 32 */ NOT_ON_TIMER,	//*	PA 6	**	32
    /* 33 */ NOT_ON_TIMER,	//*	PA 7	**	33
    /* 34 */ NOT_ON_TIMER,	//*	PG 2	**	34
    /* 35 */ NOT_ON_TIMER,	//*	PC 7	**	35

    /* 36 */ NOT_ON_TIMER,	//*	PF 0	**	36
    /* 37 */ NOT_ON_TIMER,	//*	PF 1	**	37
    /* 38 */ NOT_ON_TIMER,	//*	PF 2	**	38
    /* 39 */ NOT_ON_TIMER,	//*	PF 3	**	39
    /* 40 */ NOT_ON_TIMER,	//*	PF 4	**	40
    /* 41 */ NOT_ON_TIMER,	//*	PF 5	**	41

    /* 42 */ TIMER2A,		//*	PB 7	**	42

};
#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.

#define SERIAL_PORT_MONITOR         Serial
#define SERIAL_PORT_HARDWARE        Serial

#if defined(UBRR1H)
    #define SERIAL_PORT_HARDWARE1       Serial1
    #define SERIAL_PORT_HARDWARE_OPEN   Serial1
#endif

#endif
