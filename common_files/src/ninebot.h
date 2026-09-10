/****************************************************************************/
//  Function: Header file ninebot communication
//  Author:   Camilo Ruiz
//  Date:    october 10 2017
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
// I am not responsible of any damage caused by the misuse of this library
// use at your own risk
//
// If you modify this or use this, please don't delete my name and give me the credits
// Greetings from Colombia :) 
// Hardware port by ub4raf
/****************************************************************************/


#ifndef NINEBOT_H_
#define NINEBOT_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

//#include "mbed.h"
#define NinebotHeader0 0x55//0x55
#define NinebotHeader1 0xAA//0xAA
#define Ninebotread 0x01
#define Ninebotwrite 0x03
#define NinebotMaxPayload 0x38
//message len max is 256, header, command, rw and cheksum total len is 8, therefore payload max len is 248
//max input bluetooth buffer in this chip allows a payload max 0x38
typedef struct {
	uint8_t start1;
	uint8_t start2;
	uint8_t len;
	uint8_t addr;
	uint8_t cmd;
    uint8_t arg;
    uint8_t payload[NinebotMaxPayload];
    uint8_t CheckSum[2];
} NinebotPack;
                                   //0     1    2     3      4    5     6      7     8    9     10    11    12    13
//static uint8_t	ui8_tx_buffer[] = {0x55, 0xAA, 0x08, 0x21, 0x64, 0x00, 0x01, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/*
ui8_tx_buffer[10]=MS->Speed;
			ui8_tx_buffer[6]=MS->gear_state;
			ui8_tx_buffer[7]=map(MS->Voltage,33000,42000,0,96);
			if(MS->light)ui8_tx_buffer[8]=64;
			else ui8_tx_buffer[8]=0;
			addCRC((uint8_t*)ui8_tx_buffer, ui8_tx_buffer[2]+6);
			HAL_HalfDuplex_EnableTransmitter(&huart1);
			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)ui8_tx_buffer, sizeof(ui8_tx_buffer));
			}
*/

enum m365_display_mode{
	M365_MODE_DRIVE=1,
	M365_MODE_SLOW=2,
	M365_MODE_SPORT=4,
	M365_MODE_CHARGE=8,
	M365_MODE_OFF=16,
	M365_MODE_LOCK=32,
	M365_MODE_MPH=64,
	M365_MODE_TEMP=128
};

//0      	1      		2      		3      		4      		5      		6      		7
//1		 	2			4	   		8	  		16	 		32			64	   		128
//Drive		Slow	   	Sport		Charge 		Off	 		Lock		MPH   		Temp

typedef struct {
	uint8_t start1;  	//0x55
	uint8_t start2;  	//0xAA
	uint8_t len;
	uint8_t addr;
	uint8_t cmd;
    uint8_t arg;
    uint8_t mode;
	uint8_t battery;
	uint8_t light;
	int8_t  esc_temp;   /* [9]  was "beep", only ever written as 0.
	                     *      ESC/FET temperature in degrees C, signed. */
	uint8_t speed;
	int8_t  power;      /* [11] was "faultcode", which the display does not read.
	                     *      Electrical power in 20 W steps, negative on regen,
	                     *      clamped to +/-127 => +/-2540 W. */
    uint8_t CheckSum[2];
} m365Answer;

/* The frame is a fixed 14 bytes on the wire and the ESP32 indexes it by offset,
 * so both the total size and the position of every field are load-bearing.
 * There is no #pragma pack here - the struct is 14 bytes only because every
 * member is byte-sized, which is exactly the property that would break silently
 * if someone widened a field. These assertions make that failure a build error.
 * Repurposing beep/faultcode as int8_t keeps alignment at 1 and size at 14. */
_Static_assert(sizeof(m365Answer) == 14, "m365Answer must stay exactly 14 bytes on the wire");
_Static_assert(offsetof(m365Answer, mode)     ==  6, "m365Answer.mode must stay at offset 6");
_Static_assert(offsetof(m365Answer, battery)  ==  7, "m365Answer.battery must stay at offset 7");
_Static_assert(offsetof(m365Answer, light)    ==  8, "m365Answer.light must stay at offset 8");
_Static_assert(offsetof(m365Answer, esc_temp) ==  9, "m365Answer.esc_temp must stay at offset 9 (was beep)");
_Static_assert(offsetof(m365Answer, speed)    == 10, "m365Answer.speed must stay at offset 10");
_Static_assert(offsetof(m365Answer, power)    == 11, "m365Answer.power must stay at offset 11 (was faultcode)");
_Static_assert(offsetof(m365Answer, CheckSum) == 12, "m365Answer.CheckSum must stay at offset 12");


void addCRC(uint8_t * message, uint8_t size);
extern m365Answer m365_to_display;

uint16_t ninebot_parse(uint8_t data, NinebotPack *message);

#endif
