/*
             LUFA Library
     Copyright (C) Dean Camera, 2017.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2017  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Header file for GrubControl.c.
 */

#ifndef _GRUBCONTROL_H_
#define _GRUBCONTROL_H_

	/* Includes: */
		#include <avr/io.h>
		#include <avr/wdt.h>
		#include <avr/power.h>
		#include <avr/interrupt.h>
		#include <stdlib.h>
		#include <ctype.h>
		#include <string.h>
		#include <LUFA/Drivers/Peripheral/Serial.h>
		#include <LUFA/Drivers/Misc/RingBuffer.h>

		#include "Descriptors.h"

		#include <LUFA/Drivers/USB/USB.h>
		#include <LUFA/Drivers/Board/LEDs.h>
		#include <LUFA/Platform/Platform.h>

	typedef struct
		{
			uint32_t BaudRateBPS; /**< Baud rate of the virtual serial port, in bits per second. */
			uint8_t  CharFormat; /**< Character format of the virtual serial port, a value from the
								  *   \ref CDC_LineEncodingFormats_t enum.
								  */
			uint16_t  ParityType; /**< Parity setting of the virtual serial port, a value from the
								  *   \ref CDC_LineEncodingParity_t enum.
								  */
			uint8_t  DataBits; /**< Bits of data per character of the virtual serial port. */
			uint8_t  Latency;
			uint8_t event_chr; //event character
			uint8_t error_chr; //error character
			
		} ATTR_PACKED FTDI_LineEncoding_t;

		#define FTDI_SIO_RESET				0 /* Reset the port */
		#define FTDI_SIO_MODEM_CTRL			1 /* Set the modem control register */
		#define FTDI_SIO_SET_FLOW_CTRL		2 /* Set flow control register */
		#define FTDI_SIO_SET_BAUD_RATE		3 /* Set baud rate */   //LineEncoding..BaudRateBPS
		#define FTDI_SIO_SET_DATA			4 /* Set the data characteristics of the port */
		#define FTDI_SIO_GET_MODEM_STATUS	5 /* Retrieve current value of modem status register */
		#define FTDI_SIO_SET_EVENT_CHAR		6 /* Set the event character */
		#define FTDI_SIO_SET_ERROR_CHAR		7 /* Set the error character */
		#define FTDI_SIO_SET_LATENCY_TIMER	9 /* Set the latency timer */
		#define FTDI_SIO_GET_LATENCY_TIMER	0x0a /* Get the latency timer */
		#define FTDI_SIO_SET_BITMODE		0x0b /* Set bitbang mode */
		#define FTDI_SIO_READ_PINS			0x0c /* Read immediate value of pins */
		#define FTDI_SIO_READ_EEPROM		0x90 /* Read EEPROM */

	/* Function Prototypes: */
		void SetupHardware(void);

		uint8_t Device_SendByte(const uint8_t);
		void setSendFlag(void);
		uint8_t addStringtoBuffer( char*, RingBuffer_t);
		
		void timer1(uint8_t prescaler, uint16_t ticks, void (*f)()) ;
		void timer1_stop();
		
		int getCommandSequence(void); 
		void searchInitGrub(void);
		void findOS(void);
		void startOS(void);
		
		void receiveFromHost(void);
		void processFromHost(void);
		void sendToHost(void);
		
		void receiveUsartToHostInBuffer(void);
		void sendHostOutBufferToUsart(void);

		void EVENT_USB_Device_Connect(void);
		void EVENT_USB_Device_Disconnect(void);
		void EVENT_USB_Device_ConfigurationChanged(void);
		void EVENT_USB_Device_ControlRequest(void);

#endif

