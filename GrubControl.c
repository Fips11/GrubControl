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
 *   This file contains the main tasks of the GrubControl and
 *  is responsible for the initial application hardware configuration.
 */

#include "GrubControl.h"
//#define debug
//#define debug_request
//#define debug_bridge


//------------------------------PROGMEM-----------------------------------------------

const uint16_t FTDI_eeprom[] PROGMEM = {
	0x00 <<8|0x40,	0x03 <<8|0x04,	0x01 <<8|0x60,	0x00 <<8|0x00,	0xa0 <<8|0x2d,	0x08 <<8|0x00,
					0x00 <<8|0x00,	0x98 <<8|0x0a,	0xa2 <<8|0x20,	0xc2 <<8|0x12,	0x23 <<8|0x10,//10
					0x05 <<8|0x00,	0x0a <<8|0x03,	0x46 <<8|0x00,	0x54 <<8|0x00,	0x44 <<8|0x00,
					0x49 <<8|0x00,	0x20 <<8|0x03,	0x46 <<8|0x00,	0x54 <<8|0x00,	0x32 <<8|0x00,//20
					0x33 <<8|0x00,	0x32 <<8|0x00,	0x52 <<8|0x00,	0x20 <<8|0x00,	0x55 <<8|0x00,
					0x53 <<8|0x00,	0x42 <<8|0x00,	0x20 <<8|0x00,	0x55 <<8|0x00,	0x41 <<8|0x00,//30
					0x52 <<8|0x00,	0x54 <<8|0x00,	0x12 <<8|0x03,	0x41 <<8|0x00,	0x39 <<8|0x00,
					0x30 <<8|0x00,	0x30 <<8|0x00,	0x67 <<8|0x00,	0x61 <<8|0x00,	0x73 <<8|0x00,//40
					0x39 <<8|0x00,	0xeb <<8|0xcb,	0x3a <<8|0x90,	0x00 <<8|0x00,	0x00 <<8|0x00,
					0x00 <<8|0x00,	0x00 <<8|0x00,	0x00 <<8|0x00,	0x00 <<8|0x00,	0x00 <<8|0x00,//50
					0x00 <<8|0x00,	0x00 <<8|0x00,	0x00 <<8|0x00,	0x00 <<8|0x00,	0x00 <<8|0x00,
					0x00 <<8|0x00,	0x00 <<8|0x00,	0x00 <<8|0x00,	0x00 <<8|0x00,	0x00 <<8|0x00,//60
					0x00 <<8|0x00,	0x00 <<8|0x00,	0xa0 <<8|0x2a
};

//-----------------------------grub searchbytes------------------------------------------------

const uint8_t grub_init_string[] PROGMEM = {0x47,0x4E,0x55,0x20,0x47,0x52,0x55,0x42};
//47 4E 55 20 47 52 55 42 //GNU GRUB

/*
boot_var's
0 = start menu without nativedisk  (fallback if default dont load complete ... com error to usb serial) 
1 = start windows at once  (no nativedisk modul) and set boot_var to 2 (default)
2 = (default) start menu with nativedisk
3 = start linux at once

*/

//---------------------------- boot windows - Commands to Host-------------------------------------------------
uint8_t win_size = 2;
const char win_comm_1[] PROGMEM = "c\r";  
const char win_comm_2[] PROGMEM = "set boot_var=1 \r save_env boot_var \r clear \r reboot \r"; // starte windows 

PGM_P const win_mess_table[] PROGMEM = 
{
	 win_comm_1
	,win_comm_2
} ;

//---------------------------- boot linux - Commands to Host-------------------------------------------------
uint8_t lin_size = 2;
const char lin_comm_1[] PROGMEM = "c\r";  
const char lin_comm_2[] PROGMEM = "set boot_var=3 \r save_env boot_var \r clear \r reboot \r";  // starte linux 

PGM_P const lin_mess_table[] PROGMEM =
{
	 lin_comm_1
	,lin_comm_2
};

//---------------------------- boot no entry Linux - Commands to Host-------------------------------------------------
uint8_t se_size = 3;
const char se_comm_1[] PROGMEM = "c\r"; // 
const char se_comm_2[] PROGMEM = "linux (hd0,gpt2)/pup/vmlinuz root=/dev/sda2/pup psubdir=/pup/ \r"; //    start puppylinux		das (hd0,gpt2) muss wahrscheinlich noch angepasst werden weil ja dann kein efi mehr läuft
const char se_comm_3[] PROGMEM = "initrd (hd0,gpt2)/pup/initrd.gz \r boot\r"; //

PGM_P const se_mess_table[] PROGMEM =
{
	 se_comm_1
	,se_comm_2
	,se_comm_3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
	
};


//--------------------------------STATE------------------------------------------------

uint8_t  latency = 0;	// lateny value of the ftdi driver

int8_t os_message_counter = -1; // welche os startmessage schon gesedet wurde 

uint8_t state = 0;	//(0 << 0)| //time to send // wird vom timer1 gesetzt und löst meldung an den host aus
					//(0 << 1)| //select windows  
					//(0 << 2)| //select linux 
					//(0 << 3)|	//os identify bit
					//(0 << 4)| //os identify bit
					//(0 << 5)| // Grub is init
					//(0 << 6)| 
					//(0 << 7)) 
				
// state |= (1 << Bitnummer);  // Hiermit wird ein Bit in x gesetzt
// state &= ~(1 << Bitnummer); // Hiermit wird ein Bit in x geloescht

//-------------------------IO-Buffer------------------------------------------------------

/** Circular buffer to hold data from the host before it is sent to the device via the serial port. */
static RingBuffer_t USBHostToDevice_Buffer;

/** Underlying data buffer for \ref USBHostToDevice_Buffer, where the stored bytes are located. */
static uint8_t      USBHostToDevice_Buffer_Data[128];

/** Circular buffer to hold data from the serial port before it is sent to the host. */
static RingBuffer_t USBDeviceToHost_Buffer;

/** Underlying data buffer for \ref USBDeviceToHost_Buffer, where the stored bytes are located. */
static uint8_t      USBDeviceToHost_Buffer_Data[128];

#if defined(debug)
//------------------------------ debug serial out ---------------------------

static RingBuffer_t debugUSART_Buffer;

static uint8_t      debugUSART_Buffer_Data[256];

//---------------------------------------------------------------------------
	
#endif



///////////////////////////////////////////////////////////////////////////////////////////////
/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();
	
	RingBuffer_InitBuffer(&USBHostToDevice_Buffer, USBHostToDevice_Buffer_Data, sizeof(USBHostToDevice_Buffer_Data));
	RingBuffer_InitBuffer(&USBDeviceToHost_Buffer, USBDeviceToHost_Buffer_Data, sizeof(USBDeviceToHost_Buffer_Data));
	
#if defined(debug)
			
	RingBuffer_InitBuffer(&debugUSART_Buffer, debugUSART_Buffer_Data, sizeof(debugUSART_Buffer_Data));

#endif
	
	GlobalInterruptEnable();

	for (;;)
	{
		// TODO: buttonabfragen durch interrupt ersetzen ? ändert sich ja später eh durch wahl des eingabemediums
		
		if(PINB & (1 << 6) ) {												// if pin D10 is high
			state |= (1 << 1);												//if only... select windows to boot 
		}
			
		if(PINB & (1 << 7) ) {												// if pin D11 is high  
			state |= (1 << 2);												//if only... select linux to boot					
		}
		
																			// together select a third system
			
		receiveFromHost();  
		
		if(state & ( 1 << 0 )){												//sende neues datenpacket zum Host wenn timer abgelaufen  
			 sendToHost();
			 state &= ~(1 << 0 );											// reset timetosendflag	
		}
		
		if(state & ( 1 << 5 )){												// fülle USBDeviceToHost_Buffer mit befehlen , sobald klar ist das der host sie verarbeiten kann 
			 		
			startOS();
		}
		
		
		
#if defined(debug_bridge)													//serial bridge between usb_serial - uart     breaks the grub mechanism? 
		
		sendHostOutBufferToUsart();
		receiveUsartToHostInBuffer();
	
#endif
		
		USB_USBTask();
		
#if defined(debug)
		
		if(RingBuffer_GetCount(&debugUSART_Buffer))   
		{
	
				if(Serial_IsSendReady())
				Serial_SendByte(RingBuffer_Remove(&debugUSART_Buffer));
			
		}
		
#endif

	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
#elif (ARCH == ARCH_XMEGA)
	/* Start the PLL to multiply the 2MHz RC oscillator to 32MHz and switch the CPU core to run from it */
	XMEGACLK_StartPLL(CLOCK_SRC_INT_RC2MHZ, 2000000, F_CPU);
	XMEGACLK_SetCPUClockSource(CLOCK_SRC_PLL);

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, F_USB);

	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
#endif
 

	/* Hardware Initialization */
	Serial_Init(115200, true);
	
	LEDs_Init();
	
	USB_Init();
	
	DDRB &= ~(1 << 6);      // damit ist dann PB1 ein Eingang
	PORTB &= ~(1 << 6);     // PB1 als Tri-State
		
    DDRB &= ~(1 << 7);      // damit ist dann PB1 ein Eingang
    PORTB &= ~(1 << 7);     // PB1 als Tri-State
	
}

/** Event handler for the USB_Connect event. This indicates that the device is enumerating via the status LEDs and
 *  starts the library USB task to begin the enumeration and USB management process.
 */
void EVENT_USB_Device_Connect(void)
{

}

/** Event handler for the USB_Disconnect event. This indicates that the device is no longer connected to a host via
 *  the status LEDs and stops the USB management 
 */
void EVENT_USB_Device_Disconnect(void)
{

}

/** Event handler for the USB_ConfigurationChanged event. This is fired when the host set the current configuration
 *  of the USB device after enumeration - the device endpoints are configured 
 */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	// Setup Data Endpoints */
	ConfigSuccess &= Endpoint_ConfigureEndpoint(RX_EPADDR, EP_TYPE_BULK, TXRX_EPSIZE, 1);
	ConfigSuccess &= Endpoint_ConfigureEndpoint(TX_EPADDR, EP_TYPE_BULK, TXRX_EPSIZE, 1);
	
	timer1(5, 250,  setSendFlag);		//alle 16ms							//(preescaler,ticks,function)
																			//1 tick = 4 us (microsekunden); //7812 ~ 500ms  max 65535

}

/** Event handler for the USB_ControlRequest event. This is used to catch and process control requests sent to
 *  the device from the USB host before passing along unhandled control requests to the library for processing
 *  internally.
 */
void EVENT_USB_Device_ControlRequest(void)
{

#if defined(debug_request)

//-----debugging over uart__start------------------------------------------------------
// send the recieved usb control messages to the uart buffer

	//RingBuffer_Insert(&debugUSART_Buffer, 'r');
	char buff0[16];
	uint8_t 7+data[] = {
					USB_ControlRequest.bmRequestType,
					USB_ControlRequest.bRequest,
					USB_ControlRequest.wValue,  
					USB_ControlRequest.wIndex,	
					USB_ControlRequest.wLength};
				
	RingBuffer_Insert(&debugUSART_Buffer, '\r');
	
	for(int y=0;y<5;y++){
		
		itoa(data[y], buff0, 16);
	
		for(int i=0;i<9;i++){
			if(buff0[i]!= '\0')RingBuffer_Insert(&debugUSART_Buffer, buff0[i]);
			else i=10;
		}
		RingBuffer_Insert(&debugUSART_Buffer, ',');
	}
	
	RingBuffer_Insert(&debugUSART_Buffer, '\r');
	
#endif

//---------------------------------------------------------------------------------

	/* Process FTDI specific control requests */
	/*sourceparts :https://dev.nf-design.eu/nfdesign/qemu-xen-traditional/raw/99e3a03d860ed650300b9aaf7d004e90c299ce96/hw/usb-serial.c */
	
	if(USB_ControlRequest.bmRequestType & ( 1 << 6 )) // true if vendorspecific bit is set (0x40,0xc0)
	{	
		switch (USB_ControlRequest.bRequest)
		{
						
			case  FTDI_SIO_RESET: /*0 Reset the port */ 
					
				Endpoint_ClearSETUP();
				// send packet
				Endpoint_ClearIN();
				// and mark the whole request as successful:
				Endpoint_ClearStatusStage();
							
			break;

			case	FTDI_SIO_MODEM_CTRL: /*1 Set the modem control register */ 
					
				Endpoint_ClearSETUP();
				//https://en.wikipedia.org/wiki/RS-232#RTS,_CTS,_and_RTR
				/*
				if (USB_ControlRequest.wValue & (2 << 8)) {		//FTDI_RTS
					if (USB_ControlRequest.wValue & 2)
					mdmControlFlag |= 0x004;					//CHR_TIOCM_RTS; request to send (RTS)
					else
					mdmControlFlag &= ~0x004;					//CHR_TIOCM_RTS;
				}
				if (USB_ControlRequest.wValue & (1 << 8)) {		//FTDI_DTR
					if (USB_ControlRequest.wValue & 1)
					mdmControlFlag |= 0x002	;					//CHR_TIOCM_DTR;  data terminal ready (DTR)
					else
					mdmControlFlag &= ~0x002;					//CHR_TIOCM_DTR;
				}
				*/
				
				Endpoint_ClearIN();
				
				Endpoint_ClearStatusStage();
						
			break;

			case	FTDI_SIO_SET_FLOW_CTRL: /*2 Set flow control register */
					
				Endpoint_ClearSETUP();
											
				// send packet
				Endpoint_ClearIN();
				// and mark the whole request as successful:
				Endpoint_ClearStatusStage();
						
			break;

			case	FTDI_SIO_SET_BAUD_RATE: /*3 Set baud rate */
					
				Endpoint_ClearSETUP();
											
				//	LineEncoding.BaudRateBPS = ((uint32_t)USB_ControlRequest.wValue << 16) | USB_ControlRequest.wIndex;
				static const int subdivisors8[8] = { 0, 4, 2, 1, 3, 5, 6, 7 };
				int subdivisor8 = subdivisors8[((USB_ControlRequest.wValue & 0xc000) >> 14) | ((USB_ControlRequest.wIndex & 1) << 2)];
										 
				int divisor = USB_ControlRequest.wValue & 0x3fff;

				/* chip special cases */
				if (divisor == 1 && subdivisor8 == 0)
				subdivisor8 = 4;
				if (divisor == 0 && subdivisor8 == 0)
				divisor = 1;
				
				/* diese beiden zeile stellen den uart auf die vom host geforderte baudrate. interessant für den bridgemode*/						 
				//Serial_Disable();	
				//Serial_Init((48000000 / 2) / (8 * divisor + subdivisor8), true);
						
				
				Endpoint_ClearIN();
				
				Endpoint_ClearStatusStage();
										 
#if defined(debug)
						// gibt die vom host eingestelle baudrate im als debuggernachricht aus	
						/*			 	
				char buff0[16];
				ltoa((48000000 / 2) / (8 * divisor + subdivisor8), buff0, 10);
										 
				RingBuffer_Insert(&debugUSART_Buffer, '\r');
											
				for(int i=0;i<17;i++){
					if(buff0[i]!= '\0')
					{
						RingBuffer_Insert(&debugUSART_Buffer, buff0[i]);
					}
					else i=17;
				}
										
				RingBuffer_Insert(&debugUSART_Buffer, '\r');
				*/
															
#endif
											
			break;

			case    FTDI_SIO_SET_DATA: /*4 Set the data characteristics of the port */
					
				Endpoint_ClearSETUP();
				
				Endpoint_ClearIN();
				
				Endpoint_ClearStatusStage();
			break;

			case	FTDI_SIO_GET_MODEM_STATUS: /*5 Retrieve current value of modem status registe   r */
				Endpoint_ClearSETUP();

				Endpoint_Write_16_BE(0x01 <<8|0x70);
										
				Endpoint_ClearIN();
			
				Endpoint_ClearStatusStage();
						
			break;

			case	FTDI_SIO_SET_EVENT_CHAR: /*6 Set the event character */
					
				Endpoint_ClearSETUP();

				Endpoint_ClearIN();
				
				Endpoint_ClearStatusStage();
						
			break;

			case    FTDI_SIO_SET_ERROR_CHAR: /*7 Set the error character */
					
				Endpoint_ClearSETUP();

				Endpoint_ClearIN();
				
				Endpoint_ClearStatusStage();
						
			break;

			case	FTDI_SIO_SET_LATENCY_TIMER: /*9 Set the latency timer */
					
				Endpoint_ClearSETUP();

				latency = USB_ControlRequest.wValue;

				Endpoint_ClearIN();
				
				Endpoint_ClearStatusStage();
						
			break;

			case	FTDI_SIO_GET_LATENCY_TIMER: /*0x0a Get the latency timer */
					
				Endpoint_ClearSETUP();
														
				Endpoint_Write_8(latency);	
																	
				Endpoint_ClearIN();

				Endpoint_ClearStatusStage();
											
			break;

			case	FTDI_SIO_SET_BITMODE: /*0x0b Set bitbang mode */
					
				Endpoint_ClearSETUP();
				
				Endpoint_ClearIN();
				
				Endpoint_ClearStatusStage();
					
			break;

			case	FTDI_SIO_READ_PINS: /*0x0c Read immediate value of pins */
					
				Endpoint_ClearSETUP();
				
				Endpoint_ClearIN();
				
				Endpoint_ClearStatusStage();
					
			break;
					
			case FTDI_SIO_READ_EEPROM:	 /*0x90 read eeprom*/  // used by windows driver
			
				Endpoint_ClearSETUP();
						
				//wIndex wird bei jeder anfrage um 1 erhöht
				
				Endpoint_Write_16_BE(pgm_read_word(&(FTDI_eeprom[USB_ControlRequest.wIndex])));
						
				Endpoint_ClearIN();
 
				Endpoint_ClearStatusStage();

			break;
					
		}
		
	}
	
	findOS();

}


//------------------------Timer Funktionen zum senden der Device to Host Pakete---------------------------------------------------------

/*https://github.com/jvalrog/atmega-timers*/

void (*_t1_func)();

void timer1(uint8_t prescaler, uint16_t ticks, void (*f)()) {
	TIMSK1 &= ~(_BV(OCIE1A));
	_t1_func = f;
	OCR1A = ticks;
	TCCR1A = 0;
	TCCR1B = prescaler | _BV(WGM12);
	TCNT1 = 0;
	TIMSK1 |= _BV(OCIE1A);
}

void timer1_stop() {
	TCCR1B = 0;
}


ISR(TIMER1_COMPA_vect) {
	_t1_func();
}



//---------------------------------Betriebssystemerkennung------------------------------------------------
// jedes system welches einen FTDI rs232 nutzen will unterscheidet sich in der weise ihn zu initialisieren
// jedes system sendet mehr oder weniger charakteristische requests (oder im fall von grub fast keine)

void findOS(void)
{	if((USB_ControlRequest.bmRequestType & ( 1 << 6 ))){
		//c0//40,90 Windows
		if(USB_ControlRequest.bRequest == 0x90 /*&& USB_ControlRequest.bmRequestType & ( 1 << 6 )*/ ){
			
			state |= (1 << 3);
			state |= (1 << 4);
			
		}
		
		//c0/40,4,8,0 Android
		if(USB_ControlRequest.bRequest == 0x04 && USB_ControlRequest.wValue == 0x08 /*&& USB_ControlRequest.bmRequestType & ( 1 << 6 )*/){ // true if vendorspecific bit is set (0x40,0xc0))
			//set to android
				
			state &= ~(1 << 3); //0
			state |= (1 << 4);  //1
		}
		
	}else{
		
			// 80/0,0 Linux
			if(!USB_ControlRequest.bRequest /*&& !(USB_ControlRequest.bmRequestType & ( 1 << 6 ))*/ ) {
				//set to linux
				state |= (1 << 3);	//1
				state &= ~(1 << 4); //0
			}	
		
	}
	
}

//---------------------------------USB I/O Funktionen------------------------------------------------
void receiveFromHost(void)
{
	
	
	if (!(RingBuffer_IsFull(&USBHostToDevice_Buffer)))								//Only try to read in bytes from the interface if the transmit buffer is not full 
	{
		int16_t ReceivedByte = -1;
		
		Endpoint_SelectEndpoint(RX_EPADDR);

		if (Endpoint_IsOUTReceived())
		{
			if (Endpoint_BytesInEndpoint())	
			{
				ReceivedByte = Endpoint_Read_8();
				if(ReceivedByte >= 0)	RingBuffer_Insert(&USBHostToDevice_Buffer, ReceivedByte);
				
			}
			
			if (!(Endpoint_BytesInEndpoint()))										//wenn endpoint komplett ausgelesen ist
			{	
		
			    Endpoint_ClearOUT();
				processFromHost();
				
			}
		}
		
	}
	
}

void sendToHost(void){																// wenn nichts zu senden ist wied immer 0x0170/0x0110 gesendet
																					//wenn inhalt gesendet wird immer 0x0170+payload gesendet
 	
	Endpoint_SelectEndpoint(TX_EPADDR);
			
	if (!Endpoint_IsINReady())return;
	
	//-------------------------- Modem status bits
		
	//Device_SendByte(0x01);
		
	Device_SendByte(
		((1 << 0)| //data ready
		 //(0 << 1)|
		 //(0 << 2)|
		 //(0 << 3)|
		 //(0 << 4)|
		 //(0 << 5)|
		 //(0 << 6)|
		 (0 << 7))
	);  
	
	if( RingBuffer_GetCount(&USBHostToDevice_Buffer) < 65 )
	{			
		//Device_SendByte(0x70);		
		Device_SendByte(
			(//(0 << 0)|
			 //(0 << 1)|
		 	 //(0 << 2)|
			 //(0 << 3)|
			 (1 << 4)| //CTS line status
			 (1 << 5)| //DSR line status										//if ftdi recieve many data = 0
			 (1 << 6)| //RI line status											//if ftdi recieve many data = 0
			 (0 << 7))
		);
			
	}else
	{
		//Device_SendByte(0x10);
		Device_SendByte(
			(//(0 << 0)|
			 //(0 << 1)|
			 //(0 << 2)|
			 //(0 << 3)|
			 (1 << 4)| //CTS line status
			 (0 << 5)| //DSR line status	
			 (0 << 6)| //RI line status		
			 (0 << 7))
		);	
	}
			
			
	uint16_t BufferCount = RingBuffer_GetCount(&USBDeviceToHost_Buffer);

	if(BufferCount)
	{
			
		uint8_t BytesToSend = MIN(BufferCount , (TXRX_EPSIZE - 3)); 
		
		while (BytesToSend--)
		{

			if (Device_SendByte(RingBuffer_Peek(&USBDeviceToHost_Buffer)) != ENDPOINT_READYWAIT_NoError)
			{
					
				break;
			}
				
			RingBuffer_Remove(&USBDeviceToHost_Buffer);
		}
		//--------------------------------------------------------------
	}
		Endpoint_ClearIN();
	
}

uint8_t Device_SendByte(const uint8_t Data)
{
	if (USB_DeviceState != DEVICE_STATE_Configured )
	return ENDPOINT_RWSTREAM_DeviceDisconnected;

	Endpoint_SelectEndpoint(TX_EPADDR);

	if (!(Endpoint_IsReadWriteAllowed()))
	{
		Endpoint_ClearIN();

		uint8_t ErrorCode;

		if ((ErrorCode = Endpoint_WaitUntilReady()) != ENDPOINT_READYWAIT_NoError)
		return ErrorCode;
	}
	
	Endpoint_Write_8(Data);
	return ENDPOINT_READYWAIT_NoError;
}


void setSendFlag(void)
{
	
	state |= (1 << 0);															//setzt statebit für timetosend im state
	
}



//---------------------------------USART IO Funktionen------------------------------------------------

void receiveUsartToHostInBuffer(void)
{
	
	if(Serial_IsCharReceived())
	{
		
		uint8_t ReceivedByte =  Serial_ReceiveByte(); //UDR1;
		
		if ((USB_DeviceState == DEVICE_STATE_Configured) && !(RingBuffer_IsFull(&USBDeviceToHost_Buffer)))
		{
			RingBuffer_Insert(&USBDeviceToHost_Buffer, ReceivedByte);
		}
		
	}

}


void sendHostOutBufferToUsart(void)												// Load the next byte from the Host transmit buffer into the USART if transmit buffer space is available
{
																				 
	if (Serial_IsSendReady() && !(RingBuffer_IsEmpty(&USBHostToDevice_Buffer))) 
	{
		Serial_SendByte(RingBuffer_Remove(&USBHostToDevice_Buffer));
	}
	
}

//--------------------------------- Procces Host-to-Device Packets ------------------------------------------------

void processFromHost(void)
{																				//wenn noch nicht festgestellt wurde ob grub Eingaben entgegen nehmen kann
																				//suche nach zeichen die nur grub menüoutput sein können
	if(!(state & ( 1 << 5 )))
	{
		
		searchInitGrub();    
		
	}
	else
	{
		
		while(RingBuffer_GetCount(&USBHostToDevice_Buffer))						// lösche gesamten ringbuffer
		{
			RingBuffer_Remove(&USBHostToDevice_Buffer);
		}
	}
	
}


void searchInitGrub(void){ 
	
	uint8_t buffer = 0;
	uint8_t buffer2 = 0;
	uint8_t buffer3 = 0;
	int y = 0;
	uint8_t count = RingBuffer_GetCount(&USBHostToDevice_Buffer);
	
	for(int i = 0 ; i<count ; i++)												// wiederhole für jedes im buffer vorhandene zeichen
	{
		buffer = RingBuffer_Remove(&USBHostToDevice_Buffer);
		buffer2 = pgm_read_word(&(grub_init_string[y]));
		buffer3 = pgm_read_word(&(grub_init_string[0]));													
																				//grub_init_string == 47 4E 55 20 47 52 55 42 //GNU GRUB
		
		if(buffer2 == buffer)													//ptüfe ob  byte im buffer zum gesuchten string passt 
		{   
		
			RingBuffer_Insert(&USBHostToDevice_Buffer, buffer);
			y++;																//wenn ja erhöhe um 1 um zu schauen ob die nächsten bytes auch passen 
		}
		else
		{
			if(buffer3 == buffer)
			{	
				RingBuffer_Insert(&USBHostToDevice_Buffer, buffer);
				y = 1;															// wenn übereinstimmung mit erstem byte 
			}
			else
			{
				y = 0;															// setze index von suchstringarray wieder auf null wenn keine übereinstimmung
			}
																					
		}
				
								
		if(y>7)																	//wenn der gesammte string drin vorkommt 
		{
			state |= (1 << 5);													// set searchInitGrubBit to 1  -- das heist wir wissen wir sind definitiv in grub sind und empfangen die menüseite also können wir nun anfangen befehle zu senden
			
			break;		
		}
		
	}
	
	if(y == 0 || y>7)															// y<7 = sequenz gefunden  y==0 = keine bytes mit richtiger reihenfolge im buffer 
																				// alles von 1-6 hießt es könnten die letzten bytes passen und der rest folgt im nächsten paket
	{
		
		while(RingBuffer_GetCount(&USBHostToDevice_Buffer))						// lösche gesamten ringbuffer
		{
			RingBuffer_Remove(&USBHostToDevice_Buffer);
		}
		
	}
	
}




//---------------------------------Device-to-Host Packets------------------------------------------------



void startOS(void){
	
	if(!RingBuffer_IsEmpty(&USBDeviceToHost_Buffer))return;
	
	if(os_message_counter == 0)return;	// wenn os_message_counter ==0 müssten alle zeilen gesendet worden sein und wir können abbrechen
	
		// eigendlich !(state & ( 1 << 3 )) && !(state & ( 1 << 4 )) aber osSelect funktioniert nicht richtig
	if( !(state & ( 1 << 3 ))  ) // wenn grub als system erkannt wurde
	{	 
	  
		if(!(state & ( 1 << 1 )) && !(state & ( 1 << 2 )) ) return; // wenn kein os gewählt mache nichts
		
		uint16_t pointer0 = 0;
		uint8_t size = 0;
		
		
		if( (state & ( 1 << 1 )) && !(state & ( 1 << 2 )) ) // wenn select windows statebit gesetzt und select linux nicht gesetzt --- starte windows
		{
			
			pointer0 = (int) &(win_mess_table[0]) ;
			size = win_size;	//anzahl der arrayeintäge
			
		}
		
		if( !(state & ( 1 << 1 )) && (state & ( 1 << 2 )) ) // wenn select linux statebit gesetzt und select windows nicht gesetzt --- starte linux
		{
			
			pointer0 = (int) &(lin_mess_table[0]) ;
			size = lin_size;	//anzahl der arrayeintäge 
			
		}
		
		if((state & ( 1 << 1 )) && (state & ( 1 << 2 )) ) // wenn select linux statebit gesetzt und select windows nicht gesetzt --- starte anderes os linux
		{
			
			pointer0 = (int) &(se_mess_table[0]) ;
			size = se_size;	//anzahl der arrayeintäge
			
		}
		
		// wenn os_message_counter ==  -1 (initzustand) setze ihn auf anzahl der zeilen
		if(os_message_counter < 0) os_message_counter = size; // wenn messagezähler noch nicht gesetzt. setze ihn
		

		//pointer = (PGM_P)pgm_read_word( &(pointer[size - os_message_counter]) );
		PGM_P pointer = (PGM_P)pgm_read_word( pointer0+((size - os_message_counter)*2) ); //*2 weil pointer 16bit = 2 byte // für nächste pointeradresse muss man immer 2 byte weiter springen

		//RingBuffer_Insert(&debugUSART_Buffer, os_message_counter); //0x01
		
		
		uint8_t buffer = 0;
		
		for(;;pointer++){	//übertrage jedes byte des chararrays (zb win_comm_1) auf das der pointer zeigt bis der nullterminator erreicht ist 
			 
			 buffer = pgm_read_byte( pointer );
			 
			 if(buffer != '\0'  && !(RingBuffer_IsFull(&USBDeviceToHost_Buffer)))
			 {
				 RingBuffer_Insert(&USBDeviceToHost_Buffer, buffer);
				 //RingBuffer_Insert(&debugUSART_Buffer, buffer);
			 }
			 else
			 {
				 break;
			 }
		 } 
		 
		 os_message_counter--; 
		 
		 if( os_message_counter == 0)
		{
			
			state &= ~(1 << 5 );
		
		}
		
	}
	
	
}
