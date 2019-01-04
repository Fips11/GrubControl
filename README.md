

The goal is to create an interactive boot process independent of UEFI.  It would be possible to pre-select the operating system via additional hardware buttons when starting the computer. It would also be possible to boot LUKS encrypted partitions without entering the password or plaintext stored passwords by storing the password in the IC.

The firmware is able to recognize the currently running operating system in order to be able to perform other tasks (such as HID hotkeys) or not to accidentally release sensitive data of the boot process. 

Most laptops have internal USB interfaces to connect the atmega32u4.
Many integrated webcams or SD card readers are connected via USB, their connections can be used for example.

Hints: 
- only works with GRUB 2
- Booting Windows is a little bit hacky
- the Grub.d script 41_custom must be modified
- in some laptops the webcam is supplied with 3.3v (the operation at 3.3 volts usually requires a 
lower clock frequency of 8mhz. This was not considered in the firmware yet ) 

- The current program size is currently: Program Memory Usage 	:	5284 bytes   16,1 % 
				                                 Data Memory Usage 		:	301 bytes   11,8 % 



TODO:
- Support for more than one device descriptor (for HID mode)
- 8 mhz support
- capacitive buttons
- bugfix in the detection of Android
- a better evaluation of the grub output
