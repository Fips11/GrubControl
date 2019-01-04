

The goal is to create an interactive boot process independent of UEFI.  It would be possible to pre-select the operating system via additional hardware buttons when starting the computer. It would also be possible to boot LUKS encrypted partitions without entering the password or plaintext stored passwords by storing the password in the IC.

The firmware is able to recognize the currently running operating system in order to be able to perform other tasks (such as HID hotkeys) or not to accidentally release sensitive data of the boot process. 

Most laptops have internal USB interfaces to connect the atmega32u4.
Many integrated webcams or SD card readers are connected via USB, their connections can be used for example.

