To flash the monome:

Prerequisites:
- jtag device
- avarice.exe + cygwin1.dll (from install\bin of winavr)

Steps:
- plug jtag into pc usb
- connect jtag to monome (red side of cable to jtag connector pin 1)
- connect pc usb to monome (supplies power to board)
- jtag red led should be lit at this point
- open command prompt with administrative rights
- determine COM port of the jtag device and update command line "S0" text with appropriate value below:
	other OS, replace /dev/ttyS0 with /dev/tty.usb? (instead of COM1 per http://www.avrfreaks.net/wiki/index.php/Documentation:Things_That_Are_Broken#Mega128_Programming)
	On Windows, check Device Manager for COM assignment of the JTAG device (check monome first, then power up JTAG to differentiate between the 2 FTDI ports):
		S0 == COM1
		S1 == COM2
		S2 == COM3
		S3 == COM4
- run avarice, COM1 (S0) example:

	avarice --erase --program --file monome40h.hex --jtag /dev/ttyS0 --jtag-bitrate 125KHz

	(the path to the hex file must be absolute and fully specified, or in current dir)

	if it fails with permission denied, wait a few seconds and try the command again
