To flash the monome:

Prerequisites:
- jtag
- winavr for avarice (avarice.exe + cygwin1.dll from install\bin)

Steps:
- plug jtag into serial port
- connect to monome (red side of cable to jtag connector pin 1)
- connect pc usb to monome (supplies power to board)
- jtag red led should be lit at this point
- run avarice:

makefile rule was:
avarice --erase --program --file $(TARGET).hex --jtag /dev/tty.usbserial-1B1 --jtag-bitrate 500KHz

for windows, replace /dev/tty.usb* with /dev/ttyS0 (instead of COM1 per http://www.avrfreaks.net/wiki/index.php/Documentation:Things_That_Are_Broken#Mega128_Programming)
	Check Device Manager for COM assignment:
		S0 == COM1
		S1 == COM2
		S2 == COM3
		S3 == COM4
avarice --erase --program --file monome40h.hex --jtag /dev/ttyS0 --jtag-bitrate 500KHz

path to hex file must be absolute or in current dir:
d:avarice --erase --program --file monome40h.hex --jtag /dev/ttyS2 --jtag-bitrate 500KHz
