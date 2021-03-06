# monome40hFirmware
This repro originated from https://github.com/sean-e/mTroll/tree/master/Monome40h /40hFirmware

The original monome 40h source was not hosted at GitHub until 2015 -- see https://github.com/monome/40h/ 

monome info: 
- https://monome.org/docs/grid/kits/40h-kit/
- https://web.archive.org/web/20130820105410/http://monome.org/docs/tech:kits:40h_kit 
- https://web.archive.org/web/20131023193104/http://monome.org/docs/tech:kits:kit_assembly

The submodules directory references https://github.com/sean-e/NeoPixelBus and https://github.com/sean-e/ArduinoCore-avr, which are forks of repos modified for use on the monome board (dependent on NeoPixelBus for use of NeoPixel LED strips, while NeoPixelBus is dependent upon ArduinoCore, though I'm using AtmelStudio and not the Arduino IDE to compile).  Pull the submodules via `git submodule update --init --recursive`

## How is this different than the original 40h firmware?
- Improved ADC handling for use with expression pedals (2007)
- Support for 3-wire RGB NeoPixel LEDs (2020)
- Updated serial protocol for RGB LED support (2020)

## To Use 3-wire RGB NeoPixel LEDs
- in 40h.cpp, set `kLedStripPixelCount` to the number of LEDs to be used (max 40h)
- in 40h.cpp, update `kLedMatrix` for your matrix (keep the matrix 8x8, but assign the value `kInvalidPixel` to any position that should be considered empty; in general, the non-invalid values should be in order matching the LED strip order and should not repeat)
- Remove the MAX7219 IC from the socket on the monome board
- Compile the source and flash the board
- Add a jumper between NeoPixel data in and pin 1 of the former MAX7219 socket
- Add a jumper between NeoPixel ground and pin 4 of the former MAX7219 socket
- Add a jumper between NeoPixel vcc and pin 19 of the former MAX7219 socket (opposite of pin 1, count back from 24)

## Updated Serial Protocol
(There are 32 preset color slots which can be changed at runtime.)

This command no longer has any effect:
- kMessageTypeLedIntensity

These commands work as before with the color of the LED taken from preset slot 0:
- kMessageTypeLedStateChange
- kMessageTypeLedSetRow
- kMessageTypeLedSetColumn

This command now accepts a parameter that specifies a particular test pattern:
- kMessageTypeLedTest                      // 2 test pattern values: 10, 11 (see RunPixelTest in 40h.cpp)

These are new commands:
- kMessageTypeLedRgbOn                     // enable LED using specified RGB value
- kMessageTypeUpdatePresetGroup1           // set preset color slot to specified RGB value (for slots 0 - 15)
- kMessageTypeLedOnPresetGroup1            // enable LED using preset color slot 0-15
- kMessageTypeUpdatePresetGroup2           // set preset color slot to specified RGB value (slots 16 - 31 specified as 0 - 15)
- kMessageTypeLedOnPresetGroup2            // enable LED using preset color slot 16-31 (specified as 0 - 15)
