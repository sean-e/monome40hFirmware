# monome40hFirmware
This repro originated from https://github.com/sean-e/mTroll/tree/master/Monome40h /40hFirmware

The original monome 40h source was not originally hosted at GitHub until a few years ago -- see https://github.com/monome/40h/ 

The submodules directory references https://github.com/sean-e/NeoPixelBus and https://github.com/sean-e/ArduinoCore-avr, which are forks of repos modified for use on the monome board (dependent on NeoPixelBus for use of NeoPixel LED strips, while NeoPixelBus is dependent upon ArduinoCore, though I'm using AtmelStudio and not the Arduino IDE to compile).  Pull the submodules via `git submodule update --init --recursive`
