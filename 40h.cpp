/* 
 *  Copyright (C) 2006, Brian Crabtree and Joe Lake, monome.org
 * 
 *  This file is part of 40h.
 *
 *  40h is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  40h is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with 40h; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *  $Id: 40h.c,v. 1.1.1.1 2006/05/02 1:01:22
 */

// #define F_CPU 16000000UL // moved to project definition
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "config.h"
#include "message.h"
#include "adc.h"
#include "button.h"

#if defined(ENABLE_NEOPIXELBUS)
#include <NeoPixelBus.h>
#elif defined(ENABLE_FASTLED)
#include <FastLED.h>
#endif

struct io_pin_t 
{
    enum port_num { A, B, C, D } port;
    uint8 pin;
};

inline void output_pin(io_pin_t iopin, bool state)    
{
    if (state) {
        switch (iopin.port) {
        case io_pin_t::A:
            PORTA |= (1 << iopin.pin);
            break;
        case io_pin_t::B:
            PORTB |= (1 << iopin.pin);
            break;
        case io_pin_t::C:
            PORTC |= (1 << iopin.pin);
            break;
        case io_pin_t::D:
            PORTD |= (1 << iopin.pin);
            break;
        }
    } else {
        switch (iopin.port) {
        case io_pin_t::A:
            PORTA &= ~(1 << iopin.pin);
            break;
        case io_pin_t::B:
            PORTB &= ~(1 << iopin.pin);
            break;
        case io_pin_t::C:
            PORTC &= ~(1 << iopin.pin);
            break;
        case io_pin_t::D:
            PORTD &= ~(1 << iopin.pin);
            break;
        }
    } 
}

bool input_pin(io_pin_t pin)
{
    switch (pin.port) {
    case io_pin_t::A:
        return (PINA & (1 << pin.pin));                    
    case io_pin_t::B:
        return (PINB & (1 << pin.pin));
    case io_pin_t::C:
        return (PINC & (1 << pin.pin));
    case io_pin_t::D:
        return (PIND & (1 << pin.pin));
    }

    return 0;
}

#if defined(ENABLE_FASTLED) || defined(ENABLE_NEOPIXELBUS)
const uint16 kLedStripPixelCount = 17;
const uint8 kLedStripDataPin = 5; // MCU pin 6 / PB5 -> Arduino D5
// remove max7219 (it was used for SPI LEDs)
// 3 wire strip connections:
// data       from max7219 socket pin 1
// ground     from max7219 socket pin 4
// vcc        from max7219 socket pin 19 (opposite side of 1, count back from 24)
#endif

#if defined(ENABLE_NEOPIXELBUS)
const RgbColor kOffColor{ 0 };
constexpr uint8 kDimVal = 4;
constexpr uint8 kPresetCount = 32;
constexpr uint8 kPresetGroupSize = 16;
RgbColor gColorPresets[kPresetCount]
{
    // Group 1 (2 groups so that slot numbers fit in 4 bits of message header)
    { 0, 0, 0x3f },
    { 0, 0x3f, 0 },
    { 0x3f, 0, 0 },
    { 0, 0x3f, 0x3f },
    { 0x3f, 0x3f, 0 },
    { 0x3f, 0, 0x3f },
    { 0, 0, 0x7f },
    { 0, 0x7f, 0 },
    { 0x7f, 0, 0 },
    { 0, 0x7f, 0x7f },
    { 0x7f, 0x7f, 0 },
    { 0x7f, 0, 0x7f },
    { 0, 0, kDimVal },
    { 0, kDimVal, 0 },
    { kDimVal, 0, 0 },
    { kDimVal, 0, kDimVal },

    // Group 2
    { 0, 0, 0x1f },
    { 0, 0x1f, 0 },
    { 0x1f, 0, 0 },
    { 0, 0x1f, 0x1f },
    { 0x1f, 0x1f, 0 },
    { 0x1f, 0, 0x1f },
    { 0, 0, 0xf },
    { 0, 0xf, 0 },
    { 0xf, 0, 0 },
    { 0, 0xf, 0xf },
    { 0xf, 0xf, 0 },
    { 0xf, 0, 0xf },
    { 0, 0, kDimVal*2 },
    { 0, kDimVal*2, 0 },
    { kDimVal*2, 0, 0 },
    { kDimVal*2, 0, kDimVal*2 }
};

constexpr uint8 kMatrixRows = 8;
constexpr uint8 kMatrixCols = 8;
constexpr uint8 kInvalidPixel = -1;
// map of x/y coordinate to pixel strip ID (x == col, y == row)
constexpr uint8 kLedMatrix[kMatrixRows][kMatrixCols] 
{
    // my 5 column config; sparsely maps kLedStripPixelCount to the 8x8 matrix
    { 11, 12, 13, 14, 15, kInvalidPixel, kInvalidPixel, kInvalidPixel },
    { 10,  9,  8,  7,  6, kInvalidPixel, kInvalidPixel, kInvalidPixel },
    {  1,  2,  3,  4,  5, kInvalidPixel, kInvalidPixel, kInvalidPixel },
    { kInvalidPixel, kInvalidPixel, kInvalidPixel, kInvalidPixel, kInvalidPixel, 16, kInvalidPixel },
    { kInvalidPixel, kInvalidPixel, kInvalidPixel, kInvalidPixel, kInvalidPixel, kInvalidPixel, kInvalidPixel, kInvalidPixel },
    { kInvalidPixel, kInvalidPixel, kInvalidPixel, kInvalidPixel, kInvalidPixel, kInvalidPixel, kInvalidPixel, kInvalidPixel },
    { kInvalidPixel, kInvalidPixel, kInvalidPixel, kInvalidPixel, kInvalidPixel, kInvalidPixel, kInvalidPixel, kInvalidPixel },
    { kInvalidPixel, kInvalidPixel, kInvalidPixel, kInvalidPixel, kInvalidPixel, kInvalidPixel, kInvalidPixel, kInvalidPixel }
//     { 20, 19, 18, 17, kInvalidPixel, kInvalidPixel, kInvalidPixel, kInvalidPixel }, 
//     { 21, 22, 23, 24, kInvalidPixel, kInvalidPixel, kInvalidPixel, 0 }

//  {  0,  1,  2,  3,  4,  5,  6,  7 },
//  {  8,  9, 10, 11, 12, 13, 14, 15 },
//  { 16, 17, 18, 19, 20, 21, 22, 23 },
//  { 24, 25, 26, 27, 28, 29, 30, 31 },
//  { 32, 33, 34, 35, 36, 37, 38, 39 },
//  { 40, 41, 42, 43, 44, 45, 46, 47 },
//  { 48, 49, 50, 51, 52, 53, 54, 55 },
//  { 56, 57, 58, 59, 60, 61, 62, 63 }
};

using PixelStrip = NeoPixelBus<NeoRgbFeature, NeoAvr800KbpsMethod>;
void RunPixelTest(PixelStrip &strip, uint8 pattern);
#endif

struct SerialInputData
{
    // first two data bytes are not in the dataEx array for compatibility with t_message unpacking macros
    uint8 data0 = 0;
    uint8 data1 = 0;
    uint8 dataEx[3]{0};
    uint8 expectedLen = 0;
    uint8 readLen = 0;
    uint8 rx_roll = 0;

    static uint8 GetExpectedMessageLen(uint8 msgHeader)
    { 
        switch (msgHeader >> 4)
        {
        case kMessageTypeUpdatePresetGroup1:     return 4;
        case kMessageTypeUpdatePresetGroup2:     return 4;
        case kMessageTypeLedRgbOn:               return 5;
        default:                                 return 2;
        }
    }

    void Read(uint8 b)
    {
        switch (readLen)
        {
        case 0:
            data0 = b;
            expectedLen = GetExpectedMessageLen(b);
            break;
        case 1:
            data1 = b;
            break;
        default:
            if (readLen > 1 && readLen < 5) // this condition should never fail...
                dataEx[readLen - 2] = b;
        }
        ++readLen;
    }

    bool PacketReady() 
    {
        if (!readLen)
            return false;

        const bool ret = readLen == expectedLen;
        if (ret)
        {
            // reset for read of next packet, current packet data still usable until next read
            readLen = 0;
            rx_roll = 0;
        }

        return ret;
    }

    void CheckRoll() 
    {
        if (!readLen)
            return;

        // if single packet is "lost" trash it after a chance to get a match
        // CheckRoll is called outside of the inner RXF loop
        if (++rx_roll > 80)
        {
            rx_roll = 0;
            readLen = 0;
        }
    }
};


// #define DEBUG if you're going to debug over JTAG because we'll get caught in one of those
// while loops otherwise.  I think the JTAG interface must share a pin with the SPI, but I
// haven't confirmed this.  JAL 5/1/2006

#if defined(ENABLE_LED_SPI)
inline void spi_led(uint8 msb, uint8 lsb)
{
#if !defined(DEBUG)
    PORTB &= ~(1 << PB4);
    SPDR = msb;
    while (!(SPSR & (1 << SPIF)));
    SPDR = lsb;
    while (!(SPSR & (1 << SPIF)));
    PORTB |= (1 << PB4);
#endif
}
#endif

int main(void)
{
    uint8 i1, i2, i3;

    SerialInputData serial_in;
    t_message serial_out;

#if defined(ENABLE_LED_SPI)
    uint8 led_data[8];
    uint8 firstRun = true;
    io_pin_t ioPWREN{ io_pin_t::C, 0 };     // UMR245R PWE#
    DDRC &= ~(1 << ioPWREN.pin); PORTC |= (1 << ioPWREN.pin);
#elif defined(ENABLE_FASTLED)
    CRGB led_data[kLedStripPixelCount];
#endif

    io_pin_t ioRXF{ io_pin_t::B, 1 };         // UMR245R RXF
    io_pin_t ioRD{ io_pin_t::B, 2 };          // UMR245R RD
    io_pin_t ioWR{ io_pin_t::B, 3 };          // UMR245R WR

    io_pin_t ioLOAD{ io_pin_t::C, 6 };        // 74LS165 SH/LD
    io_pin_t ioDATA{ io_pin_t::C, 7};         // 74LS165 QH

    io_pin_t ioCLKSEL{ io_pin_t::A, 6 };      // 74LS164 CLK
    io_pin_t ioCLKIN{ io_pin_t::A, 7 };       // 74LS165 CLK
    io_pin_t ioSET{ io_pin_t::A, 5 };         // 74LS164 A

    DDRB &= ~(1 << ioRXF.pin); PORTB |= (1 << ioRXF.pin);
    DDRB |= (1 << ioRD.pin); PORTB |= (1 << ioRD.pin);
    DDRB |= (1 << ioWR.pin); PORTB |= (1 << ioWR.pin);

    DDRC |= (1 << ioLOAD.pin); PORTC |= (1 << ioLOAD.pin);
    DDRC &= ~(1 << ioDATA.pin); PORTC |= (1 << ioDATA.pin);

    DDRA |= (1 << ioCLKSEL.pin); PORTA |= (1 << ioCLKSEL.pin);
    DDRA |= (1 << ioCLKIN.pin); PORTA |= (1 << ioCLKIN.pin);
    DDRA |= (1 << ioSET.pin); PORTA |= (1 << ioSET.pin);

    output_pin(ioSET, 1);                // clear out row selector
    for (i1 = 0; i1 < 8; i1++) {
#if defined(ENABLE_LED_SPI)
        led_data[i1] = 0;
#endif
        output_pin(ioCLKSEL, 1);         // clear out row selector
        output_pin(ioCLKSEL, 0);
    }

    buttonInit();

#if defined(ENABLE_NEOPIXELBUS) || defined(ENABLE_FASTLED)
    init(); // for timer0 init, also calls sei() -- modified version of default Arduino code
#else
    sei(); // for ADC
#endif

#if defined(ENABLE_LED_SPI)
    // init SPI
    SPCR = (1 << SPE) | (1 << MSTR) | (SPI2X);
    DDRB |= (1 << PB5)|(1 << PB4)|(1 << PB7);
    
    spi_led(11, 7);                    // set scan limit to full range
    spi_led(10, 15);                   // set to max intensity
    for(i1 = 1; i1 < 9; i1++) {
        spi_led(i1, i1);               // print startup pattern 
    }
    spi_led(12, 1);                    // come out of shutdown mode 
    spi_led(15, 0);                    // test mode off
   
    for(i1=0;i1<64;) {
        spi_led(10, (64-i1)/4);        // set to max intensity
        if(!input_pin(ioPWREN)) i1++;  // wait for USB enumeration, to prevent auto-sleep
        _delay_ms(8);
    }

    for(i1 = 1; i1 < 9; i1++) 
        spi_led(i1, 0);                // clear led data
 
    spi_led(10, 15);                   // set to max intensity
#elif defined(ENABLE_NEOPIXELBUS)
    PixelStrip strip(kLedStripPixelCount, kLedStripDataPin);
    strip.Begin();
    RunPixelTest(strip, 10);
#elif defined(ENABLE_FASTLED)
    for (i1 = 0; i1 < kLedStripPixelCount; i1++)
        led_data[i1] = CRGB::Black;
    FastLED.addLeds<NEOPIXEL, kLedStripDataPin>(led_data, kLedStripPixelCount);

    for (i1 = 0; i1 < (int)kLedStripPixelCount; ++i1)
        led_data[i1] = CRGB::Blue;
    FastLED.show(); 
    delay(2000);

    for (i1 = 0; i1 < (int)kLedStripPixelCount; ++i1)
        led_data[i1] = CRGB::Black;
    FastLED.Show();
    delay(2000);
#endif

    // ******** main loop ********
    while (1) 
    {
        // read incoming serial **********************************************
        PORTD = 0;                      // setup PORTD for input
        DDRD = 0;                       // input w/ tristate

        while (!(input_pin(ioRXF))) 
        {
            output_pin(ioRD, 0);
            serial_in.Read(PIND);
            output_pin(ioRD, 1);
            if (serial_in.PacketReady())
            {
                uint8 msg_data0;
                
                // *********** process packet
                
                const uint8 kMsgType = messageGetType(serial_in);
                switch (kMsgType) {
                case kMessageTypeLedTest:
                    msg_data0 = messageGetLedTestState(serial_in);
#if defined(ENABLE_LED_SPI)
                    spi_led(15, msg_data0);
#elif defined(ENABLE_NEOPIXELBUS)
                    RunPixelTest(strip, msg_data0);
#endif
                    break;

                case kMessageTypeLedIntensity:
#if defined(ENABLE_LED_SPI)
                    msg_data0 = messageGetLedIntensity(serial_in);
                    spi_led(10, msg_data0);
#elif defined(ENABLE_NEOPIXELBUS)
                    // not supported
#endif
                    break;

                case kMessageTypeLedStateChange:
                    i1 = messageGetLedX(serial_in);
                    i2 = messageGetLedY(serial_in);

#if defined(ENABLE_LED_SPI)
                    if(messageGetLedState(serial_in) == 0)
                        led_data[i2] &= ~(1 << i1);
                    else
                        led_data[i2] |= (1 << i1);

                    spi_led(i2 + 1, led_data[i2]);
#elif defined(ENABLE_NEOPIXELBUS)
                    if (i2 < kMatrixRows && i1 < kMatrixCols)
                    {
                        if(messageGetLedState(serial_in) == 0)
                            strip.SetPixelColor(kLedMatrix[i2][i1], kOffColor);
                        else
                            strip.SetPixelColor(kLedMatrix[i2][i1], gColorPresets[0]);
                    }
#elif defined(ENABLE_FASTLED)
                    if(messageGetLedState(serial_in) == 0)
                        led_data[(i2 * 8) + i1] = CRGB::Black;
                    else
                        led_data[(i2 * 8) + i1] = CRGB::Blue;
#endif
                    break;

#if defined(ENABLE_NEOPIXELBUS)
                case kMessageTypeLedRgbOn:
                    i1 = messageGetLedX(serial_in);
                    i2 = messageGetLedY(serial_in);

                    if (i2 < kMatrixRows && i1 < kMatrixCols)
                        strip.SetPixelColor(kLedMatrix[i2][i1], RgbColor{serial_in.dataEx[0], serial_in.dataEx[1], serial_in.dataEx[2]});
                    break;

                case kMessageTypeLedOnPresetGroup1:
                case kMessageTypeLedOnPresetGroup2:
                    i1 = messageGetLedX(serial_in);
                    i2 = messageGetLedY(serial_in);
                    i3 = messageGetLedColorPreset(serial_in);
                    if (kMessageTypeLedOnPresetGroup2 == kMsgType)
                        i3 += kPresetGroupSize;

                    if (i3 < kPresetCount && i2 < kMatrixRows && i1 < kMatrixCols)
                        strip.SetPixelColor(kLedMatrix[i2][i1], gColorPresets[i3]);
                    break;

                case kMessageTypeUpdatePresetGroup1:
                case kMessageTypeUpdatePresetGroup2:
                    i1 = messageGetLedColorPreset(serial_in);
                    if (kMessageTypeUpdatePresetGroup2 == kMsgType)
                        i1 += kPresetGroupSize;

                    if (i1 < kPresetCount)
                        gColorPresets[i1] = RgbColor{serial_in.data1, serial_in.dataEx[0], serial_in.dataEx[1]};
                    break;
#endif

                case kMessageTypeAdcEnable:
                    if (messageGetAdcEnableState(serial_in))
                        enableAdc(messageGetAdcEnablePort(serial_in));
                    else
                        disableAdc(messageGetAdcEnablePort(serial_in));
                    break;

                case kMessageTypeShutdown:
#if defined(ENABLE_LED_SPI)
                    msg_data0 = messageGetShutdownState(serial_in);
                    spi_led(12, msg_data0);
#elif defined(ENABLE_NEOPIXELBUS)
                    strip.ClearTo(kOffColor);
                    strip.Show();
                    delay(500);
#elif defined(ENABLE_FASTLED)
                    for (i1 = 0; i1 < kLedStripPixelCount; ++i1)
                        led_data[i1] = CRGB::Black;
                    FastLED.show();
                    delay(500);
#endif
                    break;

#if defined(ENABLE_LED_SPI)
                case kMessageTypeLedSetRow:
                    if (firstRun == true) {
                        for (i1 = 0; i1 < 8; i1++) {
                            led_data[i1] = 0;
                            spi_led(i1 + 1, led_data[i1]);
                        }
                        
                        firstRun = false;
                    }

                    i1 = (messageGetLedRowIndex(serial_in) & 0x7); // mask this value so we don't write to an invalid address.
                                                                   // this will have to change for 100h.
                    i2 = messageGetLedRowState(serial_in);
                    
                    led_data[i1] = i2;
                    spi_led(i1 + 1, led_data[i1]);
                    break;

                case kMessageTypeLedSetColumn:
                    if (firstRun == true) {
                        for (i1 = 0; i1 < 8; i1++) {
                            led_data[i1] = 0;
                            spi_led(i1 + 1, led_data[i1]);
                        }
                        
                        firstRun = false;
                    }

                    i1 = (messageGetLedColumnIndex(serial_in) & 0x7);
                    i2 = messageGetLedColumnState(serial_in);

                    for (i3 = 0; i3 < 8; i3++) {
                        if (i2 & (1 << i3))
                            led_data[i3] |= 1 << i1;
                        else
                            led_data[i3] &= ~(1 << i1);

                        spi_led(i3 + 1, led_data[i3]);
                    }
                    break;
#elif defined(ENABLE_NEOPIXELBUS)
                case kMessageTypeLedSetRow:
                    i1 = (messageGetLedRowIndex(serial_in) & 0x7);
                    i2 = messageGetLedRowState(serial_in);

                    if (i1 < kMatrixRows)
                    {
                        for (i3 = 0; i3 < kMatrixCols; ++i3)
                        {
                            strip.SetPixelColor(kLedMatrix[i1][i3], (i2 & 0x1) ? gColorPresets[0] : kOffColor);
                            i2 >>= 1;
                        }
                    }
                    break;

                case kMessageTypeLedSetColumn:
                    i1 = (messageGetLedColumnIndex(serial_in) & 0x7);
                    i2 = messageGetLedColumnState(serial_in);

                    if (i1 < kMatrixCols)
                    {
                        for (i3 = 0; i3 < kMatrixRows; ++i3)
                        {
                            strip.SetPixelColor(kLedMatrix[i3][i1], (i2 & 0x1) ? gColorPresets[0] : kOffColor);
                            i2 >>= 1;
                        }
                    }
                    break;
#endif
                }
            }
        }

        // called even if RXF has no data
        serial_in.CheckRoll();

#if defined(ENABLE_NEOPIXELBUS)
        strip.Show();
#elif defined(ENABLE_FASTLED)
        FastLED.show();
#endif


        // output serial data **********************************************
        PORTD = 0;            // setup PORTD for output
        DDRD = 0xFF;


        // process buttons

        output_pin(ioSET,0);
        for (i1 = 0; i1 < 8; i1++) {
            output_pin(ioCLKSEL, 1);
            output_pin(ioCLKSEL, 0);
            output_pin(ioSET, 1);

            button_last[i1] = button_current[i1];

            output_pin(ioLOAD, 0);        // set 165 to load
            output_pin(ioLOAD, 1);        // 165 shift

            for (i2=0; i2 < 8; i2++) {
                i3 = input_pin(ioDATA);
                i3 = (i3 == 0);
                if (i3) 
                    button_current[i1] |= (1 << i2);
                else
                    button_current[i1] &= ~(1 << i2);

                buttonCheck(i1, i2);

                if (button_event[i1] & (1 << i2)) {
                    button_event[i1] &= ~(1 << i2);

                    messagePackButtonPress(&serial_out, (button_state[i1] & (1 << i2)) ? kButtonDownEvent : kButtonUpEvent, i2, i1);
                    
                    output_pin(ioWR, 1);
                    PORTD = serial_out.data0; 
                    output_pin(ioWR, 0);
                    
                    output_pin(ioWR, 1);
                    PORTD = serial_out.data1;
                    output_pin(ioWR, 0); 
                } 
                
                output_pin(ioCLKIN, 1);
                output_pin(ioCLKIN, 0);
            }
        }


        // process ADC

        for (i1 = 0; i1 < kAdcFilterNumAdcs; i1++) { 
            if (gAdcFilters[i1].dirty == true) {
                messagePackAdcVal(&serial_out, i1, gAdcFilters[i1].value);

                output_pin(ioWR, 1);
                PORTD = serial_out.data0; 
                output_pin(ioWR, 0);
                
                output_pin(ioWR, 1);
                PORTD = serial_out.data1;
                output_pin(ioWR, 0); 
                
                gAdcFilters[i1].dirty = false;
            }
        } 
    }
    
    return 0;
}

#if defined(ENABLE_NEOPIXELBUS)
void
// recognized pattern values are:
//   10 : single RGB colors by row
//   11 : single RGB colors by row then by col
//   12 : show preset slots (up to kLedStripPixelCount presets if it is less than 32)
RunPixelTest(PixelStrip &strip, uint8 pattern)
{
    strip.ClearTo(kOffColor);

    if (10 == pattern || 11 == pattern)
    {
        const uint8 hiVal = 64;
        for (uint8 i0 = 0; i0 < (11 == pattern ? 2 : 1); ++i0) // once by rows, once by columns
        {
            for (uint8 i2 = 0; i2 < 3; ++i2) // iterate over each of the 3 color elements
            {
                uint8 r1 = 0, g1 = 0, b1 = 0;
                switch (i2)
                {
                case 0:
                    r1 = hiVal;
                    break;
                case 1:
                    g1 = hiVal;
                    break;
                case 2:
                    b1 = hiVal;
                    break;
                }

                for (uint8 i1 = 0; i1 < kMatrixRows; ++i1)
                {
                    bool changed = false;
                    for (uint8 i3 = 0; i3 < kMatrixCols; ++i3)
                    {
                        const uint8 pixelId = (i0 == 0) ? kLedMatrix[i1][i3] : kLedMatrix[i3][i1];
                        if (kInvalidPixel != pixelId && pixelId < strip.PixelCount())
                        {
                            strip.SetPixelColor(pixelId, RgbColor{r1, g1, b1});
                            changed = true;
                        }
                    }

                    if (changed)
                    {
                        strip.Show();
                        delay(500);
                        strip.ClearTo(kOffColor);
                    }
                }
            }
        }
    }
    else if (12 == pattern)
    {
        // display the preset slots (up to kLedStripPixelCount presets if it is less than 32)
        uint8 slot = 0;
        for (uint8 i1 = 0; i1 < kMatrixRows; ++i1)
        {
            for (uint8 i2 = 0; i2 < kMatrixCols && slot < kPresetCount; ++i2)
            {
                const uint8 pixelId = kLedMatrix[i1][i2];
                if (kInvalidPixel != pixelId)
                    strip.SetPixelColor(pixelId, gColorPresets[slot++]);
            }
        }

        strip.Show();
		// leave displayed for user to clear
		return;
    }

    strip.Show();
    delay(250);
}
#endif
