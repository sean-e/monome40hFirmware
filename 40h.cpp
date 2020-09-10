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


struct SerialInputData
{
    // for compatibility with t_message unpacking macros
    uint8 data0 = 0;
    uint8 data1 = 0;
    uint8 readLen = 0;
    uint8 rx_roll = 0;

    void Read(uint8 b)
    {
        switch (readLen)
        {
        case 0:
            data0 = b;
            break;
        case 1:
            data1 = b;
            break;
        }
        ++readLen;
    }

    bool PacketReady() 
    {
        if (!readLen)
            return false;

        const bool ret = readLen == 2;
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

    sei(); // for ADC

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
#endif
                    break;

                case kMessageTypeLedIntensity:
#if defined(ENABLE_LED_SPI)
                    msg_data0 = messageGetLedIntensity(serial_in);
                    spi_led(10, msg_data0);
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
#endif
                    break;

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
#endif
                }
            }
        }

        // called even if RXF has no data
        serial_in.CheckRoll();


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
