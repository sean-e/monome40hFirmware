/* 
 *  Copyright (C) 2006, Joe Lake, monome.org
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
 *  $Id: adc.c,v. 1.1.1.1 2006/05/02 1:01:22
 */

/*
Altered source version
	Modified by: Sean Echevarria (http://www.creepingfog.com/sean/) 2007.11.03
	changed implementation of adcAddNextValue
*/


#include <avr/io.h>
#include <avr/interrupt.h>
#include "config.h"
#include "adc.h"


t_adc_filter gAdcFilters[kAdcFilterNumAdcs];
static uint8 adcSelect = 0;
static uint8 gAdcEnableState = 0;


/***************************************************************************************************
 *
 * DESCRIPTION: Initialize filtering software and adc/timer0 hardware.  JAL
 *
 * ARGUMENTS:   
 *
 * RETURNS:
 *
 * NOTES:       
 *
 ****************************************************************************************************/

void initAdcs(void)
{
    initAdcFilters();
    hardInitAdcs();
}


/***************************************************************************************************
 *
 * DESCRIPTION: Stop adc conversions and turn off adcs.
 *
 * ARGUMENTS:   
 *
 * RETURNS:
 *
 * NOTES:       
 *
 ****************************************************************************************************/

void disableAdcs(void)
{
#if !defined(ENABLE_NEOPIXELBUS) && !defined(ENABLE_FASTLED)
    TIMSK &= ~(1 << TOIE0);  // disable timer0 interrupts
    TCCR0 = 0x00;            // disable timer0
#endif

    ADCSRA &= ~(1 << ADATE); // disable conversion auto trigger.
    ADCSRA &= ~(1 << ADEN);  // disable adcs
    ADCSRA &= ~(1 << ADIE);  // disable adc interrupts
}



/***************************************************************************************************
 *
 * DESCRIPTION: Initialize adc/timer0 hardware.  JAL
 *
 * ARGUMENTS:   
 *
 * RETURNS:
 *
 * NOTES:       This function sets up timer0 (automatic reset to 0 on overflow) to run in normal mode at sysclk/64.
 *              Adc conversions are triggered automatically by timer0 overflow.  One thing that is fucked is that
 *              for this to work timer0 interrupts have to be enabled.  I don't think this should be the case
 *              because we don't care about servicing overflow interrupts, we only service the adc conversion finished
 *              interrupts.  As a result I've added "EMPTY_INTERRUPT(SIG_OVERFLOW0)" to this file so we don't hit 
 *              the reset vector when timer0 overflows.  I also think the call to timer_enable_int may disable other timer
 *              interrupts if they're enabled, but I'm not sure.
 *
 ****************************************************************************************************/

void hardInitAdcs(void)
{
    SFIOR |= 0x80;                // set conversion auto trigger source to timer0 overflow

    ADCSRA |= 0x7;          // set adc clock prescaler bits to sysclk/128 to get a conversion clock of 125KHz
                            // (must be between 50Khz and 200Hz).
    ADCSRA |= (1 << ADATE); // enable conversion auto trigger.
    ADCSRA |= (1 << ADEN);  // enable adcs
    ADCSRA |= (1 << ADIE);  // enable adc interrupts

#if !defined(ENABLE_NEOPIXELBUS) && !defined(ENABLE_FASTLED) // prescale is handled in init()
    TCCR0 = 0x04; // enable timer in normal mode.  prescale = /64
    TIMSK |= (1 << TOIE0);  // disable timer0 interrupts
    //TIMSK = (1 << OCIE0);
    //timer_enable_int(_BV(TOIE0));
#endif
}


/***************************************************************************************************
 *
 * DESCRIPTION: Zeroes filter structures.  JAL
 *
 * ARGUMENTS:
 *
 * RETURNS:
 *
 * NOTES:       
 *
 ****************************************************************************************************/

void initAdcFilters(void)
{
    for (uint8 adc = 0; adc < kAdcFilterNumAdcs; adc++) {
        gAdcFilters[adc].accum = 0;
        gAdcFilters[adc].value = 0;
        gAdcFilters[adc].index = 0;
        gAdcFilters[adc].dirty = 0;
        gAdcFilters[adc].last_value = 0;

        for (uint8 bucket = 0; bucket < kAdcFilterNumBuckets; bucket++)
            gAdcFilters[adc].bucket[bucket] = 0;
    }
}


/***************************************************************************************************
 *
 * DESCRIPTION: Adds a new frame to the adc accumulator and generate a new value.  JAL
 *
 * ARGUMENTS:   adc -   number of adc filter to be updated (0 = PORTA^0, 3 = PORTA^3).
 *              value - 10 bit conversion value (0 = GND, 1023 = AREF.   
 *
 * RETURNS:
 *
 * NOTES:       
 *
 ****************************************************************************************************/
       
void adcAddNextValue(uint8 adc, uint16 value)
{
    if (!(gAdcEnableState & (1 << adc)))
        return;

    t_adc_filter *curAdcFilter = &gAdcFilters[adc];
    curAdcFilter->accum -= curAdcFilter->bucket[curAdcFilter->index];
    curAdcFilter->accum += value;
    curAdcFilter->bucket[curAdcFilter->index] = value;

    curAdcFilter->last_value = curAdcFilter->value;
    curAdcFilter->value = curAdcFilter->accum >> kAdcFilterRightShiftValue; // accum divided by kAdcFilterNumBuckets
    if (curAdcFilter->value != curAdcFilter->last_value)
        curAdcFilter->dirty = true;

    if (++curAdcFilter->index >= kAdcFilterNumBuckets) 
        curAdcFilter->index = 0;
}


/***************************************************************************************************
 *
 * DESCRIPTION: Enables an adc and if all adcs were previously off performs an adc 
 *              hardware init.
 *
 * ARGUMENTS:   adc -   number of adc filter enabled (0 = PORTA^0, 3 = PORTA^3).
 *
 * RETURNS:
 *
 * NOTES:       
 *
 ****************************************************************************************************/

void enableAdc(uint8 adc)
{
    if (adc >= kAdcFilterNumAdcs)
        return;

    if ((gAdcEnableState & 0x0F) == 0)
        initAdcs();

    gAdcEnableState |= (1 << adc);
}


/***************************************************************************************************
 *
 * DESCRIPTION: Disables an adc and if all adcs are off stops adc conversions.
 *
 * ARGUMENTS:   adc -   number of adc filter enabled (0 = PORTA^0, 3 = PORTA^3).
 *
 * RETURNS:
 *
 * NOTES:       
 *
 ****************************************************************************************************/

void disableAdc(uint8 adc)
{
    if (adc >= kAdcFilterNumAdcs)
        return;

    gAdcEnableState &= ~(1 << adc);

    if ((gAdcEnableState & 0x0F) == 0)
        disableAdcs();
}

#if !defined(ENABLE_NEOPIXELBUS) && !defined(ENABLE_FASTLED)
EMPTY_INTERRUPT(TIMER0_OVF_vect);
#endif

ISR(ADC_vect)
{
    adcAddNextValue(adcSelect, (uint16)ADCL | (((uint16)ADCH) << 8));

    if (++adcSelect >= kAdcFilterNumAdcs) 
        adcSelect = 0;

    ADMUX &= 0xF0;
    ADMUX |= adcSelect & 0xF;
}
