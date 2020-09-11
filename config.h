#ifndef __CONFIG_H__
#define __CONFIG_H__

// define ENABLE_LED_SPI to use original monome SPI LED logic
//#define ENABLE_LED_SPI // is this is defined, don't compile with arduinocore

// define ENABLE_NEOPIXELBUS to use arduino neopixel logic
#define ENABLE_NEOPIXELBUS

// define ENABLE_FASTLED to use arduino fastled -- not completed because I got NeoPixelBus to work
//#define ENABLE_FASTLED

#if defined(ENABLE_LED_SPI) && defined(ENABLE_NEOPIXELBUS)
#error "invalid configuration: do not define both ENABLE_LED_SPI and ENABLE_NEOPIXELBUS"
#endif

#if defined(ENABLE_LED_SPI) && defined(ENABLE_FASTLED)
#error "invalid configuration: do not define both ENABLE_LED_SPI and ENABLE_FASTLED"
#endif

#if defined(ENABLE_NEOPIXELBUS) && defined(ENABLE_FASTLED)
#error "invalid configuration: do not define both ENABLE_NEOPIXELBUS and ENABLE_FASTLED"
#endif

#endif // __CONFIG_H__
