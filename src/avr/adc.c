// Analog to Digital Converter support
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_MACH_atmega644p
#include "command.h" // shutdown
#include "gpio.h" // gpio_adc_read
#include "internal.h" // GPIO
#include "pgm.h" // PROGMEM
#include "sched.h" // sched_shutdown
#include <string.h>

static const uint8_t adc_pins[] PROGMEM = {
#if CONFIG_MACH_atmega168 || CONFIG_MACH_atmega328 || CONFIG_MACH_atmega328p
    GPIO('C', 0), GPIO('C', 1), GPIO('C', 2), GPIO('C', 3),
    GPIO('C', 4), GPIO('C', 5), GPIO('E', 2), GPIO('E', 3),
#elif CONFIG_MACH_atmega644p || CONFIG_MACH_atmega1284p
    GPIO('A', 0), GPIO('A', 1), GPIO('A', 2), GPIO('A', 3),
    GPIO('A', 4), GPIO('A', 5), GPIO('A', 6), GPIO('A', 7),
#elif CONFIG_MACH_at90usb1286 || CONFIG_MACH_at90usb646
    GPIO('F', 0), GPIO('F', 1), GPIO('F', 2), GPIO('F', 3),
    GPIO('F', 4), GPIO('F', 5), GPIO('F', 6), GPIO('F', 7),
#elif CONFIG_MACH_atmega32u4
    GPIO('F', 0), GPIO('F', 1), GPIO('F', 2), GPIO('F', 3),
    GPIO('F', 4), GPIO('F', 5), GPIO('F', 6), GPIO('F', 7),
    GPIO('D', 4), GPIO('D', 6), GPIO('D', 7), GPIO('B', 4),
#elif CONFIG_MACH_atmega1280 || CONFIG_MACH_atmega2560
    GPIO('F', 0), GPIO('F', 1), GPIO('F', 2), GPIO('F', 3),
    GPIO('F', 4), GPIO('F', 5), GPIO('F', 6), GPIO('F', 7),
    GPIO('K', 0), GPIO('K', 1), GPIO('K', 2), GPIO('K', 3),
    GPIO('K', 4), GPIO('K', 5), GPIO('K', 6), GPIO('K', 7),
#endif
};

// The atmega168/328 have two analog only pins
#if CONFIG_MACH_atmega168 || CONFIG_MACH_atmega328
DECL_ENUMERATION_RANGE("pin", "PE2", GPIO('E', 2), 2);
#endif

enum { ADMUX_DEFAULT = 0x40 };
enum { ADC_ENABLE = (1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2)|(1<<ADEN)|(1<<ADIF) };

DECL_CONSTANT("ADC_MAX", 1023);  //TODO: What is this constant used for?

//TODO: Configure ADC accordingly
struct gpio_adc
gpio_adc_setup(uint8_t admux)
{
    // Find pin in adc_pins table
    struct gpio_adc adc_cfg;
    memset(&adc_cfg, 0x00, sizeof(adc_cfg));

    adc_cfg.admux = admux;

    if (adc_cfg.admux >= 0b111110)
        shutdown("Not a valid ADC input channel");

    // Enable ADC
    adc_cfg.adcsra = ADC_ENABLE;
    ADCSRA = adc_cfg.adcsra;

    // Disable digital input for this pin
    if (adc_cfg.admux >= 0b100000)
        adc_cfg.didr2 = 1 << (adc_cfg.admux & 0x07);
    else if(adc_cfg.admux & (0b001001 | 0b001000))
        adc_cfg.didr0 = (1 << 0) | (1 << 1);
    else
        adc_cfg.didr0 = 1 << (adc_cfg.admux & 0x07);

    DIDR0 |= adc_cfg.didr0;
    DIDR2 |= adc_cfg.didr2;
    
    return adc_cfg;
}

enum { ADC_DUMMY=0xff };
static uint8_t last_analog_read = ADC_DUMMY;

//TODO: Configure ADC accordingly. Done!
// Try to sample a value. Returns zero if sample ready, otherwise
// returns the number of clock ticks the caller should wait before
// retrying this function.
uint32_t
gpio_adc_sample(struct gpio_adc g)
{
    if (ADCSRA & (1<<ADSC))
        // Busy
        goto need_delay;
    if (last_analog_read == g.admux)
        // Sample now ready
        return 0;
    if (last_analog_read != ADC_DUMMY)
        // Sample on another channel in progress
        goto need_delay;
    last_analog_read = g.admux;

    // Set the channel to sample
    ADCSRB = g.adcsrb;
    //ADMUX = ADMUX_DEFAULT | (g.admux & 0x07);
    ADMUX = g.admux;

    // Start the sample
    ADCSRA = ADC_ENABLE | (1<<ADSC);

    // Schedule next attempt after sample is likely to be complete
need_delay:
    return (13 + 1) * 128 + 200;
}

//TODO: Check the sign bit (10th) and pad bits accordingly. Done!
//TODO: Change return type from uint16_t to int16_t. Done!
// Read a value; use only after gpio_adc_sample() returns zero
int16_t
gpio_adc_read(struct gpio_adc g)
{
    last_analog_read = ADC_DUMMY;
    if(ADC & 0x200)   // Check the sign bit (10th)
    {        
        return ADC | 0xFC00; // Pad 1's (two's complement)
    }

    return ADC;                     
}

// Cancel a sample that may have been started with gpio_adc_sample()
void
gpio_adc_cancel_sample(struct gpio_adc g)
{
    if (last_analog_read == g.admux)
        last_analog_read = ADC_DUMMY;
}
