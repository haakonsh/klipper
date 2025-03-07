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

// What is this constant used for?
// ADC_MAX and _MIN is used to verify that an analog sensor is within range. F.ex min and max temperatures.
// Set min/max to minimum and maximum values a single ADC sample can have: 10bit sign-extended to 16-bit = +2^10-1 to -2^10
// TODO: Create unique sets of min/max values for the heater and hotend ADC channels.
DECL_CONSTANT("ADC_MAX_DIFFERENTIAL", 511);
DECL_CONSTANT("ADC_MIN_DIFFERENTIAL", -512);
DECL_CONSTANT("ADC_MAX_SINGLE_ENDED", 1023);
DECL_CONSTANT("ADC_MIN_SINGLE_ENDED", 0);

//0b001000 - 0b011111, 0b101000 - 0b111101  Diff
//0b000000 - 0b000111, 0b100000 - 0b100111  Single
enum { ADC_DIFF_MASK=0b00011000 }; 

//TODO: Configure ADC accordingly
struct gpio_adc
gpio_adc_setup(uint8_t adcsra, uint8_t adcsrb, uint8_t admux, uint8_t didr0, uint8_t didr2)
{
    // Find pin in adc_pins table
    struct gpio_adc adc_cfg;
    memset(&adc_cfg, 0x00, sizeof(adc_cfg));


    adc_cfg.adcsra  = adcsra;
    adc_cfg.adcsrb  = adcsrb;
    adc_cfg.admux   = admux;
    adc_cfg.didr0   = didr0;
    adc_cfg.didr2   = didr2;
    adc_cfg.mux     = (adc_cfg.admux & 0b11111) | ((adc_cfg.adcsrb & 0b1000) << 2);
    // output("adc_cfg.mux(%c) & ADC_DIFF_MASK(%c) = %c", adc_cfg.mux, ADC_DIFF_MASK, (adc_cfg.mux & ADC_DIFF_MASK));
    if(adc_cfg.mux & ADC_DIFF_MASK)
    {
        adc_cfg.differential_inputs = 1;
    }
    else
    {
        adc_cfg.differential_inputs = 0;
    }
    adc_cfg.differential_settled = 0;
    adc_cfg.running = 0;

    output(".adcsra:%c, .adcsrb:%c, .admux:%c, .didr0:%c, .didr2:%c, .mux:%c, .differential_input:%c",   \
            adc_cfg.adcsra, adc_cfg.adcsrb, adc_cfg.admux, adc_cfg.didr0, adc_cfg.didr2,        \
            adc_cfg.mux, adc_cfg.differential_inputs);
            
    if ((adc_cfg.mux & 0b00011110) == 0b00011110) // Vbg, Vgnd, or one of the two reserved channels.
        shutdown("Not a valid ADC input channel");

    // Enable ADC
    ADCSRA = adc_cfg.adcsra;

    // Disable digital input buffers for this channel
    DIDR0 |= adc_cfg.didr0;
    DIDR2 |= adc_cfg.didr2;

    return adc_cfg;
}

enum { ADC_DUMMY=0xff };


static uint8_t last_analog_read = ADC_DUMMY;
static uint8_t settling = 0;


// Try to sample a value. Returns zero if sample ready, otherwise
// returns the number of clock ticks the caller should wait before
// retrying this function.
uint32_t
gpio_adc_sample(struct gpio_adc g)
{
    if (ADCSRA & (1<<ADSC))
        // Busy
        goto need_delay;
    if (last_analog_read == g.mux)
        // Sample now ready
        return 0;
    if (last_analog_read != ADC_DUMMY)
        // Sample on another channel in progress
        goto need_delay;
    if (settling && !g.differential_settled)
        // ADC is settling before a diff input channel can begin to sample
        goto need_delay;

    if(!g.running)
    {
        // Set the channel to sample
        ADCSRB = g.adcsrb;
        ADMUX = g.admux;
    }
    //output("ADMUX=%c ADCSRA=%c ADCSRB=%c DIDR0=%c DIDR1=%c DIDR2=%c", ADMUX, ADCSRA, ADCSRB, DIDR0, DIDR1, DIDR2);    

    // Differential channels need 125µs before the first conversion can be triggered.
    if(g.differential_inputs && (!g.differential_settled))
    {
        settling = 1;
        g.differential_settled = 1;
        return 2000;    // 2000/16MHz = 125µs
    } 
    settling = 0;
    last_analog_read = g.mux;

    // Start the sample
    ADCSRA |= (1 << ADSC);
    
    // Schedule next attempt after sample is likely to be complete
    if(!g.running)
    {
        g.running = 1;
        return (25 + 1) * 128 + 200;    // First ADC conversion takes 25 ADC cycles = 208µs + 12.5µs  
    }
    // Schedule next attempt after sample is likely to be complete
need_delay:
    // ADC CLK = SYS CLK / ADC Prescaler = 16MHz/128 = 125kHz
    return (13 + 1) * 128 + 200; // 14 ADC CLK * 128 ADC Prescaler + 200 SYS CLK = 1992 SYS CLK / 16MHz = 124.5µs
}

//TODO: Check the sign bit (10th) and pad bits accordingly. Done!
//TODO: Change return type from uint16_t to int16_t. Done!
// Read a value; use only after gpio_adc_sample() returns zero
int16_t
gpio_adc_read(struct gpio_adc g)
{
    last_analog_read = ADC_DUMMY;

    if(!g.differential_inputs)
    {
        return ADC;
    }
    else if(ADC & 0x200)   // Check the sign bit (10th)
    {        
        return ADC | 0xFC00; // Pad 1's (two's complement)
    }
    else
    {
        return ADC;
    }
}

// Cancel a sample that may have been started with gpio_adc_sample()
void
gpio_adc_cancel_sample(struct gpio_adc g)
{
    if (last_analog_read == g.mux)
        last_analog_read = ADC_DUMMY;
    g.running = 0;
    g.differential_settled = 0;
}
