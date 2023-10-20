#include "autoconf.h" // CONFIG_MACH_atmega644p
#include "command.h" // shutdown
#include "gpio.h" // gpio_adc_read
#include "internal.h" // GPIO
#include "pgm.h" // PROGMEM
#include "sched.h" // sched_shutdown
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

// ADC_MAX and _MIN is used to verify that an analog sensor is within range. F.ex min and max temperatures.
// Set min/max to minimum and maximum values a single ADC sample can have: 10bit sign-extended to 16-bit = +2^10-1 to -2^10
DECL_CONSTANT("ADC_MAX_DIFFERENTIAL", 511);
DECL_CONSTANT("ADC_MIN_DIFFERENTIAL", -512);
DECL_CONSTANT("ADC_MAX_SINGLE_ENDED", 1023);
DECL_CONSTANT("ADC_MIN_SINGLE_ENDED", 0);

//0b001000 - 0b011111, 0b101000 - 0b111101  Diff
//0b000000 - 0b000111, 0b100000 - 0b100111  Single
enum { ADC_DIFF_MASK=0b00011000 }; 
enum { ADMUX_DEFAULT    = 0x40 };
enum { ADC_ENABLE       = (1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2)|(1<<ADEN)|(1<<ADIF) };

typedef enum {
    ERROR,
    IDLE,
    READY_TO_SAMPLE,
    READY_TO_SAMPLE_AGAIN,
    SAMPLED
}adc_chn_state_t;

typedef enum {
    ACQUIRED,
    RELEASED
}mutex_t;

typedef struct{
    uint8_t     adcsra;
    uint8_t     adcsrb;
    uint8_t     admux;
    uint8_t     didr0;
    uint8_t     didr2;
    uint8_t     mux;
    bool        differential_inputs;
    mutex_t     mut;
    adc_chn_state_t state;
}adc_channel_t;

static inline void adc_setup (adc_channel_t *adc_chn, uint8_t adcsra, uint8_t adcsrb, uint8_t admux, uint8_t didr0, uint8_t didr2) __attribute__((always_inline));
static inline void adc_sample_rdy(bool *rdy) __attribute__((always_inline));
static inline void adc_chn_sel(adc_channel_t *adc_chn) __attribute__((always_inline));
static inline void adc_sample(void) __attribute__((always_inline));
static inline void adc_acquire(mutex_t *mut_cb, mutex_t *mut_chn) __attribute__((always_inline));
static inline void adc_release(mutex_t *mut_cb, mutex_t *mut_chn) __attribute__((always_inline));
static inline void adc_read(volatile int16_t *value, bool differential_inputs) __attribute__((always_inline));
static inline void adc_cancel_sample(mutex_t *mut_cb, adc_channel_t *chn) __attribute__((always_inline));

static inline void adc_setup(adc_channel_t *adc_chn, uint8_t adcsra, uint8_t adcsrb, uint8_t admux, uint8_t didr0, uint8_t didr2)
{
    memset(adc_chn, 0x00, sizeof(adc_channel_t));
    adc_chn->adcsra  = adcsra;
    adc_chn->adcsrb  = adcsrb;
    adc_chn->admux   = admux;
    adc_chn->didr0   = didr0;
    adc_chn->didr2   = didr2;
    adc_chn->mux     = (adc_chn->admux & 0b11111) | ((adc_chn->adcsrb & 0b1000) << 2);
    // output("adc_chn.mux(%c) & ADC_DIFF_MASK(%c) = %c", adc_chn.mux, ADC_DIFF_MASK, (adc_chn.mux & ADC_DIFF_MASK));
    if(adc_chn->mux & ADC_DIFF_MASK) adc_chn->differential_inputs = 1; 
    else adc_chn->differential_inputs = 0;
            
    if ((adc_chn->mux & 0b00011110) == 0b00011110) // Vbg, Vgnd, or one of the two reserved channels.
        shutdown("Not a valid ADC input channel");

    // Initial state and mutex
    adc_chn->state = IDLE;
    adc_chn->mut = RELEASED;
    // Enable ADC
    ADCSRA = adc_chn->adcsra;
    // Disable digital input buffers for this channel
    DIDR0 |= adc_chn->didr0;
    DIDR2 |= adc_chn->didr2;

    return;
}

static inline void adc_sample_rdy(bool *rdy)
{
    *rdy=(ADCSRA & (1 << ADIF));
    return;
} 

static inline void adc_chn_sel(adc_channel_t *adc_chn)
{
    // Set the channel to sample
    ADCSRB = adc_chn->adcsrb;
    ADMUX = adc_chn->admux;
    // output("Channel selected! ADCSRB:%c ADMUX:%c", ADCSRB, ADMUX);

}

static inline void adc_sample(void)
{
    // Start the sample
    // output("Sampling triggered!");
    ADCSRA |= (1 << ADSC);
    // output("Did we really start a conversion? ADCSRA:%c ", ADCSRA);
    return;
}

static inline void adc_acquire(mutex_t *mut_cb, mutex_t *mut_chn)
{
    irq_disable();
    if(*mut_cb == RELEASED)
    {
        *mut_cb = ACQUIRED;
        *mut_chn = ACQUIRED;
        irq_enable();
        // output("ADC acquired!");
        return;
    }
    irq_enable();
}

static inline void adc_release(mutex_t *mut_cb, mutex_t *mut_chn)
{
    irq_disable();
    if(*mut_chn == ACQUIRED)
    {
        *mut_cb = RELEASED;
        *mut_chn = RELEASED;
    }
    irq_enable();
    // output("ADC released!");
}

// Read the ADC conversion results and handle sign bit
static inline void adc_read(volatile int16_t *value, bool differential_inputs)
{
    // output("Did we really finish a conversion? ADSC:%c ADIF:%c", (ADCSRA & (1 << ADSC)), (ADCSRA & (1 << ADIF)));
    // Clear ADC conversion complete flag
    ADCSRA |= (1 << ADIF);
    if(!differential_inputs)
    {
        *value = ADC;
        return;
    }
    else if(ADC & (1 << 9))  // Check the sign bit (10th)
    {        
        *value =  ADC | 0xFC00; // Pad 1's (two's complement)
        return;
    }
    *value = ADC;
    return;
}

// Cancel a sample that may have been started with gpio_adc_sample()
static inline void adc_cancel_sample(mutex_t *mut_cb, adc_channel_t *chn)
{
    // ADCSRA &= ~(1 << ADEN);
    ADCSRA |= (1 << ADIF);
    adc_release(mut_cb, &chn->mut);
    chn->state = IDLE;
}