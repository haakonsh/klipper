// Commands for controlling GPIO analog-to-digital input pins
//
// Copyright (C) 2016  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "basecmd.h"    // oid_alloc
#include "board/gpio.h" // struct gpio_adc
#include "board/irq.h"  // irq_disable
#include "command.h"    // DECL_COMMAND
#include "sched.h"      // DECL_TASK
#include "avr/adc_.h"

// ADC mutex control block
static mutex_t mut_cb = RELEASED;

struct analog_in
{
    struct timer timer;
    uint32_t rest_time, sample_time, next_begin_time;
    int16_t value, min_value, max_value;
    adc_channel_t adc_chn;
    uint8_t invalid_count, range_check_count;
    uint8_t state, sample_count;
};

static struct task_wake analog_wake;

static uint_fast8_t
analog_in_event(struct timer *timer)
{
    struct analog_in *a = container_of(timer, struct analog_in, timer);
    if(a->adc_chn.mut == ACQUIRED)
    {
        volatile int16_t val;
        switch(a->adc_chn.state)
        {
            case IDLE:
                // Select channel
                adc_chn_sel(&a->adc_chn);
                a->adc_chn.state = READY_TO_SAMPLE;
                // Differential inputs need 125µs of intial settling
                if(a->adc_chn.differential_inputs)
                {
                    a->timer.waketime += 2000;  // 2000/16MHz = 125µs
                }
                else
                {
                    a->timer.waketime += 200;   // 200/16MHz = 12.5µs
                }
                return SF_RESCHEDULE;

            case READY_TO_SAMPLE:
                adc_sample();
                a->adc_chn.state = READY_TO_SAMPLE_AGAIN;
                a->timer.waketime += (24 + 1) * 128 + 200 + a->sample_time;   // 25ADC CLK + 12.5µs + a->sample_time
                a->value = 0;
                return SF_RESCHEDULE;
            
            case READY_TO_SAMPLE_AGAIN:                
                // Read and store last ADC conversion
                adc_read(&val, a->adc_chn.differential_inputs);
                // output("mux:%c val:%hi", a->adc_chn.mux, val);
                a->value += val;
                // output("mux:%c value:%hi", a->adc_chn.mux, a->value);

                // Check to see if we will continue sampling or send our results to klippy host
                if(a->state++ < a->sample_count)
                {
                    adc_sample();
                    a->timer.waketime += a->sample_time;
                    return SF_RESCHEDULE;
                }

                // Check minimum and maximum values.
                if (likely(a->value >= a->min_value && a->value <= a->max_value))
                {
                    a->invalid_count = 0;
                }
                else
                {
                    a->invalid_count++;
                    if (a->invalid_count >= a->range_check_count)
                    {
                        a->invalid_count = 0;
                    }
                }
                a->state = 1;
                a->adc_chn.state = SAMPLED;
                a->next_begin_time += a->rest_time;
                a->timer.waketime = a->next_begin_time;
                sched_wake_task(&analog_wake);
                adc_release(&mut_cb, &a->adc_chn.mut);
                return SF_RESCHEDULE;
            default:
                shutdown("Not a valid ADC channel state");
        }
    }
    else // ADC Mutex not acquired
    {
        adc_acquire(&mut_cb, &a->adc_chn.mut);
        if(a->adc_chn.mut == ACQUIRED)
        {
            // Select channel
            adc_chn_sel(&a->adc_chn);
            a->timer.waketime += 200;   // 200/16MHz = 12.5µs
            return SF_RESCHEDULE;
        }
        else // ADC Mutex not released yet, try again later
        {
            a->timer.waketime += 8000;  // 8000/16MHz = 500µs
            return SF_RESCHEDULE;
        }        
    }
}

void command_config_analog_in(uint32_t *args)
{
    adc_channel_t adc_chn; 
    adc_setup(&adc_chn, args[1], args[2], args[3], args[4], args[5]);
    struct analog_in *a = oid_alloc(
        args[0], command_config_analog_in, sizeof(*a));
    a->timer.func = analog_in_event;
    a->adc_chn = adc_chn;
    a->state = 1;
}
DECL_COMMAND(command_config_analog_in, "config_analog_in oid=%c adcsra=%c adcsrb=%c admux=%c didr0=%c didr2=%c");

void command_query_analog_in(uint32_t *args)
{
    struct analog_in *a = oid_lookup(args[0], command_config_analog_in);
    sched_del_timer(&a->timer);
    adc_cancel_sample(&mut_cb, &a->adc_chn);
    a->next_begin_time = args[1];
    a->timer.waketime = a->next_begin_time;
    a->sample_time = args[2];
    a->sample_count = args[3];
    a->state = a->sample_count + 1;
    a->rest_time = args[4];
    a->min_value = args[5];
    a->max_value = args[6];
    a->range_check_count = args[7];
    output("a->adc_chn.admux:%c a->min_value:%hi a->max_value:%hi", a->adc_chn.admux, a->min_value, a->max_value);
    if (!a->sample_count)
        return;
    sched_add_timer(&a->timer);
}
DECL_COMMAND(command_query_analog_in,
             "query_analog_in oid=%c clock=%u sample_ticks=%u sample_count=%c"
             " rest_ticks=%u min_value=%hi max_value=%hi range_check_count=%c");

void analog_in_task(void)
{
    if (!sched_check_wake(&analog_wake))
        return;
    uint8_t oid;
    struct analog_in *a;
    foreach_oid(oid, a, command_config_analog_in)
    {
        if (a->adc_chn.state != SAMPLED)
            continue;
        irq_disable();
        if (a->adc_chn.state != SAMPLED)
        {
            irq_enable();
            continue;
        }
        int16_t value = a->value;
        uint32_t next_begin_time = a->next_begin_time;
        a->adc_chn.state = IDLE;
        irq_enable();
        // output("mux:%c value:%hi differential:%c",a->adc_chn.mux, a->value, a->adc_chn.differential_inputs);    
        // output("mux:%c value:%hi",a->adc_chn.mux, a->value);    
        sendf("analog_in_state oid=%c next_clock=%u value=%hi", oid, next_begin_time, value);
    }
}
DECL_TASK(analog_in_task);

void analog_in_shutdown(void)
{
    uint8_t i;
    struct analog_in *a;
    foreach_oid(i, a, command_config_analog_in)
    {
        adc_cancel_sample(&mut_cb, &a->adc_chn);
        if (a->sample_count)
        {
            a->state = a->sample_count + 1;
            a->next_begin_time += a->rest_time;
            a->timer.waketime = a->next_begin_time;
            sched_add_timer(&a->timer);
        }
    }
}
DECL_SHUTDOWN(analog_in_shutdown);
