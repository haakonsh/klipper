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

struct analog_in
{
    struct timer timer;
    uint32_t rest_time, sample_time, next_begin_time;
    int16_t value, min_value, max_value;    // TODO: Change to int16_t. Done
    struct gpio_adc adc_cfg;                // TODO: Change to adc_cfg. Done
    uint8_t invalid_count, range_check_count;
    uint8_t state, sample_count;
};

static struct task_wake analog_wake;

static uint_fast8_t
analog_in_event(struct timer *timer)
{
    struct analog_in *a = container_of(timer, struct analog_in, timer);
    // Check if ADC has been configured correctly already
    uint32_t sample_delay = gpio_adc_sample(a->adc_cfg);
    if (sample_delay)
    {
        a->timer.waketime += sample_delay;
        return SF_RESCHEDULE;
    }
    int16_t value = gpio_adc_read(a->adc_cfg);// TODO: Change value from uint16_t to int16_t
    uint8_t state = a->state;
    if (state >= a->sample_count)
    {
        state = 0;
    }
    else
    {
        value += a->value;
    }
    a->value = value;
    a->state = state + 1;
    
    // if(a->adc_cfg.mux == 0x9)output("mux:%c differential:%c value:%hi",a->adc_cfg.mux ,a->adc_cfg.differential_inputs, a->value);
    output("mux:%c differential:%c value:%hi",a->adc_cfg.mux ,a->adc_cfg.differential_inputs, a->value);

    if (a->state < a->sample_count)
    {
        a->timer.waketime += a->sample_time;
        return SF_RESCHEDULE;
    }
    a->adc_cfg.running = 0;
    a->adc_cfg.differential_settled = 0;

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
    sched_wake_task(&analog_wake);
    a->next_begin_time += a->rest_time;
    a->timer.waketime = a->next_begin_time;
    return SF_RESCHEDULE;
}

void command_config_analog_in(uint32_t *args)
{
    struct gpio_adc adc_cfg = gpio_adc_setup(args[1], args[2], args[3], args[4], args[5]);
    struct analog_in *a = oid_alloc(
        args[0], command_config_analog_in, sizeof(*a));
    a->timer.func = analog_in_event;
    a->adc_cfg = adc_cfg;
    a->state = 1;
}
DECL_COMMAND(command_config_analog_in, "config_analog_in oid=%c adcsra=%c adcsrb=%c admux=%c didr0=%c didr2=%c");

void command_query_analog_in(uint32_t *args)
{
    struct analog_in *a = oid_lookup(args[0], command_config_analog_in);
    sched_del_timer(&a->timer);
    gpio_adc_cancel_sample(a->adc_cfg);
    a->next_begin_time = args[1];
    a->timer.waketime = a->next_begin_time;
    a->sample_time = args[2];
    a->sample_count = args[3];
    a->state = a->sample_count + 1;
    a->rest_time = args[4];
    a->min_value = args[5];
    a->max_value = args[6];
    a->range_check_count = args[7];
    output("a->adc_chn.admux:%c a->min_value:%hi a->max_value:%hi", a->adc_cfg.admux, a->min_value, a->max_value);
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
        if (a->state != a->sample_count)
            continue;
        irq_disable();
        if (a->state != a->sample_count)
        {
            irq_enable();
            continue;
        }
        int16_t value = a->value; //TODO: Change to int? Done!
        // if(a->adc_cfg.mux == 0x9)output("mux: %c value:%hi[int16_t]", a->adc_cfg.mux , value);
        output("mux:%c differential:%c value:%hi",a->adc_cfg.mux ,a->adc_cfg.differential_inputs, a->value);
        // uint8_t mux = a->adc_cfg.mux;
        // if(mux == 2)
        // {
        //     static uint16_t counter = 1;
        //     static int16_t accumulator = 0;
        //     int16_t average = 0;
        //     accumulator += value;
        //     if(!(counter % 64))
        //     {
        //         average = accumulator/64;
        //         output("Average offset on ADC channel %c is %hi codes", mux, average);
        //         accumulator = 0;
        //     }
        //     counter++;        
        // }
            
        uint32_t next_begin_time = a->next_begin_time;
        a->state++;
        // if(mux == 2)
        // {
        //     //value += 10;
        //     // if(value < 0) value = 0;
        // }
        irq_enable();
        // output("mux=%c", mux);
        // output("value=%hi", value);
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
        gpio_adc_cancel_sample(a->adc_cfg);
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
