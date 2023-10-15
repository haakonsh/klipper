# Extended support for AVR ADC
#
# Copyright (C) 2023  Håkon Holdhus <holdhus@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging, bisect

######################################################################
# Interface between MCU adc and heater temperature callbacks
######################################################################

SAMPLE_TIME = 0.001
SAMPLE_COUNT = 8
REPORT_TIME = 0.300
RANGE_CHECK_COUNT = 4

# Interface between ADC and heater temperature callbacks
class PrinterADCtoTemperatureAVR:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.mcu = self.printer.lookup_object('mcu')
        # Get configs. Should I iterate over all 'names'/'sections' and register callbacks and stuff here? 
        # Or should I create a new PrinterADCtoTemperatureAVR object for each 'name'?       
        self.name   = config.get_name()
        self._adcsra = config.getint('adcsra')
        self._adcsrb = config.getint('adcsrb')
        self._admux  = config.getint('admux')
        self._didr0  = config.getint('didr0')
        self._didr2  = config.getint('didr2')
        self.oversampling_factor = config.getint('oversampling_factor')

        self.last_value = 0

        self.mcu.register_config_callback(self._build_config)

        # If any error occurs during config: raise config.error("my error")

        # Setup adc channel
    def create(self):
        # Initialize variables from config, then register commands?
        if not self._sample_count:
            return
        self._oid = self.printer.create_oid()
        self.printer.add_config_cmd("config_analog_in oid=%d adcsra=%s adcsrb=%s admux=%s didr0=%s didr2=%s" % (
            self._oid, self._adcsra, self._adcsrb, self._admux, self._didr0, self._didr2))
        clock = self.printer.get_query_slot(self._oid)
        sample_ticks = self.printer.seconds_to_clock(self._sample_time)

        #TODO: Use different min/max values for single-ended and differential inputs
        if((self._admux < (64+8)) and (self._admux > (64+29))): # Single-ended
            mcu_adc_max = self.printer.get_constant_float("ADC_MAX_SINGLE_ENDED")
            mcu_adc_min = self.printer.get_constant_float("ADC_MIN_SINGLE_ENDED")
            max_adc = self._sample_count * mcu_adc_max
            min_adc = self._sample_count * mcu_adc_min

            min_sample = max(0, min(0, math.floor(self._min_sample * min_adc))) #TODO: Set min adc vals
            max_sample = min(0, min(int(0xffff), math.ceil(self._max_sample * max_adc)))            

        elif((self._admux > (64+7)) and (self._admux < (64+30))): # Differential
            mcu_adc_max = self.printer.get_constant_float("ADC_MAX_BED")
            mcu_adc_min = self.printer.get_constant_float("ADC_MIN_BED")
            max_adc = self._sample_count * mcu_adc_max
            min_adc = self._sample_count * mcu_adc_min

            min_sample = max(int(-0x8000), min(0, math.floor(self._min_sample * min_adc))) #TODO: Set min adc vals
            max_sample = min(0, min(int(0x7fff), math.ceil(self._max_sample * max_adc)))            

        else:
            logging.info("Error! Cannot configure MIN/MAX values. Incorrect admux value: %s", self._admux)
            mcu_adc_min = 1
            mcu_adc_max = 1
            max_adc = 1
            min_adc = 1
            min_sample = 1
            max_sample = 1

        self._inv_max_adc = 1.0 / max_adc
        self._inv_min_adc = 1.0 / min_adc
        self._report_clock = self.printer.seconds_to_clock(self._report_time)

        self.printer.add_config_cmd(
            "query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d"
            " rest_ticks=%d min_value=%d max_value=%d range_check_count=%d" % (
                self._oid, clock, sample_ticks, self._sample_count,
                self._report_clock, min_sample, max_sample,
                self._range_check_count), is_init=True)
        
        self.printer.register_response(self._handle_analog_in_state,
                                    "analog_in_state", self._oid)
 
    def setup_minmax(self, sample_time, sample_count,
                     minval=-1., maxval=1., range_check_count=0):
        self._sample_time = sample_time
        self._sample_count = sample_count
        self._min_sample = minval
        self._max_sample = maxval
        self._range_check_count = range_check_count
        
    def setup_adc_callback(self, report_time, callback):
        self._report_time = report_time
        self._callback = callback

    def get_status(self, eventtime):
        
        return {'value': self.last_value}
    
    def _sample_adc(self):
         

def load_config_prefix(config):
    #return PrinterADCtoTemperatureAVR(config)
    if config.get("sensor_type", None) is 'my_sensor':
        custom_sensor = PrinterADCtoTemperatureAVR(config)
    else:
        raise config.error("Config error: 'sensor_type' not equal to 'my_sensor'")
    print(custom_sensor.name)
    
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory(custom_sensor.name, custom_sensor.create)
