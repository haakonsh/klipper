# Extended support for AVR ADC
#
# Copyright (C) 2023  Håkon Holdhus <holdhus@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging, bisect, mcu, math
from thermistor import Thermistor

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
        self._mcu = mcu.get_printer_mcu(self.printer, config.get('mcu'))
        # Get configs. Should I iterate over all 'names'/'sections' and register callbacks and stuff here? 
        # Or should I create a new PrinterADCtoTemperatureAVR object for each 'name'?       
        self.name   = config.get_name()
        self._adcsra = config.getint('adcsra')
        self._adcsrb = config.getint('adcsrb')
        self._admux  = config.getint('admux')
        self._didr0  = config.getint('didr0')
        self._didr2  = config.getint('didr2')
        self._oversampling_factor = config.getint('oversampling_factor')
        self._report_time = config.getfloat('report_time')
        self._sample_time = SAMPLE_TIME
        self._sample_count = SAMPLE_COUNT
        self._range_check_count = RANGE_CHECK_COUNT
        self._last_value = 0
        self._single_ended = False

        self._mcu.register_config_callback(self._build_config)
        # If any error occurs during config: raise config.error("my error")

        # Setup adc channel
    def _build_config(self, config):
        # Initialize variables from config, then register commands.
        if not self._sample_count:
            return
        self._oid = self.printer.create_oid()

        #TODO: Use different min/max values for single-ended and differential inputs
        if((self._admux < (64+8)) and (self._admux > (64+29))): # Single-ended
            self._single_ended = True
            adc_sample_max = self.printer.get_constant_float("ADC_MAX_SINGLE_ENDED")
            adc_sample_min = self.printer.get_constant_float("ADC_MIN_SINGLE_ENDED")
            self.adc_accumulated_max = self._sample_count * adc_sample_max
            self.adc_accumulated_min = self._sample_count * adc_sample_min

            self.adc_accumulated_min = max(0, min(0, math.floor(self.adc_accumulated_min)))
            self.adc_accumulated_max = min(0, min(int(0xffff), math.ceil(self.adc_accumulated_max)))

        elif((self._admux > (64+7)) and (self._admux < (64+30))): # Differential
            adc_sample_max = self.printer.get_constant_float("ADC_MAX_BED")
            adc_sample_min = self.printer.get_constant_float("ADC_MIN_BED")
            self.adc_accumulated_max = self._sample_count * adc_sample_max
            self.adc_accumulated_min = self._sample_count * adc_sample_min

            self.adc_accumulated_min = max(int(-0x8000), min(0, math.floor(self.adc_accumulated_min)))
            self.adc_accumulated_max = min(0, min(int(0x7fff), math.ceil(self.adc_accumulated_max))) 

        else:
            logging.info("Error! Cannot configure MIN/MAX values. Incorrect admux value: %s", self._admux)
            mcu_adc_min = 1
            mcu_adc_max = 1
            self.adc_accumulated_max = 1
            self.adc_accumulated_min = 1
            min_sample = 1
            max_sample = 1

        self._normalization_vector = 1.0 /(self.adc_accumulated_max - self.adc_accumulated_min)

        self.printer.add_config_cmd("config_analog_in oid=%d adcsra=%s adcsrb=%s admux=%s didr0=%s didr2=%s" % (
            self._oid, self._adcsra, self._adcsrb, self._admux, self._didr0, self._didr2))
        clock = self.printer.get_query_slot(self._oid)
        sample_ticks = self.printer.seconds_to_clock(self._sample_time)        
        self._report_clock = self.printer.seconds_to_clock(self._report_time)

        self.printer.add_config_cmd(
            "query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d"
            " rest_ticks=%d min_value=%d max_value=%d range_check_count=%d" % (
                self._oid, clock, sample_ticks, self._sample_count,
                self._report_clock, self.adc_accumulated_min, self.adc_accumulated_max,
                self._range_check_count), is_init=True)
        
        self.printer.register_response(self._handle_analog_in_state,
                                    "analog_in_state", self._oid)
 
    def setup_minmax(self, min_temp, max_temp):
        self._min_temp = min_temp
        self._max_temp = max_temp      

    def get_report_time_delta(self):
        return self._report_time

    def setup_callback(self, temperature_callback):
        self.temperature_callback = temperature_callback

    def get_status(self, eventtime):        
        return {'value': self._last_value}
    
    def _handle_analog_in_state(self, params):
        # logging.info("Type of adc value passed to _handle_analog_in_state:")
        # logging.info(type(params['value']))
        # logging.info("ADC value passed to _handle_analog_in_state: ")
        # logging.info(params['value'])

        # Normalize the raw ADC value (0.0 to 1.0). Thermistor.calc_temp() requirement.
        self._last_value = self.calc_temp(self._normalization_vector * params['value'])
        # Need to convert the accumulated ADC codes to temperature[C]. What format?
        next_clock = self._mcu.clock32_to_clock64(params['next_clock'])
        last_read_clock = next_clock - self._report_clock
        last_read_time = self._mcu.clock_to_print_time(last_read_clock)
        self._last_state = (self._last_value, last_read_time)
        if self.temperature_callback is not None:
            self.temperature_callback(last_read_time, self._last_value)


class PT1000_WheatStone(PrinterADCtoTemperatureAVR):
    def __init__(self, config):
        PrinterADCtoTemperatureAVR.__init__(self, config)
        self._reference_pullup_resistor = config.getfloat('reference_pullup_resistor', 4700., above=0.)
        self._reference_resistor = config.getfloat('reference_resistor', 500., above=0.)
        self._pt1000_pullup_resistor = config.getfloat('pt1000_pullup_resistor', 4700., above=0.)
        self._vref = config.getfloat('vref', 4.98, above=0.)
        self._r0 = config.getfloat('r0', 1000., above=0.)
        self._alpha = config.getfloat('alpha', 3.85055, above=0.)
        self._reference_resistor_voltage = self._vref * (self._reference_resistor/(self._reference_resistor + self._reference_pullup_resistor))
        
    def calc_temp(self, differential_voltage): # Convert ADC value to temperature
        # Find the voltage across the PT1000 sensor, 'differential_voltage' is normalized to a unit vector of 1
        pt1000_voltage = (self._reference_resistor_voltage * self._normalization_vector) + differential_voltage
        
        # Find the resistance of the PT1000 sensor based on its measured voltage and the wheatstone parameters
        pt1000_resistance =  ((pt1000_voltage/self._vref)*self._pt1000_pullup_resistor)/(1-(pt1000_voltage/self._vref))
        
        # Find the temperature(in celcius) of the PT1000 sensor based on its temp/res parameters its calculated resistance
        temp = (pt1000_resistance - self._r0)/self._alpha
        return temp
    
    def calc_adc(self, temp): # Convert temperature to ADC value  
        # return adc

class BedTempMK3S(PrinterADCtoTemperatureAVR, Thermistor):
    def __init__(self, config):
        PrinterADCtoTemperatureAVR.__init__(self, config)

        Thermistor.__init__(self, 
                            config.getfloat('pullup_resistor', 4700., above=0.), 
                            config.getfloat('inline_resistor', 0., minval=0.))
        
        self.setup_coefficients(t1=config.getfloat('t1', 0., above=0.), 
                                r1=config.getfloat('r1', 0., above=0.),
                                t2=config.getfloat('t2', 0., above=0.),
                                r2=config.getfloat('r2', 0., above=0.),
                                t3=config.getfloat('t3', 0., above=0.),
                                r3=config.getfloat('r3', 0., above=0.),
                                name=config.get_name())

Sensors = {
    "hotend_temp": PT1000_WheatStone,
    "bed_temp": BedTempMK3S,
}

def load_config(config):
    # Register sensors
    pheaters = config.get_printer().load_object(config, "heaters")
    for name, klass in Sensors.items():
        pheaters.add_sensor_factory(name, klass)
