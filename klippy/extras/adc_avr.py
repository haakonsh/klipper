# Extended support for AVR ADC
#
# Copyright (C) 2023  Håkon Holdhus <holdhus@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging, bisect, mcu, math, msgproto, serialhdl
from .thermistor import Thermistor
import numpy as np

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
        self._mcu = mcu.get_printer_mcu(self.printer, 'mcu')
        self.name   = config.get_name()
        # Read and store ADC channel configuration
        self._adcsra = config.getint('adcsra')
        self._adcsrb = config.getint('adcsrb')
        self._admux  = config.getint('admux')
        self._didr0  = config.getint('didr0')
        self._didr2  = config.getint('didr2')
        self._report_time = config.getfloat('report_time')
        self._sample_time = SAMPLE_TIME
        self._sample_count = SAMPLE_COUNT
        self._range_check_count = RANGE_CHECK_COUNT
        self._last_value = 0
        self._single_ended = False
        # Let the MCU object know it needs to call _build_config 
        self._mcu.register_config_callback(self._build_config)

        # If any error occurs during config: raise config.error("my error")
        self._config_error = config.error

        # Setup adc channel
    def _build_config(self):
        # Initialize variables from config, then register commands.
        if not self._sample_count:
            return
        self._oid = self._mcu.create_oid()

        mcu_constants = self._mcu._get_status_info['mcu_constants']
        print(mcu_constants['ADC_MAX_SINGLE_ENDED'])

        # Use different min/max values for single-ended and differential inputs
        if((self._admux < (64+8)) or (self._admux > (64+29))): # Single-ended
            self._single_ended = True
            adc_sample_max = self._mcu._serial.get_msgparser().get_constant_int("ADC_MAX_SINGLE_ENDED")
            adc_sample_min = self._mcu._serial.get_msgparser().get_constant_int("ADC_MIN_SINGLE_ENDED")
            self.adc_accumulated_max = self._sample_count * adc_sample_max
            self.adc_accumulated_min = self._sample_count * adc_sample_min

            self.adc_accumulated_min = max(0, math.floor(self.adc_accumulated_min))
            self.adc_accumulated_max = min(int(0xffff), math.ceil(self.adc_accumulated_max))

        elif((self._admux > (64+7)) and (self._admux < (64+30))): # Differential
            adc_sample_max = self._mcu._serial.get_msgparser().get_constant_int("ADC_MAX_DIFFERENTIAL")
            adc_sample_min = self._mcu._serial.get_msgparser().get_constant_int("ADC_MIN_DIFFERENTIAL")
            self.adc_accumulated_max = self._sample_count * adc_sample_max
            self.adc_accumulated_min = self._sample_count * adc_sample_min

            self.adc_accumulated_min = max(int(-0x8000), min(0.0001, math.floor(self.adc_accumulated_min)))
            self.adc_accumulated_max = min(int(0x7fff), math.ceil(self.adc_accumulated_max))

        else:
            raise self._config_error('Error! Cannot configure MIN/MAX values. Incorrect admux value: %s' % self._admux)
            mcu_adc_min = 1
            mcu_adc_max = 1
            self.adc_accumulated_max = 1
            self.adc_accumulated_min = 1
            min_sample = 1
            max_sample = 1

        self._normalization_vector = 1.0 /(self.adc_accumulated_max - self.adc_accumulated_min)       

        self._mcu.add_config_cmd("config_analog_in oid=%d adcsra=%d adcsrb=%d admux=%d didr0=%d didr2=%d" % (
            self._oid, self._adcsra, self._adcsrb, self._admux, self._didr0, self._didr2))     

        clock = self._mcu.get_query_slot(self._oid)
        sample_ticks = self._mcu.seconds_to_clock(self._sample_time)        
        self._report_clock = self._mcu.seconds_to_clock(self._report_time)

        self.adc_min = min(self.calc_adc(self._min_temp), self.calc_adc(self._max_temp))
        self.adc_max = max(self.calc_adc(self._min_temp), self.calc_adc(self._max_temp))
        self._mcu.add_config_cmd(
            "query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d"
            " rest_ticks=%d min_value=%d max_value=%d range_check_count=%d" % (
                self._oid, clock, sample_ticks, self._sample_count,
                self._report_clock, self.adc_min, self.adc_max,
                self._range_check_count), is_init=True)
        
        self._mcu.register_response(self._handle_analog_in_state,
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
        self._last_value = self.calc_temp(params['value'])
        # print("calc_temp from '%s': %.3f" % (self.name, self._last_value))
        # if(self.name == 'extruder'):
            # print("ADC codes:%.3f calculated temp:%.3f from [%-11s]" % (params['value'], self._last_value, self.name))
            
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
        self._r0 = config.getfloat('r0', 1000., above=0.)
        self._vref = config.getfloat('vref', above=0.)
        self._reference_resistor_voltage = self._vref * (self._reference_resistor/(self._reference_resistor + self._reference_pullup_resistor))
        self._lin_temp_coeff = config.getfloat('linear_temperature_coefficient', 0.0039083, above=0.)
        self._quad_temp_coeff = config.getfloat('quadratic_temperature_coefficient', -0.0000005775, below=0.)
        self._adc_gain_coefficient_0 = config.getfloat('adc_gain_coefficient_0')
        self._adc_gain_coefficient_1 = config.getfloat('adc_gain_coefficient_1')
        
        self.fifo = np.full(10, -3200, dtype=np.int16)
        
    def calc_temp(self, raw_adc): # Convert ADC value to temperature
        
        # self.fifo = np.roll(self.fifo, 1)
        # np.put(self.fifo, [0], raw_adc)
        # self.adc_avg = np.average(self.fifo)#, weights=np.arange(self.fifo.size, 0, -1))       
        self.adc_avg = raw_adc       
        adc_avg_div8 = (self.adc_avg/self._sample_count) 

        # adc_normalized = self._normalization_vector * raw_adc
        adc_voltage = adc_avg_div8*(self._vref/(512*10))

        # Find the voltage across the PT1000 sensor, 'adc_normalized' is normalized to a unit vector of 1
        pt1000_voltage = self._reference_resistor_voltage + adc_voltage
        
        # Find the resistance of the PT1000 sensor based on its measured voltage and the wheatstone parameters
        pt1000_resistance =  ((pt1000_voltage/self._vref)*self._pt1000_pullup_resistor)/(1-(pt1000_voltage/self._vref))

        # Correct for ADC gain error. Linear estimation was used to find a set of coeficcients that would scale the estimated
        # resistance of the PT1000 to what my multimeter would measure during a sweep of hotend temp from 40C - 250C at 10C intervals.
        pt1000_resistance_gain_corrected =  self._adc_gain_coefficient_1*pt1000_resistance + self._adc_gain_coefficient_0
                
        # Find the temperature(in celcius) of the PT1000 sensor based on its temp/res parameters    
        # Quadratic solution for t(r), given by r(t) = r(0)*(1 + lin_temp_coeff*t + quad_temp_coeff*t^2)
        temp = (-self._lin_temp_coeff+(math.sqrt(self._lin_temp_coeff**2 - 4*self._quad_temp_coeff*(1.0-(pt1000_resistance_gain_corrected/self._r0))))) / (2*self._quad_temp_coeff)
       
        # print(  f'********************************* PT1000 data *******************************************\n'
        #         f'Codes: \t\t\t\t{adc_avg_div8: 7.1f}\n'
        #         f'ADC Voltage: \t\t\t{adc_voltage: 7.1f} mV\n'
        #         f'PT1000 voltage: \t\t{pt1000_voltage: 7.1f} mV\n'
        #         f'PT1000 Resistance: \t\t{pt1000_resistance: 7.1f} Ohm\n'
        #         f'Temp: \t\t\t\t{temp: 7.1f} C\n')

        return temp
    
    def calc_adc(self, temp): # Convert temperature to ADC value. Used to set min/max. Reverse of calc_temp()
        
        pt1000_resistance_gain_corrected = self._r0*(1 + self._lin_temp_coeff*temp + self._quad_temp_coeff*temp**2)

        pt1000_resistance = (pt1000_resistance_gain_corrected - self._adc_gain_coefficient_0) / self._adc_gain_coefficient_1

        pt1000_voltage = self._vref * (pt1000_resistance/(pt1000_resistance + self._pt1000_pullup_resistor))

        vdiff = pt1000_voltage - self._reference_resistor_voltage

        raw_adc = int(vdiff*10*(512/self._vref)*self._sample_count)
        return raw_adc

class BedTempMK3S(PrinterADCtoTemperatureAVR, Thermistor):
    def __init__(self, config):
        PrinterADCtoTemperatureAVR.__init__(self, config)

        self.thermistor = Thermistor(config.getfloat('pullup_resistor', 4700., above=0.),
                                     config.getfloat('inline_resistor', 0., minval=0.))
        
        self.thermistor.setup_coefficients(t1=config.getfloat('t1', 0., above=0.), 
                                r1=config.getfloat('r1', 0., above=0.),
                                t2=config.getfloat('t2', 0., above=0.),
                                r2=config.getfloat('r2', 0., above=0.),
                                t3=config.getfloat('t3', 0., above=0.),
                                r3=config.getfloat('r3', 0., above=0.),
                                name=config.get_name())
        self.adc_gain_correction_coefficient_0 = config.getfloat('adc_gain_correction_coefficient_0')
        self.adc_gain_correction_coefficient_1 = config.getfloat('adc_gain_correction_coefficient_1')
        self.adc_gain_correction_coefficient_2 = config.getfloat('adc_gain_correction_coefficient_2')
        self._vref = config.getfloat('vref', above=0.)

        self.fifo = np.full(10, 7845, dtype=np.int16)

    def calc_temp(self, raw_adc):
        
        # self.fifo = np.roll(self.fifo, 1)
        # np.put(self.fifo, [0], raw_adc)
        # self.adc_avg = np.average(self.fifo)#, weights=np.arange(self.fifo.size, 0, -1))       
        self.adc_avg = raw_adc     
        adc_avg_div8 = (self.adc_avg/self._sample_count)

        self.adc_gain_corr = self.adc_gain_correction_coefficient_2*adc_avg_div8**2 + self.adc_gain_correction_coefficient_1*adc_avg_div8 + self.adc_gain_correction_coefficient_0
        self.adc_gain_corr_normalized =self.adc_gain_corr / self._vref
        # print("ADC after normalization:             %.4f" % self.adc_gain_corr)
        temp_gain_corr = self.thermistor.calc_temp(self.adc_gain_corr_normalized)

        # adc_voltage = adc_avg_div8*(4979/1024)
        # Normalize the raw ADC value (0.0 to 1.0). Thermistor.calc_temp() requirement.
        # adc_normalized = self._normalization_vector * self.adc_avg
        # print("adc_normalized:                      %.4f" % adc_normalized)
        # temp = self.thermistor.calc_temp(adc_normalized)
        # print(f'*******BED THERMISTOR DATA********\n'
        #       f'Codes:\t{adc_avg_div8: 7.1f}\n'
        #       f'ADC :\t{adc_voltage: 7.1f} mV\n'
        #       f'ADC LR:\t{self.adc_gain_corr: 7.1f} mV\n'
        #       f'Temp:\t{temp: 7.1f} C\n'
        #       f'Temp LR:{temp_gain_corr: 7.1f} C\n')
    
        return temp_gain_corr

    def calc_adc(self, temp):# Convert temperature to ADC value. Used to set min/max. Reverse of calc_temp()   
        raw_adc = int(self.thermistor.calc_adc(temp) * 1024 * self._sample_count)
        return raw_adc

Sensors = {
    "hotend_temp": PT1000_WheatStone,
    "bed_temp": BedTempMK3S,
}

def load_config(config):
    # Register sensors
    pheaters = config.get_printer().load_object(config, "heaters")
    for name, klass in Sensors.items():
        pheaters.add_sensor_factory(name, klass)
