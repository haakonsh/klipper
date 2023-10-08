sudo klippy-env/bin/python klipper/klippy/console.py ../../dev/prusa-einsy

allocate_oids count=10
config_analog_in oid=0 admux=2
query_analog_in oid=0 clock={0x00F42400*8} sample_ticks={0x00F42400*1} sample_count=10 rest_ticks={0x00F42400*1} min_value={10*-512} max_value={10*511} range_check_count=0
finalize_config crc=3455
get_config


Compile and flash:
sudo printer_data/config/RatOS/boards/prusa-einsy/make-and-flash-mcu.sh 

Test klippy /w logs:
sudo klippy-env/bin/python klipper/klippy/klippy.py -v printer_data/config/printer.cfg -l printer_data/logs/klippy.log -I printer_data/comms/klippy.serial

Read MCU memory:
sudo klippy-env/bin/python  klipper/scripts/dump_mcu.py -s 0x0 -l 0x3 ../../dev/prusa-einsy debug.bin && hd -v debug.bin