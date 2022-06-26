#pragma once

// holding registers locations in eeprom
#define HR_SIZE 18 // size in bytes of the HR store
// offsets
#define HR_DEVICE_ADDRESS 0
#define HR_VOLTAGE_OFFSET 2
#define HR_VOLTAGE_SCALE 4
#define HR_CURRENT_OFFSET 6
#define HR_CURRENT_SCALE 8
#define HR_TEMPERATURE_OFFSET 10
#define HR_TEMPERATURE_SCALE 12
#define HR_SERIAL_NUMBER 14
#define HR_CRC 16

// little endian
const uint8_t epromDefaultValues[HR_CRC] PROGMEM = {
  0x02, 0x00,  // device address 2
  0x00, 0x00,  // voltage offset 0 in 0.1mV steps
  0xE2, 0x26,  // voltage scale 10000=1x calibrated 9954 = 0x26E2 = 0.9954x, 
  0x16, 0xF3,  // current offset 0 in 0.1mA steps  -2556 = 0xF604   -3306 = 0F316
  0x10, 0x27,  // current scale 10000=1x  0x2710
  0x00, 0x00,  // temperature offset 0
  0xE8, 0x03,  // temperature scale 1000=1x  1000=0x03E8  
  0x00, 0x00  // serial number
};

