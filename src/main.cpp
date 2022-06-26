




#ifdef NATIVE
#include "nativemocks.h"
#include "holdingregisters.h"

SerialClass * rs485 = &Serial;
SerialClass * debug = &Serial1;


#else
#include <Arduino.h>
#include <avr/eeprom.h>



// #define DEVMODE 1
#ifdef DEVMODE
#include "devmoders485.h"
DevRS485Class * rs485 = &DevRS485;
UartClass * debug = &Serial;
#else
UartClass * rs485 = &Serial1;
UartClass * debug = &Serial;
#endif

#include "modbus.h"
#include "holdingregisters.h"
#include "commandline.h"
#endif

#include "stackcheck.h"

Modbus modbus = Modbus(rs485, debug);
CommandLine commandLine = CommandLine(debug, &modbus);
/*
RAM:   [===       ]  29.5% (used 151 bytes from 512 bytes)
Flash: [===       ]  27.2% (used 2226 bytes from 8192 bytes)

After adding in the EEPROM and holding revisters
RAM:   [===       ]  33.8% (used 173 bytes from 512 bytes)
Flash: [===       ]  34.4% (used 2818 bytes from 8192 bytes)

Added reading for voltages and temp
RAM:   [===       ]  33.8% (used 173 bytes from 512 bytes)
Flash: [=====     ]  54.8% (used 4488 bytes from 8192 bytes)

Added write and save holding registers. 337 bytes free RAM
RAM:   [===       ]  34.2% (used 175 bytes from 512 bytes)
Flash: [======    ]  59.5% (used 4874 bytes from 8192 bytes)

Dropped EEPROM class and moved registers into eeprom storage.
Moved CRC code into main and removed uneeded code.
Added stack checker
Refactored register output to be more efficient.
RAM:   [===       ]  31.2% (used 160 bytes from 512 bytes)
Flash: [====      ]  44.5% (used 3646 bytes from 8192 bytes)

// Moved to Attiny3224 which has more FLASH and RAM and far better ADC's

Advanced Memory Usage is available via "PlatformIO Home > Project Inspect"
RAM:   [=         ]  11.7% (used 360 bytes from 3072 bytes)
Flash: [====      ]  43.5% (used 14245 bytes from 32768 bytes)
Building .pio/build/attiny3224/firmware.hex

Clock speed = 16000000L
Oscillator = internal
BOD level = 2.6v
Save EEPROM = yes
UPDI pin mode = updi
-------------------------
Using manually specified: /dev/cu.wchusbserial26220

Selected fuses:
-------------------------
[fuse0 / wdtcfg   = 0x00]
[fuse1 / bodcfg   = 0x54]
[fuse2 / osccfg   = 0x01]
[fuse4 / tcd0cfg  = 0x00]
[fuse5 / syscfg0  = 0xC5]
[fuse6 / syscfg1  = 0x06]
[fuse7 / append   = 0x00]
[fuse8 / bootend  = 0x00]
[lock  / lockbit  = 0xC5]
-------------------------

*/
/* definition to expand macro then apply to pragma message */
#define VALUE_TO_STRING(x) #x
#define VALUE(x) VALUE_TO_STRING(x)
#define VAR_NAME_VALUE(var) #var "="  VALUE(var)

#pragma message(VAR_NAME_VALUE(CLOCK_TUNE_INTERNAL))
#pragma message(VAR_NAME_VALUE(F_CPU))

#define DEVICE_TYPE 0x01
#define DEVICE_ON 0xff



bool diagnosticsEnabled = false;



void setDiagnostics(bool enabled) {
  diagnosticsEnabled = enabled;
  modbus.setDiagnostics(enabled);
}


void loadDefaults() {
  // defaults need to be reset, numbers are little endian, so they can be read directly.
  // when sending out bytes need to be swapped.
  uint8_t eepromContents[HR_SIZE];
  eeprom_read_block((void*)&eepromContents[0], (const void*)0, HR_SIZE);
  uint16_t crcv =  modbus.crc16(&eepromContents[0], HR_SIZE-2);
  uint16_t crcvs = (0xff00&eepromContents[HR_CRC]) | (0xff00&(eepromContents[HR_CRC+1]<<8)); // little endian
  if (crcv != crcvs ||eepromContents[HR_DEVICE_ADDRESS] == 0 ) {
    debug->print(F("EPROM not initialised"));
    for(uint8_t i = 0; i < HR_SIZE-2; i++) {
      eepromContents[i] = pgm_read_byte_near(epromDefaultValues+i);
    }
    crcv =  modbus.crc16(&eepromContents[0], HR_SIZE-2);
    eepromContents[HR_CRC] = 0xff&(crcv);
    eepromContents[HR_CRC+1] = 0xff&(crcv>>8); // little endian
    eeprom_update_block(&eepromContents[0], (void*)0, HR_SIZE);
  } else {
    debug->print(F("EPROM initialised "));
    debug->print(crcv);
    debug->print(F(" = "));
    debug->print(crcvs);
    debug->print(F("  device "));
    debug->println(eepromContents[HR_DEVICE_ADDRESS]);
  }
  modbus.setDeviceAddress(eepromContents[HR_DEVICE_ADDRESS]);
}

/* definition to expand macro then apply to pragma message */
#define VALUE_TO_STRING(x) #x
#define VALUE(x) VALUE_TO_STRING(x)
#define VAR_NAME_VALUE(var) #var "="  VALUE(var)

//#pragma message(VAR_NAME_VALUE(PIN_HWSERIAL0_TX))


void setup() {
  modbus.begin();
  debug->begin(9600, SERIAL_8N1);
  pinMode(PIN_PA3, INPUT); // NTC
  pinMode(PIN_PA5, INPUT); //Sense -
  pinMode(PIN_PA6, INPUT); // Sense +
  pinMode(PIN_PA7, INPUT); // Bat

  debug->println(F("Starting Battery Monitor"));
  loadDefaults();
}







int16_t readVoltage(bool logEnabled = false ) {
  uint16_t dw = eeprom_read_word((const uint16_t *)HR_VOLTAGE_SCALE);
  int16_t voltageScale = (int16_t)(0xffff&dw);
  dw = eeprom_read_word((const uint16_t *)HR_VOLTAGE_OFFSET);
  int16_t voltageOffset = (int16_t)(0xffff&dw);

  // Divider == 10/42  (R1 = 32, R2 = 10)
  // 16 bit 4.096, mV/bit == 4.096/2^16 == 0.0625 mVadc/bit
  // (4.096/2^16)*42/10 = 0.2625 mVin/bit

  // calibration
  // v
#define MV_PER_16BIT_ADC 0.0625
#define MV_PER_16BIT_VIN 0.2625
  analogReference(INTERNAL4V096); 
  delayMicroseconds(100); // wait at least 60us for the reference change to act
  analogSampleDuration(300);
  int32_t reading = analogReadEnh(PIN_PA7, 16); // discard
  analogSampleDuration(300);
  reading = analogReadEnh(PIN_PA7, 16);
  float mvAdc = reading*MV_PER_16BIT_ADC;
  float mvVin = reading*MV_PER_16BIT_VIN*(0.0001*voltageScale)+(0.1*voltageOffset);

  if ( logEnabled ) {
    debug->print("Read Voltage offset=");
    debug->print(voltageOffset);
    debug->print(" scale=");
    debug->print(voltageScale);
    debug->print(" adc=");
    debug->print(reading);
    debug->print(" mVadc=");
    debug->print(mvAdc);
    debug->print(" mVin=");
    debug->println(mvVin);
  }

  return (int16_t) (mvVin/10);  
}




/**
 * @brief read current as an int 1 == 10mA
 * 
 * @return int16_t 
 */
int16_t readCurrent(bool logEnabled = false) {
  uint16_t dw = eeprom_read_word((const uint16_t *)HR_CURRENT_SCALE);
  int16_t currentScale = (int16_t)(0xffff&dw);
  dw = eeprom_read_word((const uint16_t *)HR_CURRENT_OFFSET);
  int16_t currentOffset = (int16_t)(0xffff&dw);




#define SHUNT75_INTERNAL4V096_MA_16BIT_LSB 3.546099291
#define SHUNT75_INTERNAL2V048_MA_16BIT_LSB 1.773049645
#define SHUNT75_INTERNAL1V024_MA_16BIT_LSB 0.8865248227

#define INTERNAL4V096_MV_16BIT_LSB  0.125 // 4096/2^15  
#define INTERNAL2V048_MV_16BIT_LSB  0.0625 // 2048/2^15
#define INTERNAL1V024_MV_16BIT_LSB  0.03125 // 1024/2^15
#define INTERNAL1V024X2_MV_16BIT_LSB 0.015625
#define INTERNAL1V024X4_MV_16BIT_LSB 0.0078125
#define INTERNAL1V024X8_MV_16BIT_LSB 0.00390625
#define INTERNAL1V024X16_MV_16BIT_LSB 0.001953125
#define MAFACTOR 1333.3333333 //100000/75 in mA

  // set sample time to get a quick estimate of which range to use.
  analogSampleDuration(30);
  analogReference(INTERNAL4V096);  
  delayMicroseconds(100); // wait at least 60us for the reference change to act
  // determine the range 2048/2^^15  0.0625 mV per bit
  // 200mV is over range
  // 
  uint8_t gain = 0;
  float scale = INTERNAL4V096_MV_16BIT_LSB;
  int32_t sample = analogReadDiff(PIN_PA6,PIN_PA5, 16,0); // dispose
  sample = analogReadDiff(PIN_PA6,PIN_PA5, 16,0);
  if ( sample < 1000 && sample > -1000 ) {
    // less than +- 62.5mV 1024 16x range
    gain = 16;
    scale = INTERNAL1V024X16_MV_16BIT_LSB;
    analogReference(INTERNAL1V024);
  } else if ( sample < 2000 && sample > -2000 ) {
    // less than +- 125mV 1024 8x range 
    gain = 8;
    scale = INTERNAL1V024X8_MV_16BIT_LSB;
    analogReference(INTERNAL1V024);  
  } else if ( sample < 4000 && sample > -4000 ) {
    // less than  +-250mV 1024 4x range 
    gain = 4;
    scale = INTERNAL1V024X4_MV_16BIT_LSB;
    analogReference(INTERNAL1V024);  
  } else if ( sample < 16000 && sample > -16000 ){
    // greater than 250mV and less than 1v can be read on the 4096 range at 16bit
    scale = INTERNAL4V096_MV_16BIT_LSB;
    gain = 0;
  } else {
    // something wrong, > 1V, return 0
    return 0;
  }
  analogSampleDuration(300);
  delayMicroseconds(100); // wait at least 60us for the reference change to act

  int32_t reading = analogReadDiff(PIN_PA6,PIN_PA5, 16, gain); // discard
  reading = analogReadDiff(PIN_PA6,PIN_PA5, 16, gain);
  float mV = scale*reading;

  float current = mV*MAFACTOR;
  current *= (0.0001*currentScale);
  current += (0.1*currentOffset);
  if ( logEnabled ) {
    debug->print("Read Current gain=");
    debug->print(gain);
    debug->print(" offset=");
    debug->print(currentOffset);
    debug->print(" scale=");
    debug->print(currentScale);
    debug->print(" mV="); 
    debug->print(mV); 
    debug->print(" mA="); 
    debug->print(current);
    debug->print(" output="); 
    debug->println((int16_t)current/10);
  }

  return (int16_t)current/10; // need output to be in units of 0.01A
}







/**
 * @brief ADC readings at temperatures from MIN_NTC_TEMPERATURE to MAX_NTC_TEMPERATURE in sptes of NTC_TABLE_STEP
 * calculated using https://www.skyeinstruments.com/wp-content/uploads/Steinhart-Hart-Eqn-for-10k-Thermistors.pdf
 * https://www.gotronic.fr/pj2-mf52type-1554.pdf MC53-10K B 3950K
 * 	Supply 	5		
	Rtop	10		
C	R in K	V	ADC
-10	56.06	4.2431123221	4243.1123221314
0	32.96	3.8361266294	3836.1266294227
10	20	3.3333333333	3333.3333333333
20	12.51	2.7787649933	2778.7649933363
30	8.048	2.2296099291	2229.609929078
40	5.312	1.7345872518	1734.5872518286
50	3.588	1.3202826023	1320.2826022962
60	2.476	0.992305226	992.305226034
70	1.743	0.7421442562	742.1442561526
80	1.25	0.5555555556	555.5555555556
90	0.911	0.4174686097	417.46860966
100	0.6744	0.3158959754	315.8959754178
110	0.5066	0.2410865551	241.0865551177

 */
const int16_t ntcTable[] PROGMEM= {
4243,
3836,
3333,
2779,
2230,
1735,
1320,
992,
742,
556,
417,
316,
241
  };
#define NTC_TABLE_LENGTH 13
#define MIN_NTC_TEMPERATURE -10.0
#define MAX_NTC_TEMPERATURE 110.0
#define NTC_TABLE_STEP 10.0


int16_t readTemperature(bool logEnabled = false ) {
  uint16_t dw = eeprom_read_word((const uint16_t *)HR_TEMPERATURE_SCALE);
  int16_t temperatureScale = (int16_t)(0xffff&dw);
  dw = eeprom_read_word((const uint16_t *)HR_TEMPERATURE_OFFSET);
  int16_t temperatureOffset = (int16_t)(0xffff&dw);

  
  

  // scale for supply voltage, which should be 5V
  // 5v / 10 = 0.5v,  at 0.00025v/LSB = 5V would be 2000
  // unforotunately must do this as float 
  analogReference(INTERNAL4V096); // 1mV per LSB
  delayMicroseconds(100); // wait at least 60us for the reference change to act
  analogSampleDuration(300);
  int16_t reading = analogReadEnh(PIN_PA3, 12, 1);

  float temp = MAX_NTC_TEMPERATURE;
  int16_t cvp = ((int16_t)pgm_read_dword(&ntcTable[0]));
  if ( reading > cvp ) {
    temp = MIN_NTC_TEMPERATURE;
  } else {
    for (int i = 1; i < NTC_TABLE_LENGTH; i++) {
      int16_t cv = ((int16_t)pgm_read_dword(&ntcTable[i]));
      if ( reading > cv ) {
        temp = ((cvp-reading)*NTC_TABLE_STEP);
        temp /= (cvp-cv);
        temp += ((i-1)*NTC_TABLE_STEP)+MIN_NTC_TEMPERATURE;
        break;
      }
      cvp = cv;
    }
  }
  // perhaps this could be integer arithmetic ?
  float outputTemp = temp+(0.001*temperatureOffset);
  outputTemp *= (0.001*temperatureScale);
  outputTemp /= 0.01;
  if ( logEnabled ) {
    debug->print("Read Temperature offset=");
    debug->print(temperatureOffset);
    debug->print(" scale=");
    debug->print(temperatureScale);
    debug->print(" adc=");
    debug->print(reading);
    debug->print(" temp=");
    debug->print(temp);
    debug->print(" output=");
    debug->println(outputTemp);
  }
  return (int16_t) (outputTemp);
}



/**
 * @brief function 4 sent input registers
 * 
 */
int8_t readInput() {
  int16_t reg = modbus.getRegisterStart(); 
  int16_t count = modbus.getRegisterCount(); 
  if ( count > MAX_REGISTER_COUNT ) {
    modbus.sendFunctionCodeError(EXCEPTION_ILEGAL_DATA_VALUE);
  } else if ( reg > modbus.getMaxInputRegister() ) {
    modbus.sendFunctionCodeError(EXCEPTION_ILEGAL_DATA_ADDRESS);
  } else {
    modbus.startOutputFrame();
    for(int r = 0; r < count; r++) {
      switch (r+reg) {
        case 0: // voltage  0.01V  (+320v to -320v)
          modbus.addOutputInt(readVoltage(diagnosticsEnabled));
          break;
        case 1: // current 0.01A  (+320A to -320A)
          modbus.addOutputInt(readCurrent(diagnosticsEnabled));
          break;
        case 2: // temperature 0.01C (+320C to -273C)
          modbus.addOutputInt(readTemperature(diagnosticsEnabled));
          break;
        default:
          modbus.outputRegister((r+reg));
      }
    }
    modbus.send();
  }
}


/**
* @brief function 3, send holding registers.
* 
*/
int8_t readHolding() {
    int16_t reg = modbus.getRegisterStart(); 
    int16_t count = modbus.getRegisterCount(); 
    if ( count > MAX_REGISTER_COUNT ) {
        modbus.sendFunctionCodeError(EXCEPTION_ILEGAL_DATA_VALUE);
    } else if ( (reg+count-1)*2 >= HR_SIZE ) {
        modbus.sendFunctionCodeError(EXCEPTION_ILEGAL_DATA_ADDRESS);
    } else {
      modbus.startOutputFrame();
      for(int r = 0; r < count; r++) {     
          if ( (reg+r)*2 >= HR_CRC ) {
            modbus.addOutputInt(0x0000);
          } else {
            // covert the dword to an int16_t before sending by masking it with 0xffff
            uint16_t epromVal = eeprom_read_word((const uint16_t *)((reg+r)*2));
            int16_t regVal = (int16_t)(0xffff&epromVal);
            modbus.addOutputInt(regVal);

          }
      }
      modbus.send();
    }
};


/**
 * @brief update 1 hoding register.
 * 
 */
void writeHolding() {
  int16_t regPos = modbus.getRegisterStart()*2; 
  if ( regPos < 0 || regPos >= HR_CRC ) {
    modbus.sendFunctionCodeError(EXCEPTION_ILEGAL_DATA_ADDRESS);
  } else {
    int16_t value = modbus.getRegisterValue();
    uint8_t epromContents[HR_SIZE];
    eeprom_read_block((void*)&epromContents[0], (const void*)0, HR_SIZE);
    int16_t currentValue = epromContents[regPos] | (0xff00&(epromContents[regPos+1]<<8));
    if (regPos == HR_SERIAL_NUMBER && currentValue != 0) {
      modbus.sendFunctionCodeError(EXCEPTION_ILEGAL_DATA_VALUE);
    } else if ( currentValue != value) {
      // store little endian
      epromContents[regPos] = 0xff&value;
      epromContents[regPos+1] = 0xff&(value>>8);
      uint16_t crcv =  modbus.crc16(&epromContents[0], HR_SIZE-2);
      epromContents[HR_CRC] = 0xff&(crcv);
      epromContents[HR_CRC+1] = 0xff&(crcv>>8); // little endian
      eeprom_update_block((const void*)&epromContents[0], (void*)0, HR_SIZE);
      // update device address, which may have changed.
      modbus.setDeviceAddress(epromContents[HR_DEVICE_ADDRESS]);
      modbus.setToSend(6);
      modbus.send();
    }
  }
}

void writeSlaveId() {
  modbus.startOutputFrame();
  modbus.addOutputByte(DEVICE_TYPE);
  modbus.addOutputByte(DEVICE_ON);
  modbus.addOutputInt(eeprom_read_word((const uint16_t*) HR_SERIAL_NUMBER));
  modbus.send();
}


void loop() {
  commandLine.checkCommand();
  int8_t functionCode = modbus.readQuery();
  unsigned long start = micros();
  switch (functionCode) {
    case 0:
     // noop
      break;
    case 3: // read holding
      readHolding();
      if ( diagnosticsEnabled ) {
        debug->print(F("read holding time="));
        debug->print(micros()-start);
        debug->println(F("us"));
      }
      break;
    case 4: // read input
      readInput();
      if ( diagnosticsEnabled ) {
        debug->print(F("read input time="));
        debug->print(micros()-start);
        debug->println(F("us"));
      }
      break;
    case 6: // write single holding
      writeHolding();
      if ( diagnosticsEnabled ) {
        debug->print(F("write holding time="));
        debug->print(micros()-start);
        debug->println(F("us"));
      }
      break;
    case 17: // report slave ID
      writeSlaveId();
      if ( diagnosticsEnabled ) {
        debug->print(F("report slave time="));
        debug->print(micros()-start);
        debug->println(F("us"));
      }
      break;
    default:
      if ( diagnosticsEnabled ) {
        debug->println(F("unexpected function"));
      }
      modbus.sendFunctionCodeError(EXCEPTION_ILEGAL_FUNCTION);
      // respond with error
  }



}
