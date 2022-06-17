#ifdef NATIVE
#include "nativemocks.h"
#else
#include <Arduino.h>
#include <avr/eeprom.h>

#endif

#include "stackcheck.h"

#include "devmoders485.h"

#define DEVMODE 1
#ifdef DEVMODE
DevRS485Class * rs485 = &DevRS485;
UartClass * debug = &Serial1;
#else
UartClass * rs485 = &Serial;
UartClass * debug = &Serial1;
#endif

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



*/


#define ADC_VOLTAGE A6 // ADC 6 is PA6 on pin 4
#define ADC_CURRENT A7 // ADC 7 is PA7 on pin 5
#define ADC_NTC A5 // ADC 5 is PA5 on pin 3



#define DEVICE_TYPE 0x01
#define DEVICE_ON 0xff


#define FRAME_START_SILENCE 3645  // 3645uS
#define QUERY_FRAME_LEN 12
#define RESPONSE_FRAME_LEN 20 // address + function + count + 3xfloats (4 bytes ) + 2crc = 3+12+2 = 17
#define MAX_REGISTER_COUNT 6
#define MAX_INPUT_REGISTER 3
#define MAX_HOLDING_REGISTER 8
byte frameBuffer[RESPONSE_FRAME_LEN];


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
  0x00, 0x00,  // voltage offset 0
  0x64, 0x00,  // voltage scale 100
  0x00, 0x00,  // current offset 0
  0x64, 0x00,  // current scale 100
  0x00, 0x00,  // temperature offset 0
  0x64, 0x00,  // temperature scale 100
  0x00, 0x00  // serial number
};



// function 3 and 4
#define FRAME_OFFSET_DEVICE_ADDRESS 0 
#define FRAME_OFFSET_FUNCTION_CODE 1 
#define FRAME_OFFSET_REGISTER 2
#define FRAME_OFFSET_REGISTER_COUNT 4
#define FRAME_OFFSET_QUERY_CHECKSUM 6

//  function 6
#define FRAME_OFFSET_REGISTER_VALUE 4

// response
#define FRAME_OFFSET_BYTE_COUNT 2

// exception responses
#define FRAME_EXCEPTION_CODE 2
#define EXCEPTION_ILEGAL_FUNCTION 0x01
#define EXCEPTION_ILEGAL_DATA_ADDRESS 0x02
#define EXCEPTION_ILEGAL_DATA_VALUE 0x03
#define EXCEPTION_SLAVE_DEVICE_FALURE 0x04


#define getInt16(pos)  (int16_t)((frameBuffer[pos]<<8) + frameBuffer[pos+1])
#define getUInt16(pos) (uint16_t)((frameBuffer[pos]<<8) + frameBuffer[pos+1])


uint8_t deviceAddress = 0x00;

// CRC POLYNOME = x15 + 1 =  1000 0000 0000 0001 = 0x8001
#define CRC16_IBM                   0x8005
uint16_t crc16(const uint8_t *array, uint16_t length)
{
  uint16_t crc = 0x00;
  while (length--) 
  {
    uint8_t data = *array++;
    crc ^= ((uint16_t)data) << 8;
    for (uint8_t i = 8; i; i--) 
    {
      if (crc & (1 << 15))
      {
        crc <<= 1;
        crc ^= CRC16_IBM;
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}



void loadDefaults() {
  // defaults need to be reset, numbers are little endian, so they can be read directly.
  // when sending out bytes need to be swapped.
  uint8_t eepromContents[HR_SIZE];
  eeprom_read_block((void*)&eepromContents[0], (const void*)0, HR_SIZE);
  uint16_t crcv =  crc16(&eepromContents[0], HR_SIZE-2);
  uint16_t crcvs = (0xff00&eepromContents[HR_CRC]) | (0xff00&(eepromContents[HR_CRC+1]<<8)); // little endian
  if (crcv != crcvs ||eepromContents[HR_DEVICE_ADDRESS] == 0 ) {
    debug->printf(F("EPROM not initialised\n"));
    for(uint8_t i = 0; i < HR_SIZE-2; i++) {
      eepromContents[i] = pgm_read_byte_near(epromDefaultValues+i);
    }
    crcv =  crc16(&eepromContents[0], HR_SIZE-2);
    eepromContents[HR_CRC] = 0xff&(crcv);
    eepromContents[HR_CRC+1] = 0xff&(crcv>>8); // little endian
    eeprom_update_block(&eepromContents[0], (void*)0, HR_SIZE);
  } else {
    debug->printf(F("EPROM initialised %d = %d  device %d\n"),crcv, crcvs, eepromContents[HR_DEVICE_ADDRESS]);
  }
  deviceAddress = eepromContents[HR_DEVICE_ADDRESS];
}

/* definition to expand macro then apply to pragma message */
#define VALUE_TO_STRING(x) #x
#define VALUE(x) VALUE_TO_STRING(x)
#define VAR_NAME_VALUE(var) #var "="  VALUE(var)

//#pragma message(VAR_NAME_VALUE(PIN_HWSERIAL0_TX))


void setup() {

  SERIAL_RS485; // enable RS485 flow control on PA4/XDIR/Pin 2 
  rs485->begin(9600, SERIAL_8N1);
  debug->begin((uint32_t)115200, SERIAL_8N1);

  debug->printf(F("Starting Battery Monitor"));
  frameBuffer[RESPONSE_FRAME_LEN-1] = 0xee;



  loadDefaults();
}



/**
 * @brief read a modbus query from the serial line
 * 
 * @return int8_t the modbus function code or 0 if no code
 */
int8_t readQuery() {
  static unsigned long startFrameTimeout = micros();
  static bool readingFrame = true;
  static uint8_t readFramePos = 0;
  if ( rs485->available() > 0) {
    // was there silence ?
    unsigned long now = micros();
    if ( now > startFrameTimeout ) {
      if ( readFramePos > 0) {
        debug->printf(F("New Frame\n"));
      }
      // new frame
      readFramePos = 0;
      readingFrame = true;

    }
    // restart startFrameTimeout
    startFrameTimeout = now + FRAME_START_SILENCE;
    size_t n = rs485->readBytes(&frameBuffer[readFramePos], QUERY_FRAME_LEN-readFramePos);
    if (readingFrame) {
        readFramePos += n;
        debug->printf(F("Frame now %d\n"), readFramePos);
        uint8_t frameLength = QUERY_FRAME_LEN;
        if (readFramePos > FRAME_OFFSET_FUNCTION_CODE) {
          switch(frameBuffer[FRAME_OFFSET_FUNCTION_CODE]) {
            case 3: 
            case 4: 
            case 6: 
              frameLength = 8;
              break;
            case 17: 
              frameLength = 4; 
              break;
          }
          debug->printf(F("Frame now %d, function %d, expecting %d\n"), readFramePos, frameBuffer[FRAME_OFFSET_FUNCTION_CODE], frameLength );
        }

        if (readFramePos >= frameLength) {
          // full buffer recieved, perform checksum
          if ( frameBuffer[FRAME_OFFSET_DEVICE_ADDRESS] == deviceAddress ) {
            uint16_t crcValue = crc16(&frameBuffer[0], frameLength-2);
            uint16_t crcFrameValue = getUInt16(frameLength-2);
             if (crcValue == crcFrameValue ) {
                return frameBuffer[FRAME_OFFSET_FUNCTION_CODE]; 
             } else {
               debug->printf(F("CRC from 0 for %d is %X %X "), FRAME_OFFSET_QUERY_CHECKSUM, crcValue, crcFrameValue);
             }           
          } else {
            debug->printf(F(" Frame Address %d Device address %d\n"), frameBuffer[FRAME_OFFSET_DEVICE_ADDRESS], deviceAddress );
          }
          readFramePos = 0;
          // any more bytes are noise until we see > 3.5 chars of silence.
          readingFrame = false; 
        }
    }
  }
  return 0;
}



/**
 * @brief send and exception frame
 * 
 * @param exceptionCode 
 */
int8_t sendFunctionCodeError(uint8_t exceptionCode) {
  frameBuffer[FRAME_OFFSET_FUNCTION_CODE] |= 0x80;
  frameBuffer[FRAME_EXCEPTION_CODE] = exceptionCode;
  return FRAME_EXCEPTION_CODE+1;
}




int16_t readVoltage() {
  uint16_t dw = eeprom_read_word((const uint16_t *)HR_VOLTAGE_SCALE);
  int16_t voltageScale = (int16_t)(0xffff&dw);
  dw = eeprom_read_word((const uint16_t *)HR_VOLTAGE_OFFSET);
  int16_t voltageOffset = (int16_t)(0xffff&dw);

  int32_t voltage = analogReadEnh(ADC_VOLTAGE, 12, 1);
  // 5/4096
  // 
  voltage += voltageOffset; // offset before scaling in bits
  // R1 = 22K R2 = 10K, hence 
  // (5/4096)*(32/10) V/bit
#define MV10_PER_ADC_BIT 0.390625 // (5/4096)*(32/10) == 0.00390625 V/bit or 3.90625 mV/bitm or 0.390625 (10mV)/bit
  float v = voltage;
  v *= MV10_PER_ADC_BIT*0.001*(voltageScale*0.001);
  return (int16_t) (v);  
}
/**
 * @brief read current as an int 1 == 10mA
 * 
 * @return int16_t 
 */
int16_t readCurrent() {
  uint16_t dw = eeprom_read_word((const uint16_t *)HR_CURRENT_SCALE);
  int16_t currentScale = (int16_t)(0xffff&dw);
  dw = eeprom_read_word((const uint16_t *)HR_CURRENT_OFFSET);
  int16_t currentOffset = (int16_t)(0xffff&dw);

  int gain = 1;
  // autoscale the gain of the ADC.
  int32_t voltage = analogReadEnh(ADC_VOLTAGE, 12, gain);
  while(voltage < 1000 && gain < 16) {
    gain = gain * 2;
    voltage = analogReadEnh(ADC_VOLTAGE, 12, gain);
  }

  // 1x 5V resolution 0.001220703125
  // 2x 2.5V 2.5/4096 0.0006103515625
  // 4x 1.25 1.25/4096 0.0003051757812
  // 8x 0.625 0.625/4096 0.0001525878906
  // 16x 0.3125 0.3125/4096 0.00007629394531
  // convert the reading into a reading a 16x gain
  voltage = voltage*(16/gain);
  voltage -= 8*4096;  // theoretical midpoint
  voltage += currentOffset;  // shift midpoint in 0.00007629394531 steps integer reading representing the output of the MAX8818 
  // multiply this by 0.00007629394531 to get the voltage
  // Gain of MAX9918 is 1+47000/1000 = 48
  // voltage at MAX9918 output is voltage*0.00007629394531
  // voltage at MAX9918 input is (voltage*0.00007629394531)/48
  // current at shunt (100/0.050)*(voltage*0.00007629394531)/48
  // current in  10mA units is 100*((100/0.050)*(voltage*0.00007629394531)/48))
#define MA10_PER_ADC_BIT 0.3178914388  
  float v = voltage;
  v *= MA10_PER_ADC_BIT*0.001*(currentScale*0.001);
  return (int16_t)(v);
}







/**
 * @brief ADC readings at temperatures from MIN_NTC_TEMPERATURE to MAX_NTC_TEMPERATURE in sptes of NTC_TABLE_STEP
 * calculated using https://www.skyeinstruments.com/wp-content/uploads/Steinhart-Hart-Eqn-for-10k-Thermistors.pdf
 * https://www.gotronic.fr/pj2-mf52type-1554.pdf MC53-10K B 3950K
 * 	Supply 	5		
	Rtop	10		
C	R in K	V	mW	ADC
-30	181.7	4.7391757955	0.1236091757	3882.3328116849
-20	98.88	4.5407788391	0.208522173	3719.8060249816
-10	56.06	4.2431123221	0.3211559432	3475.95761429
0	32.96	3.8361266294	0.446476563	3142.5549348231
10	20	3.3333333333	0.5555555556	2730.6666666667
20	12.51	2.7787649933	0.6172290078	2276.3642825411
30	8.048	2.2296099291	0.617688921	1826.4964539007
40	5.312	1.7345872518	0.5664143325	1420.973876698
50	3.588	1.3202826023	0.4858266862	1081.575507801
60	2.476	0.992305226	0.3976856469	812.896441167
70	1.743	0.7421442562	0.3159943184	607.9645746402
80	1.25	0.5555555556	0.2469135802	455.1111111111
90	0.911	0.4174686097	0.1913063008	341.9902850335
100	0.6744	0.3158959754	0.147968961	258.7819830623
110	0.5066	0.2410865551	0.1147310049	197.4981059524

 */
const int16_t ntcTable[] PROGMEM= {
3476,
3143,
2731,
2276,
1826,
1421,
1082,
813,
608,
455,
342,
259,
197
  };
#define NTC_TABLE_LENGTH 13
#define MIN_NTC_TEMPERATURE -10.0
#define MAX_NTC_TEMPERATURE 110.0
#define NTC_TABLE_STEP 10.0


int16_t readTemperature() {
  uint16_t dw = eeprom_read_word((const uint16_t *)HR_TEMPERATURE_SCALE);
  int16_t temperatureScale = (int16_t)(0xffff&dw);
  dw = eeprom_read_word((const uint16_t *)HR_TEMPERATURE_OFFSET);
  int16_t temperatureOffset = (int16_t)(0xffff&dw);

  int16_t reading = analogRead(ADC_NTC);
  float temp = MAX_NTC_TEMPERATURE;
  int16_t cvp = ((int16_t)pgm_read_dword(&ntcTable[0]));
  if ( reading > cvp ) {
    temp = MIN_NTC_TEMPERATURE;
  } else {
    for (int i = 1; i < NTC_TABLE_LENGTH; i++) {
      int16_t cv = ((int16_t)pgm_read_dword(&ntcTable[i]));
      if ( reading > cv ) {
        temp = ((i-1)*NTC_TABLE_STEP)+((cvp-reading)*NTC_TABLE_STEP);
        temp /= (cvp-cv);
        temp += MIN_NTC_TEMPERATURE;
        break;
      }
      cvp = cv;
    }
  }
  // perhaps this could be integer arithmetic ?
  temp += (0.001*temperatureOffset);
  temp *= (0.001*temperatureScale);
  temp /= 0.01;
  return (int16_t) (temp);
}



/**
 * @brief function 3, send holding registers.
 * 
 */
int8_t readHolding() {
  int16_t reg = getInt16(FRAME_OFFSET_REGISTER)*2;
  int16_t count = getInt16(FRAME_OFFSET_REGISTER_COUNT);
  if ( count > MAX_REGISTER_COUNT ) {
    return sendFunctionCodeError(EXCEPTION_ILEGAL_DATA_VALUE);
  } 
  if ( count >= HR_CRC ) {
    return sendFunctionCodeError(EXCEPTION_ILEGAL_DATA_ADDRESS);
  } 
  frameBuffer[FRAME_OFFSET_BYTE_COUNT] = (count)*2;
  uint8_t framePos = FRAME_OFFSET_BYTE_COUNT+1;
  for(int r = reg; r < (reg+count*2); r+= 2) {
    if ( reg > HR_SIZE) {
      frameBuffer[framePos++] = 0x00;
      frameBuffer[framePos++] = 0x00;
    } else {
      frameBuffer[framePos++] = eeprom_read_byte((const uint8_t *)(r+1));
      frameBuffer[framePos++] = eeprom_read_byte((const uint8_t *)(r));
    }
  }
  return framePos;
}

/**
 * @brief function 4 sent input registers
 * 
 */
int8_t readInput() {
  int16_t reg = getInt16(FRAME_OFFSET_REGISTER);
  int16_t count = getInt16(FRAME_OFFSET_REGISTER_COUNT);
  if ( count > MAX_REGISTER_COUNT ) {
    return sendFunctionCodeError(EXCEPTION_ILEGAL_DATA_VALUE);
  } 
  if ( reg > MAX_INPUT_REGISTER ) {
    return sendFunctionCodeError(EXCEPTION_ILEGAL_DATA_ADDRESS);
  }
  frameBuffer[FRAME_OFFSET_BYTE_COUNT] = (count)*2;
  uint8_t framePos = FRAME_OFFSET_BYTE_COUNT+1;
  int16_t reading;
  for(int r = 0; r < count; r++) {
    switch (r+reg) {
      case 0: // voltage  0.01V  (+320v to -320v)
      reading = readVoltage();
      frameBuffer[framePos++] = 0xff&(reading>>8);
      frameBuffer[framePos++] = 0xff&(reading);
      break;
      case 1: // current 0.01A  (+320A to -320A)
      reading = readCurrent();
      frameBuffer[framePos++] = 0xff&(reading>>8);
      frameBuffer[framePos++] = 0xff&(reading);
      break;
      case 2: // temperature 0.01C (+320C to -273C)
      reading = readTemperature();
      frameBuffer[framePos++] = 0xff&(reading>>8);
      frameBuffer[framePos++] = 0xff&(reading);
      break;
      default: // null
      frameBuffer[framePos++] = 0x00;
      frameBuffer[framePos++] = 0x00;
    }
  }
  return framePos;
}
/**
 * @brief update 1 hoding register.
 * 
 */
int8_t writeHolding() {
  int16_t reg = getInt16(FRAME_OFFSET_REGISTER)*2;
  if ( reg < 0 || reg >= HR_CRC ) {
    return sendFunctionCodeError(EXCEPTION_ILEGAL_DATA_ADDRESS);
  }
  int16_t value = getInt16(FRAME_OFFSET_REGISTER_VALUE);

  uint8_t epromContents[HR_SIZE];
  eeprom_read_block((void*)&epromContents[0], (const void*)0, HR_SIZE);
  int16_t currentValue = epromContents[reg] | (0xff00&(epromContents[reg+1]<<8));
  if (reg == HR_SERIAL_NUMBER && currentValue != 0) {
    return sendFunctionCodeError(EXCEPTION_ILEGAL_DATA_VALUE);
  } else if ( currentValue != value) {
    // store little endian
    epromContents[reg] = 0xff&value;
    epromContents[reg+1] = 0xff&(value>>8);
    uint16_t crcv =  crc16(&epromContents[0], HR_SIZE-2);
    epromContents[HR_CRC] = 0xff&(crcv);
    epromContents[HR_CRC+1] = 0xff&(crcv>>8); // little endian
    eeprom_update_block((const void*)&epromContents[0], (void*)0, HR_SIZE);
    // update device address, which may have changed.
    deviceAddress = epromContents[HR_DEVICE_ADDRESS];

  }
  return 6;
}

int8_t writeSlaveId() {
  frameBuffer[FRAME_OFFSET_BYTE_COUNT] = 4;
  frameBuffer[FRAME_OFFSET_BYTE_COUNT+1] = DEVICE_TYPE;
  frameBuffer[FRAME_OFFSET_BYTE_COUNT+2] = DEVICE_ON;
  uint16_t serialNumber = eeprom_read_word((const uint16_t*) HR_SERIAL_NUMBER);
  frameBuffer[FRAME_OFFSET_BYTE_COUNT+3] = 0xff&(serialNumber>>8);
  frameBuffer[FRAME_OFFSET_BYTE_COUNT+4] = 0xff&(serialNumber);
  return FRAME_OFFSET_BYTE_COUNT+5;
}


void loop() {
  int8_t functionCode = readQuery();
  int8_t toSend = -1;
  switch (functionCode) {
    case 0:
     // noop
      break;
    case 3: // read holding
      debug->printf(F("read holding\n"));
      toSend = readHolding();
      break;
    case 4: // read input
      debug->printf(F("read input\n"));
      toSend = readInput();
      break;
    case 6: // write single holding
      debug->printf(F("write holding\n"));
      toSend = writeHolding();
      break;
    case 17: // report slave ID
      debug->printf(F("report slave\n"));
      toSend = writeSlaveId();
      break;
    default:
      debug->printf(F("unexpected function\n"));
      toSend = sendFunctionCodeError(EXCEPTION_ILEGAL_FUNCTION);
      // respond with error
  }
  if ( toSend > 0 ) {
    uint16_t crc = crc16(&frameBuffer[0], toSend);
    frameBuffer[toSend++] = (crc>>8)&0xff;
    frameBuffer[toSend++] = (crc&0xff);
    rs485->write(frameBuffer, toSend);
    rs485->flush();
  } 

  /*
  uint16_t sc = StackCount();
  if ( sc > 100 ) {
    debug->printf(F("Stack Used %d \n"), sc);
  }
  */

}
