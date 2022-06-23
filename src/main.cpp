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

#include "holdingregisters.h"
#include "commandline.h"

#endif

#include "stackcheck.h"

CommandLine commandLine = CommandLine(debug);
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

*/



#define DEVICE_TYPE 0x01
#define DEVICE_ON 0xff


#define FRAME_START_SILENCE 3645  // 3645uS
#define QUERY_FRAME_LEN 12
#define RESPONSE_FRAME_LEN 20 // address + function + count + 3xfloats (4 bytes ) + 2crc = 3+12+2 = 17
#define MAX_REGISTER_COUNT 6
#define MAX_INPUT_REGISTER 3
#define MAX_HOLDING_REGISTER 8
byte frameBuffer[RESPONSE_FRAME_LEN];




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

// counters
uint16_t  framesRecieved=0;
uint16_t  framesSent=0;
uint16_t  framesErrorRecieved=0;
uint16_t  framesIgnored=0;
uint16_t  framesErrorSent=0;


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
  debug->begin(9600, SERIAL_8N1);
  pinMode(PIN_PA3, INPUT); // NTC
  pinMode(PIN_PA5, INPUT); //Sense -
  pinMode(PIN_PA6, INPUT); // Sense +
  pinMode(PIN_PA7, INPUT); // Bat

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
                framesRecieved++;
                return frameBuffer[FRAME_OFFSET_FUNCTION_CODE]; 
             } else {
                framesErrorRecieved++;
                debug->printf(F("CRC from 0 for %d is %X %X "), FRAME_OFFSET_QUERY_CHECKSUM, crcValue, crcFrameValue);
             }           
          } else {
            framesIgnored++;
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
  framesErrorSent++;
  frameBuffer[FRAME_OFFSET_FUNCTION_CODE] |= 0x80;
  frameBuffer[FRAME_EXCEPTION_CODE] = exceptionCode;
  return FRAME_EXCEPTION_CODE+1;
}




int16_t readVoltage() {
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

  return (int16_t) (mvVin/10);  
}


float readSample(float scalar, uint8_t times, uint8_t gain) {
  debug->print(analogReadDiff(PIN_PA6,PIN_PA5, 16, gain)); // discard
  int32_t s = analogReadDiff(PIN_PA6,PIN_PA5, 16, gain);
  float mul = scalar/times;
  float acc = s*mul;
  for (uint8_t i = 1; i < times; i++) {
    delayMicroseconds(100);
    int32_t s = analogReadDiff(PIN_PA6,PIN_PA5, 16, gain);
    acc += s*mul;
  }
  return acc;
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
    debug->print(" 16x  ");  
  } else if ( sample < 2000 && sample > -2000 ) {
    // less than +- 125mV 1024 8x range 
    gain = 8;
    scale = INTERNAL1V024X8_MV_16BIT_LSB;
    analogReference(INTERNAL1V024);  
    debug->print(" 8x  ");  
  } else if ( sample < 4000 && sample > -4000 ) {
    // less than  +-250mV 1024 4x range 
    gain = 4;
    scale = INTERNAL1V024X4_MV_16BIT_LSB;
    analogReference(INTERNAL1V024);  
    debug->print(" 4x  ");  
  } else if ( sample < 16000 && sample > -16000 ){
    // greater than 250mV and less than 1v can be read on the 4096 range at 16bit
    scale = INTERNAL4V096_MV_16BIT_LSB;
    gain = 0;
    debug->print(" 4096 ");  
  } else {
    // something wrong, > 1V, return 0
    return 0;
  }
  analogSampleDuration(300);
  delayMicroseconds(100); // wait at least 60us for the reference change to act

  // read over 20 samples accumulating
  float result = readSample(scale, 20, gain);
  float current = result*MAFACTOR*(0.0001*currentScale)+(0.1*currentOffset);
  debug->print(" 20 samples mv="); debug->print(result); debug->print(" mA="); debug->println(current);
  result = readSample(scale, 1, gain);
  current = result*MAFACTOR*(0.0001*currentScale)+(0.1*currentOffset);
  debug->print(" 1 samples mv=");debug->print(result); debug->print(" mA="); debug->println(current);
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


int16_t readTemperature() {
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
  return (int16_t) (outputTemp);
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
    framesSent++;
    uint16_t crc = crc16(&frameBuffer[0], toSend);
    frameBuffer[toSend++] = (crc>>8)&0xff;
    frameBuffer[toSend++] = (crc&0xff);
    rs485->write(frameBuffer, toSend);
    rs485->flush();
  } 


  commandLine.checkCommand();

  /*
  uint16_t sc = StackCount();
  if ( sc > 100 ) {
    debug->printf(F("Stack Used %d \n"), sc);
  }
  */

}
