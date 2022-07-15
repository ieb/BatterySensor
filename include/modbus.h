#pragma once

#define FRAME_START_SILENCE 3645  // 3645uS or 3.5 chars at 9600 baud
#define QUERY_FRAME_LEN 12
#define RESPONSE_FRAME_LEN 20 // address + function + count + 3xfloats (4 bytes ) + 2crc = 3+12+2 = 17
#define MAX_REGISTER_COUNT 6
#define MAX_INPUT_REGISTER 3
#define MAX_HOLDING_REGISTER 8




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



// counters


class Modbus {
    public:
        Modbus(UartClass *rs485, UartClass *debug) : rs485{rs485}, debug{debug} { };
        void begin() {
              rs485->begin(9600, SERIAL_8N1 | SERIAL_RS485);
              frameBuffer[RESPONSE_FRAME_LEN-1] = 0xee;
              startFrameTimeout = micros();
        };
        void setDeviceAddress(uint8_t d) {
            deviceAddress = d;
        };
        uint8_t getDeviceAddress() {
            return deviceAddress;
        };

        void setDiagnostics(bool enabled) {
            diagnosticsEnabled = enabled;
        };

        /**
         * @brief crc for mdbus, polynomial = 0x8005, reverse in, reverse out, init 0xffff;
         * 
         * @param array 
         * @param length 
         * @return uint16_t 
         */
        uint16_t crc16(const uint8_t *array, uint16_t length) {
            uint16_t crc = 0xffff;
            while (length--) {
                if ((length & 0xFF) == 0) yield();  // RTOS
                uint8_t data = *array++;
                data = (((data & 0xAA) >> 1) | ((data & 0x55) << 1));
                data = (((data & 0xCC) >> 2) | ((data & 0x33) << 2));
                data =          ((data >> 4) | (data << 4));
                crc ^= ((uint16_t)data) << 8;
                for (uint8_t i = 8; i; i--) {
                if (crc & (1 << 15)) {
                    crc <<= 1;
                    crc ^= 0x8005;
                } else {
                    crc <<= 1;
                }
                }
            }
            crc = (((crc & 0XAAAA) >> 1) | ((crc & 0X5555) << 1));
            crc = (((crc & 0xCCCC) >> 2) | ((crc & 0X3333) << 2));
            crc = (((crc & 0xF0F0) >> 4) | ((crc & 0X0F0F) << 4));
            //  crc = (( crc >> 8) | (crc << 8));
            //  crc ^= endmask;
            return crc;
        };
        void dumpFrame(uint8_t size) {
            if ( diagnosticsEnabled ) {
                for(uint8_t i = 0; i < size; i++) {
                if (frameBuffer[i] < 16) {
                    debug->print(" 0x0");
                } else {
                    debug->print(" 0x");
                }
                debug->print(frameBuffer[i],HEX);
                }
                debug->print("\n");    
            }
        };

        void dumpStatus() {
            debug->print(F("Frames Recieved:"));
            debug->print(framesRecieved);
            debug->print(F(" Sent:"));
            debug->print(framesSent);
            debug->print(F(" Ignored:"));
            debug->print(framesErrorRecieved);
            debug->print(F(" Ignored:"));
            debug->print(framesErrorSent);
            debug->print(F(" Ignored:"));
            debug->print(framesIgnored);
            debug->print(F(" Overrun:"));
            debug->println(bufferOverrun);

        }
        /**
        * @brief read a modbus query from the serial line
        * 
        * @return int8_t the modbus function code or 0 if no code
        */
        int8_t readQuery() {
            int8_t functionCode = 0;
            if ( rs485->available() > 0) {
                // was there silence ?
                unsigned long now = micros();
                if ( now > startFrameTimeout ) {
                    if ( diagnosticsEnabled && readFramePos > 0) {
                        debug->print(F("New Frame "));
                        debug->println(now-startFrameTimeout);
                    }
                    // new frame
                    readFramePos = 0;
                    readingFrame = true;

                }
                while(functionCode == 0) {
                    delayMicroseconds(1041); // 1 char at 9600 baud
                    int16_t b = rs485->read();
                    if (b == -1) {
                        break;
                    } else if ( readingFrame ) {
                        if ( readFramePos >= QUERY_FRAME_LEN-1 ) {
                            bufferOverrun++;
                            readingFrame = false;
                            continue;
                        }
                        frameBuffer[readFramePos++] = 0xff&b;
                        if ( readFramePos > FRAME_OFFSET_DEVICE_ADDRESS && frameBuffer[FRAME_OFFSET_DEVICE_ADDRESS] != deviceAddress ) {
                            readingFrame = false;
                            framesIgnored++;
                            if ( diagnosticsEnabled ) {
                                debug->print(F(" Frame Address  "));
                                debug->print(frameBuffer[FRAME_OFFSET_DEVICE_ADDRESS]);
                                debug->print(F(" Device address "));
                                debug->println(deviceAddress);
                                dumpFrame(8);
                            }
                        } else if ( readFramePos > FRAME_OFFSET_FUNCTION_CODE ) {
                            uint8_t frameLength = QUERY_FRAME_LEN;
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
                            if (readFramePos >= frameLength) {
                                uint16_t crcValue = crc16(&frameBuffer[0], frameLength-2);
                                uint16_t crcFrameValue = getUInt16(frameLength-2);
                                if (crcValue == crcFrameValue ) {
                                    framesRecieved++;
                                    functionCode = frameBuffer[FRAME_OFFSET_FUNCTION_CODE]; 
                                    if ( diagnosticsEnabled ) {
                                        debug->print("Full frame, CRC Ok frameLength=");
                                        debug->print(readFramePos);
                                        debug->print(" function=");
                                        debug->println(functionCode);
                                        debug->print("Recv Frame:");
                                        dumpFrame(readFramePos);
                                    }
                                    readingFrame = false; 
                                    startFrameTimeout = micros() + FRAME_START_SILENCE;
                                } else {
                                    framesErrorRecieved++;
                                    readingFrame = false;
                                    if ( diagnosticsEnabled ) {
                                        debug->print(F("CRC from 0 for "));
                                        debug->print(frameLength-2);
                                        debug->print(F(" is calculated: "));
                                        debug->print(crcValue,HEX);
                                        debug->print(F("  recieved: "));
                                        debug->println(crcFrameValue,HEX);
                                    }
                                } 
                            }
                        }          
                    }
                }
                startFrameTimeout = micros() + FRAME_START_SILENCE;
            }
            return functionCode;
        };

        int16_t getInt16(int8_t pos) {
            return (int16_t)((frameBuffer[pos]<<8) + frameBuffer[pos+1]);
        };
        uint16_t getUInt16(int8_t pos) {
            return (uint16_t)((frameBuffer[pos]<<8) + frameBuffer[pos+1]);
        };

        int16_t getRegisterStart() {
            return (int16_t)((frameBuffer[FRAME_OFFSET_REGISTER]<<8) + frameBuffer[FRAME_OFFSET_REGISTER+1]);
        }
        int16_t getRegisterCount() {
            return (int16_t)((frameBuffer[FRAME_OFFSET_REGISTER_COUNT]<<8) + frameBuffer[FRAME_OFFSET_REGISTER_COUNT+1]);
        }
        int16_t getRegisterValue() {
            return (int16_t)((frameBuffer[FRAME_OFFSET_REGISTER_VALUE]<<8) + frameBuffer[FRAME_OFFSET_REGISTER_VALUE+1]);
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
            toSend = FRAME_EXCEPTION_CODE+1;
            send();
        };

        void setToSend(int8_t s) {
            toSend = s;
        };
        void send() {
            if ( toSend > 0 ) {
                framesSent++;
                uint16_t crc = crc16(&frameBuffer[0], toSend);
                frameBuffer[toSend++] = (crc>>8)&0xff;
                frameBuffer[toSend++] = (crc&0xff);
                rs485->write(frameBuffer, toSend);
                rs485->flush();

                if ( diagnosticsEnabled ) {
                    debug->print("Send Frame:");
                }
                dumpFrame(toSend);
                toSend = 0;
            }
        };


        void startOutputFrame() {
            toSend = FRAME_OFFSET_BYTE_COUNT+1;
            frameBuffer[FRAME_OFFSET_BYTE_COUNT] = 0;
        };
        void addOutputByte(uint8_t b) {
            frameBuffer[toSend++] = b;
            frameBuffer[FRAME_OFFSET_BYTE_COUNT] = toSend-(FRAME_OFFSET_BYTE_COUNT+1);
        };
        void addOutputInt(uint16_t u) {
            frameBuffer[toSend++] = 0xff&(u>>8);
            frameBuffer[toSend++] = 0xff&(u);
            frameBuffer[FRAME_OFFSET_BYTE_COUNT] = toSend-(FRAME_OFFSET_BYTE_COUNT+1);
        };

        void addOutputInt(int16_t u) {
            frameBuffer[toSend++] = 0xff&(u>>8);
            frameBuffer[toSend++] = 0xff&(u);
            frameBuffer[FRAME_OFFSET_BYTE_COUNT] = toSend-(FRAME_OFFSET_BYTE_COUNT+1);
        };

        int16_t getMaxInputRegister() {
            return 32000;
        }

        void outputRegister(int16_t r) {
            switch(r) {
                case 1000: addOutputInt(framesRecieved); break;
                case 1001: addOutputInt(framesSent); break;
                case 1002: addOutputInt(framesErrorRecieved); break;
                case 1003: addOutputInt(framesIgnored); break;
                case 1004: addOutputInt(framesErrorSent); break;
                case 1005: addOutputInt(bufferOverrun); break;
                default: addOutputInt(r); break;
            }
        }




    private:
        UartClass * rs485;
        UartClass * debug;
        uint8_t deviceAddress = 0x00;
        bool ignoreFrame = false;
        uint8_t readFramePos = 0;
        uint8_t toSend = 0;
        byte frameBuffer[RESPONSE_FRAME_LEN];
        uint16_t  framesRecieved=0;
        uint16_t  framesSent=0;
        uint16_t  framesErrorRecieved=0;
        uint16_t  framesIgnored=0;
        uint16_t  framesErrorSent=0;
        uint16_t  bufferOverrun=0;
        bool diagnosticsEnabled = false;
        unsigned long startFrameTimeout = 0;
        bool readingFrame = true;

        
};