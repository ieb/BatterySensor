#pragma once

/**
 * @brief Uses Serial to speak in txt form rs485 for debugging purposes.
 * Otherwise behaves like a normal sei
 */

extern uint16_t crc16(const uint8_t *array, uint16_t length);

class DevRS485Class {
    public:
        DevRS485Class() {};
        void begin(uint32_t baud, uint16_t options) { 
            // ignore since this will have already been set on the Serial class
        };
        int available() {
            if ( pos > 0 ) {
                // dont read more while the buffer has data in it.
                Serial.print("Buffer has ");
                Serial.println(pos);
                return pos;
            }
            if ( !Serial.available() ) {
                return pos;
            }
            String line = Serial.readStringUntil("\n");
            line.trim();
            char * cline = (char *) line.c_str();            

            size_t start = pos;
            if ( cline[0] == ':') {
                Serial.print("Modbus Ascii ");
                Serial.println(line);
                // ASCII Modbus
                char *clineptr = &cline[1];
                if ( *clineptr == '>') clineptr++;
                char bytebuf[5];
                bytebuf[0] = '0';
                bytebuf[1] = 'x';
                bytebuf[4] = '\0';
                                
                while(pos < 78 && *clineptr != '\0') {
                    bytebuf[2] = *(clineptr++);
                    if ( *clineptr != '\0') {
                        bytebuf[3] = *(clineptr++);
                    }
                    int x = strtol((const char*)bytebuf, nullptr, 16);
                    readbuffer[pos++] = 0xff&x;
                }
            } else if ( cline[0] == '$') {
                Serial.print("Modbus Decimal ");
                Serial.println(line);
                // fill the buffer up with hex and return the buffer size
                char *clineptr = &cline[1];
                if ( *clineptr == '>') clineptr++;
                
                while(pos < 78 && *clineptr != '\0') {
                    int x = strtol((const char*)clineptr, &clineptr, 10);
                    while( isspace(*clineptr) && *clineptr != '\0') clineptr++;
                    if ( x > 255 || x < -255 ) {
                        readbuffer[pos++] = 0xff&(x>>8);
                        readbuffer[pos++] = 0xff&(x);
                    } else {
                        readbuffer[pos++] = 0xff&(x);
                    }
                }
            } else {
                Serial.print("Noop");
                Serial.println(line);
                return pos;
            }

            // append the CRC.
            uint16_t crcVal = crc16((const uint8_t *) &readbuffer[start], pos-start);
            Serial.print("CRC adding ");
            Serial.print(start);
            Serial.print(" ");
            Serial.print(pos-start);
            Serial.print(" ");
            if ( crcVal < 0xf) {
                Serial.print("000");
            } else if (crcVal < 0xff ) {
                Serial.print("00");
            } else if (crcVal < 0xfff ) {
                Serial.print("0");
            }
            Serial.println(crcVal,HEX);
            readbuffer[pos++] = 0xff&(crcVal>>8);
            readbuffer[pos++] = 0xff&crcVal;

            Serial.print("Buffer now ");
            Serial.print(pos);
            Serial.println(" long");
            for (int i = 0; i < pos; i++) {
                if (readbuffer[i] < 0xfff ) {
                    Serial.print("0");
                }
                Serial.print(readbuffer[i],HEX);
                Serial.print(" ");
            }
            Serial.println(" ");
            
            if ( cline[1] == '>') {
                int fails = 0;
                for(int i = 0; i < pos; i++) {
                    if(readbuffer[i] != writebuffer[i]) {
                        Serial.print("Check failed pos ");
                        Serial.print(i);
                        Serial.print(" ");
                        if ( readbuffer[i] < 0xf) {
                            Serial.print("0");
                        }
                        Serial.print(readbuffer[i],HEX);
                        Serial.print(" != ");
                        if ( writebuffer[i] < 0xf) {
                            Serial.print("0");
                        }
                        Serial.println(writebuffer[i],HEX);
                        fails++;
                    }
                }
                if ( fails == 0) {
                    Serial.println("Check Ok");
                }
                pos = 0;
                return 0;
            } else {
                return pos;
            }
            
        };
        size_t readBytes(uint8_t *buffer, size_t len) {    
            size_t l = min(pos, len);
            for ( size_t i = 0; i < l; i++) {
                buffer[i] = readbuffer[i];
            }
            for ( size_t i = l; i < pos; i++) {
                readbuffer[i-pos] = readbuffer[pos];
            }
            pos -= l;
            return l;
        };
        void write(uint8_t *buffer, size_t len ) {
            Serial.print("<:");
            Serial.print(len);
            Serial.print(":");
            for (int i = 0; i < len; ++i) {
                if ( buffer[i] < 0x0f) {
                    Serial.print(0);
                }
                Serial.print(buffer[i],HEX);
                writebuffer[i] = buffer[i];
            }
            Serial.println(" ");
            Serial.print(">");

        };
        void flush() {

        };
    private:
        uint32_t baud;
        uint16_t options;
        uint8_t readbuffer[80];
        uint8_t writebuffer[80];
        size_t pos = 0;
};

DevRS485Class DevRS485 = DevRS485Class();
