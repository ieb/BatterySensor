#ifdef NATIVE
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/time.h>
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <inttypes.h>

typedef unsigned char byte;


#define digitalWrite(pin,level)  std::cout << "Digital pin:" << pin << " level:" << level << std::endl

#define CRC16                   0x1021
#define HIGH 1
#define LOW 0
#define INPUT 1
#define OUTPUT 0
#define A5 5
#define A6 6
#define A7 7
#define PROGMEM 
#define F(x) x
#define SERIAL_RS485
#define SERIAL_8N1 0
#define INTERNAL4V096 1
#define INTERNAL2V048 2
#define INTERNAL1V024 3

#define PIN_A5  5
#define PIN_A6  6
#define PIN_A7  7
#define PIN_PA3 3
#define PIN_PA4 4
#define PIN_PA5 5
#define PIN_PA6 6
#define PIN_PA7 7
#define HEX 16


// CRC POLYNOME = x15 + 1 =  1000 0000 0000 0001 = 0x8001
uint16_t crc16(const uint8_t *array, uint16_t length, const uint16_t polynome,
               const uint16_t startmask, const uint16_t endmask, 
               const bool reverseIn, const bool reverseOut)
{
  uint16_t crc = startmask;
  while (length--) 
  {
    uint8_t data = *array++;
    crc ^= ((uint16_t)data) << 8;
    for (uint8_t i = 8; i; i--) 
    {
      if (crc & (1 << 15))
      {
        crc <<= 1;
        crc ^= polynome;
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  crc ^= endmask;
  return crc;
}

void pinMode(int pin, int mode) {

}
void delay(int millis) {
    usleep(millis*1000);
}
void delayMicroseconds(int us) {
    usleep(us);
}


unsigned long micros(){
    static unsigned long start = 0;
    struct timeval tv;
    gettimeofday(&tv,NULL);
    unsigned long time_in_micros = 1000000 * tv.tv_sec + tv.tv_usec;
    if (start == 0) {
        start = time_in_micros;
        return 0;
    } 
    return time_in_micros-start;
}

int16_t analogRead(uint8_t pin) {
    return 2045;
}

int32_t analogReadEnh(uint8_t pin, uint8_t bits=12,  uint8_t gain=0) {
    int16_t max = 1<<bits;
    return max/2;
} 
int32_t analogReadDiff(uint8_t pinA, uint8_t pinB, uint8_t bits=12,  uint8_t gain=0) {
    return 100;
} 
void analogReference(int ref) {

}
void analogSampleDuration(int delay) {

}
int16_t pgm_read_dword(const int16_t *intp) {
    return *intp;
}

uint8_t  pgm_read_byte_near(const uint8_t *ptr) {
    return *ptr;
}

#define DEBUG(x) std::cout << "Debug:" << x << std::endl




class SerialClass {
    public:
        SerialClass() {};
        void begin(uint32_t baud, uint16_t options) { this->baud = baud; this->options = options;};
        int available() {
            if ( pos > 0 ) {
                // dont read more while the buffer has data in it.
                std::cout << "Buffer has " << pos << std::endl;
                return pos;
            }

            std::cout << ">";
            std::string line;
            std::getline( std::cin, line );
            char * cline = (char *) line.c_str();
            if ( line == "exit") {
                exit(0);
            }
            

            size_t start = pos;
            if ( cline[0] == ':') {
                std::cout << "Modbus Ascii " << line << std::endl;
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
                std::cout << "Modbus Decimal " << line << std::endl;
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
                std::cout << "Noop " << line << std::endl;
                return pos;
            }

            // append the CRC.
            uint16_t crcVal = crc16((const uint8_t *) &readbuffer[start], pos-start, CRC16, 0, 0, false, false);
            printf("CRC adding %ld  for %ld as %X \n",start, pos-start, crcVal);
            readbuffer[pos++] = 0xff&(crcVal>>8);
            readbuffer[pos++] = 0xff&crcVal;


            std::cout << "Buffer now " << pos << " long" << std::endl;
            char opbuffer[10];
            for (int i = 0; i < pos; i++) {
                printf("%02X ", readbuffer[i]);
            }
            std::cout << std::endl;

            if ( cline[1] == '>') {
                int fails = 0;
                for(int i = 0; i < pos; i++) {
                    if(readbuffer[i] != writebuffer[i]) {
                        printf("Check failed pos %d %X != %X \n", i, readbuffer[i], writebuffer[i]);
                        fails++;
                    }
                }
                if ( fails == 0) {
                    printf("Check Ok\n");
                } else {
                    exit(1);
                }
                pos = 0;
                return 0;
            } else {
                return pos;
            }
            
        };
        size_t readBytes(uint8_t *buffer, size_t len) {    
            size_t l = std::min(pos, len);
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
            std::cout << ">:";
            for (int i = 0; i < len; ++i) {
                writebuffer[i] = buffer[i];
                printf("%02X ", buffer[i]);
            }
            std::cout << std::endl;
        };
        void flush() {

        };
        void printf(const char * fmt, ...) {
            va_list args;
            va_start(args,fmt);
            vprintf(fmt,args);
            va_end(args);
        };
        void print(const char * v) {
            printf("%s",v);
        };
        void print(int16_t  v) {
            printf("%d",v);
        };
        void print(int32_t  v) {
            printf("%ld",v);
        };
        void print(float  v) {
            printf("%f",v);
        };
        void println(float  v) {
            printf("%f\n",v);
        };
        void println(const char * v) {
            printf("%s\n",v);
        };
        void print(uint8_t b, uint8_t base) {
            if ( base == HEX ) {
                printf("%X",b);
            }
            printf("%d",b);
        };
    private:
        uint32_t baud;
        uint16_t options;
        uint8_t readbuffer[80];
        uint8_t writebuffer[80];
        size_t pos = 0;
};

SerialClass Serial = SerialClass();
SerialClass Serial1 = SerialClass();
#define UartClass SerialClass

class CommandLine {
    public:
        CommandLine(SerialClass * io): io{io} {};
        void checkCommand() {};
    private:
        SerialClass *io; 
};


byte eeprom[512];
void eeprom_write_block (const void *pointer_ram, void *pointer_eeprom, size_t n) {
    int eprom_address = (int)(0x01ff&(int64_t)pointer_eeprom);
    uint8_t * pram = (uint8_t *)pointer_ram;
    for (int i = 0; i < n; i++) {
        eeprom[eprom_address++] = *(pram++);
    }
}

void eeprom_update_block (const void *pointer_ram, void *pointer_eeprom, size_t n) {
    int eprom_address = (int)(0x01ff&(int64_t)pointer_eeprom);
    uint8_t * pram = (uint8_t *)pointer_ram;
    for (int i = 0; i < n; i++) {
        eeprom[eprom_address++] = *(pram++);
    }
}

void eeprom_read_block (void *pointer_ram, const void *pointer_eeprom, size_t n) {
    int eprom_address = (int)(0x01ff&(int64_t)pointer_eeprom);
    uint8_t * pram = (uint8_t *)pointer_ram;
    for (int i = 0; i < n; i++) {
        *(pram++) = eeprom[eprom_address++];
    }
}

uint8_t eeprom_read_byte(const uint8_t * pointer_eeprom) {
    int eprom_address = (int)(0x01ff&(int64_t)pointer_eeprom);
    return eeprom[eprom_address];

}


uint16_t eeprom_read_word(const uint16_t * pointer_eeprom) {
    int eprom_address = (int)(0x01ff&(int64_t)pointer_eeprom);
    return (0x00ff&eeprom[eprom_address]) | (0xff00&(eeprom[eprom_address+1]<<8));
}


extern void setup();
extern void loop();


int main(void) {

	std::cout << "Setup" << std::endl;
	setup();
	std::cout << "Setup Done" << std::endl;
    
	std::cout << "Looping" << std::endl;
	for (;;) {
		loop();

        usleep(200000);
	}
        
	return 0;
}


#endif