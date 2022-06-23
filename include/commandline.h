#pragma once

// symbols from main.cpp so we dont have to get complicated.
extern int16_t readTemperature();
extern int16_t readVoltage();
extern int16_t readCurrent();
extern void loadDefaults();
extern uint16_t crc16(const uint8_t *array, uint16_t length);
extern uint8_t deviceAddress;
extern uint16_t framesRecieved;
extern uint16_t framesSent;
extern uint16_t framesErrorRecieved;
extern uint16_t framesIgnored;
extern uint16_t framesErrorSent;




class CommandLine {
    public:
        CommandLine(UartClass * io) : io{io} {};


        void checkCommand() {
            static bool enableForward = false;
            if (io->available()) {
                char chr = io->read();
                switch ( chr ) {
                    case 'h': showHelp(); break;
                    case 's': showStatus(); break;
                    case 't': readTemperature();  break;
                    case 'v': readVoltage();  break;
                    case 'c': readCurrent();  break;
                    case 'S': doSetup(); break;
                    case 'F': loadDefaults(); break;
                }
            }
        }
    private:
        UartClass * io;

        bool readInt(int16_t *v) {
            String line = io->readStringUntil("\n");
            line.trim();
            if ( line.length() > 0 ) {
                v = line.toInt();
                return true;
            }
            return false;
        };
        bool updateHoldingRegister(int8_t * registers, uint8_t offset) {
            int16_t value;
            if (readInt(&value)) {
                registers[offset] = 0xff&value;
                registers[offset+1] = 0xff&(value>>8);
                io->println(" - changed");
                return true;
            } else {
                io->println(" - no change");
                return false;
            }
        };
        int16_t asInt16(uint8_t * registers, uint8_t offset) {
            return registers[offset] | (0xff00&(registers[offset+1]<<8));
        };
        void doSetup() {
            showStatus();
            uint8_t epromContents[HR_SIZE];
            eeprom_read_block((void*)&epromContents[0], (const void*)0, HR_SIZE);
            io->println(F("Setup: Enter new values or return to leave unchanged."));
            io->print(F("Setup - device address "));io->print(epromContents[HR_DEVICE_ADDRESS]);io->print(F(" >"));
            bool changed = false;
            int16_t value;
            if (readInt(&value)) {
                if ( value > 0 && value < 255 ) {
                    epromContents[HR_DEVICE_ADDRESS] = value;
                    epromContents[HR_DEVICE_ADDRESS+1] = 0x00;
                    io->println(" - changed");
                    changed = true;
                } else {
                    io->println(" - invalid, no change");
                }
            } else {
                io->println(" - no change");
            }
            io->print(F("Setup - voltage offset "));io->print(asInt16(epromContents,HR_VOLTAGE_OFFSET));io->print(F(" >"));
            changed = updateHoldingRegister(epromContents, HR_VOLTAGE_OFFSET) || changed;
            io->print(F("Setup - voltage scale "));io->print(asInt16(epromContents,HR_VOLTAGE_SCALE));io->print(F(" >"));
            changed = updateHoldingRegister(epromContents, HR_VOLTAGE_SCALE) || changed;
            io->print(F("Setup - current offset "));io->print(asInt16(epromContents,HR_CURRENT_OFFSET));io->print(F(" >"));
            changed = updateHoldingRegister(epromContents, HR_CURRENT_OFFSET) || changed;
            io->print(F("Setup - current scale "));io->print(asInt16(epromContents,HR_CURRENT_SCALE));io->print(F(" >"));
            changed = updateHoldingRegister(epromContents, HR_CURRENT_SCALE) || changed;
            io->print(F("Setup - temperature offset "));io->print(asInt16(epromContents,HR_TEMPERATURE_OFFSET));io->print(F(" >"));
            changed = updateHoldingRegister(epromContents, HR_TEMPERATURE_OFFSET) || changed;
            io->print(F("Setup - temperature scale "));io->print(asInt16(epromContents,HR_TEMPERATURE_SCALE));io->print(F(" >"));
            changed = updateHoldingRegister(epromContents, HR_TEMPERATURE_SCALE) || changed;
            io->print(F("Setup - serial number "));io->print(asInt16(epromContents,HR_SERIAL_NUMBER));io->print(F(" >"));
            changed = updateHoldingRegister(epromContents, HR_SERIAL_NUMBER) || changed;
            if ( changed ) {
                // update CRC and save.
                uint16_t crcv =  crc16(&epromContents[0], HR_SIZE-2);
                epromContents[HR_CRC] = 0xff&(crcv);
                epromContents[HR_CRC+1] = 0xff&(crcv>>8); // little endian
                eeprom_update_block((const void*)&epromContents[0], (void*)0, HR_SIZE);
                // update device address, which may have changed.
                deviceAddress = epromContents[HR_DEVICE_ADDRESS];
            }
        };


        void showHelp() {
            io->println(F("Battery Monitor - key presses"));
            io->println(F("  - 'h' or '?' to show this message"));
            io->println(F("  - 's' show status"));
            io->println(F("  - 't' read temperature"));
            io->println(F("  - 'v' read voltage"));
            io->println(F("  - 'c' read current"));
            io->println(F("  - 'S' setup"));
            io->println(F("  - 'F' factory reset"));
        };
        
        void showStatus() {

            io->print(F("Status\nDevice Address ;"));
            io->println(eeprom_read_byte(HR_DEVICE_ADDRESS), HEX);
            io->print(F("Voltage Offset :"));
            io->println((int16_t)eeprom_read_word((const uint16_t *)HR_VOLTAGE_OFFSET));
            io->print(F("Voltage Scale  :"));
            io->println((int16_t)eeprom_read_word((const uint16_t *)HR_VOLTAGE_SCALE));
            io->print(F("Current Offset :"));
            io->println((int16_t)eeprom_read_word((const uint16_t *)HR_CURRENT_OFFSET));
            io->print(F("Current Offset :"));
            io->println((int16_t)eeprom_read_word((const uint16_t *)HR_CURRENT_SCALE));
            io->print(F("Temp Offset    :"));
            io->println((int16_t)eeprom_read_word((const uint16_t *)HR_TEMPERATURE_OFFSET));
            io->print(F("Temp Scale     :"));
            io->println((int16_t)eeprom_read_word((const uint16_t *)HR_TEMPERATURE_SCALE));
            io->print(F("Serial Number  :"));
            io->println((int16_t)eeprom_read_word((const uint16_t *)HR_SERIAL_NUMBER));


            io->print(F("Frames Recieved:"));
            io->print(framesRecieved);
            io->print(F(" Sent:"));
            io->print(framesSent);
            io->print(F(" Ignored:"));
            io->print(framesErrorRecieved);
            io->print(F(" Ignored:"));
            io->print(framesErrorSent);
            io->print(F(" Ignored:"));
            io->println(framesIgnored);
            io->print((F("MCU V=")));
            io->print(readMCUVoltage());
            io->print((F("mV T=")));
            int32_t t = readMCUTemperature();
            t -= 273;
            io->print(t);
            io->println((F("C")));

        };

        uint16_t readMCUVoltage() {
            analogReference(INTERNAL1V024);
            delayMicroseconds(100);
            int32_t vddmeasure = analogReadEnh(ADC_VDDDIV10, 12); // Take it at 12 bits
            vddmeasure *= 10; // since we measured 1/10th VDD
            int16_t returnval = vddmeasure >> 2; // divide by 4 to get into millivolts.
            if (vddmeasure & 0x02) {
                // if last two digits were 0b11 or 0b10 we should round up
                returnval++;
            }
            return returnval;
        }

        uint16_t readMCUTemperature() {
            int8_t sigrowOffset = SIGROW.TEMPSENSE1;
            uint8_t sigrowGain = SIGROW.TEMPSENSE0;
            analogSampleDuration(128); // must be >= 32µs * f_CLK_ADC per datasheet 30.3.3.7
            analogReference(INTERNAL1V024);
            uint32_t reading = analogRead(ADC_TEMPERATURE);
            reading -= sigrowOffset;
            reading *= sigrowGain;
            reading += 0x80; // Add 1/2 to get correct rounding on division below
            reading >>= 8; // Divide result to get Kelvin
            return reading;

        }

};


