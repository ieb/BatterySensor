# Battery Sensor

Aim is to create a low power smat shunt batter sensor reporting volage, current and temperatire over ModbusRTU.

Shunt is measured using a MAX9918 shut amplifier amplifying 48x a 75mV/100A shunt scaled down to 50mV to keep the MAX9918 input optimal.
Temperature by a 10K NTC read by ADC using an interpolation table.
Voltage by direct ADC with a voltage divder
RS485 by MAX485

# Protocol

Modbus RTU - bigendian


# Holding registers - function 3 to read, 6 to set

Stored in EEPROM, which is typically reset on a firmware upgrade.

| Offset | Name                          | type  | details                                          |
|--------|-------------------------------|-------|--------------------------------------------------|
| 0      | Device Adddress               | int16 | 1-254 modbus address, default is 2               |
| 1      | Voltage offset adjustment     | int16 | Offset in 0.1mv steps eg 10 = 1mV                |
| 2      | Voltage scale adjustment      | int16 | scale in 1/10000th eg 10000 = 1x                 |
| 3      | Current offset adjustment     | int16 | offset in 0.1mA steps eg 10 = 1mA                |
| 4      | Current scale adjustment      | int16 | scale in 1/10000th eg 10000 = 1x                 |
| 5      | Temperature offset adjustment | int16 | offset in C in 1/1000th eg 1000 = +1C            |
| 6      | Temperature scale adjustment  | int16 | scale in C in 1/1000th eg 1000 = 1x              |
| 7      | Serial Number                 | int16 | Device serial number, once set cannot be changed |


# Input Registers - function 4 to read

| Offset | Name            | type   | details                                 |
|--------|-----------------|--------|-----------------------------------------|
| 0      | Voltage         | int16  | Units 0.01V                             |
| 1      | Current         | int16  | Units 0.01A                             |
| 2      | Temperature     | int16  | 1 = 0.01C                               |
| 1001   | Recieved        | uint16 | count of frames recieved                |
| 1002   | Sent            | uint16 | count off frames send                   |
| 1003   | Errors Recieved | uint16 | count off frames recieved with errors   |
| 1004   | Ignored         | uint16 | count off frames dropped                |
| 1005   | Errors Sent     | uint16 | count off error frames send             |


# Other functions
* 17 - report - 4 bytes, uint8 device type (0x01), uint device on (0xff), int16 serial number (0xXXXX)

Exceptions are reported as per ModbuRTU Spec.


# MCU

Platform ATTiny8224
Could have also used a ATTiny84, but this has no uarts and requires ICSP programming.
Programmming using jtag2udpi on an Arduino Uno with avrdude.conf setup for the ATTiny8224, running at 5V.

# Voltage

The ATTiny ADC in 16bit using 4096 refrence through aa 32K/10K divider, gives withing 10mv of a 4 digit multi meter after calibration setting (0.9954 register 2 set to 9954). Hard to tell which is more accutage, probably the ADC. Voltage is stable with the LSB changing +- 1 or 2.

# Current

Multiple frustrating attempts with a current amplifier, eventually abandoned.

The originam MAX9918 had an unstable 75mV offset from the reference resistors at 0A which would not go, chip might have been damaged during soldering or the EP ground plane was not good enough.

2x INA169 that I had from years ago could not be made to amplify the input correctly at the required 47x gain. Looking back I had tried INA169 before and had problems eventually abandoning.

Now trying the Attiny3224 ADC + PGA at 16x which measures down to 0.0019mV perbit below 64mV differntial on the 16bit range with a 1024mV refrence, however this might really be 0.015625mV resolution as the 16 bit is oversampling 12bit. If it is, this represents a resolution of +-20mA which is perfectly reasonable for a 100A/75mV shunt. Quick testing indicates this is stable and repeatable with 10R on the input lines for protection from misconnection and 100nF accross the ADC inputs to remove high frequency components.

If that fails I will use one of the Alegro Hall chips, althought I prefer the simplicity of a shunt.

# debugging

Performed over serial on the second uart at 115200 baud.
Modbus is at 9600 baud 8N1

Building with DEVMODE (see main.cpp) defined allows the rs485 serial to be diverted to Serial1 and test patterns to be pasted from testpatterns.txt over the monitoring serial line.

Testing can be done with 

            pio run -e native 

which produces a native binary then

            cat testpattern.txt |  ./.pio/build/native/program

Which checks exercises the RS485 protocol using test patterns checking the response.



# Programming

Currently using jtag2udpi, via an Uno, but setting fuses seems hard
Needs a modified avrdude.conf file to add support for the attiny3224
pio run -t fuse -e attiny3224 does appear to work.

Fuses must be set to ensure the chip clock speed matches the compiled code. This only needs to be done once per chip.
It its not done the serial output doesnt sync with the baud rate and garbage is seen on the serial line.

    pio run -t fuses -e attiny3224

Looking at megaTinyCore support in the Arduino IDE using SerialUDPI is much better and more advanced.

    pio run -t upload -e attiny3224 

works ok for the moment.

# Board

For full details see the pcb/*
In outline.

            ------------------------------------------
            | a           1 2 3 4 5                 A|
            | b   MAX                      MAX      B|     
            | c   RS485   Atting3224       9918     C|     
            | d                                     D|     
            |     AMS1117                           E|
            ------------------------------------------     

            a = GND
            b = Vin (12v)
            c = RS485 A
            d = RS485 B

            1 = GND
            2 = +5V
            3 = UDPI
            4 = Debug RX
            5 = Debug TX

            A = Battery +Ve
            B = Shunt -
            C = Shunt +
            D = GND
            E = NTC

# Test client

in testclient/ requires python3.
install with 

    pip install -r requirements.txt

run with 

    python3 batterymon.py

# TODO

* [x] Check MAX9918 amplifier - Abandoned using MAX9918 as it had 75mV offset from its reference point and a lot of instability. Have switched to a pair of INA196 current amplifiers which so far draw much less current and seem stable
* [x] Test temperature
* [x] Test voltage
* [x] Test current - switched to direct to ADC which gives good 0mA stability, need to verify linearit
* [x] Test RS485 
* [x] Write Python Modbus RTU Controller to test for real.
* [ ] Calibrate
* [ ] Integrate with CanDiagnose controler to act as ModBus Controller for 1..n battery sensors.