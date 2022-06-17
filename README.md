# Battery Sensor

Aim is to create a low power smat shunt batter sensor reporting volage, current and temperatire over ModbusRTU.

Shunt is provided by a 2x MAX9918 shut amplifiers for low and high range
Temperature by a 10K NTC read by ADC using an interpolation table.
Voltage by direct ADC with a voltage divder
RS485 by MAX485
Current read through a MAX9918 current amplifier.

# Protocol
Modbus RTU - bigendian

# Holding registers - function 3 to read, 6 to set

| Offset | Name                          | type  | details                                          |
|--------|-------------------------------|-------|--------------------------------------------------|
| 0      | Device Adddress               | int16 | 1-254 modbus address                             |
| 1      | Voltage offset adjustment     | int16 | ADC reading offset in bits                       |
| 2      | Voltage scale adjustment      | int16 | scale in 1/1000th eg 1000 = 1x                   |
| 3      | Current offset adjustment     | int16 | ADC reading offset in bits                       |
| 4      | Current scale adjustment      | int16 | scale in 1/1000th eg 1000 = 1x                   |
| 5      | Temperature offset adjustment | int16 | offset in C in 1/1000th eg 1000 = +1C            |
| 6      | Temperature scale adjustment  | int16 | scale in C in 1/1000th eg 1000 = 1x              |
| 7      | Serial Number                 | int16 | Device serial number, once set cannot be changed |


# Input Registers - function 4 to read

| Offset | Name        | type  | details     |
|--------|-------------|-------|-------------|
| 0      | Voltage     | int16 | Units 0.01V |
| 1      | Current     | int16 | Units 0.01A |
| 2      | Temperature | int16 | 1 = 0.01C   |

# Other functions
* 17 - report - 4 bytes, uint8 device type (0x01), uint device on (0xff), int16 serial number (0xXXXX)

Exceptions are reported as per ModbuRTU Spec.


# MCU
Platform ATTiny8224
Could have also used a ATTiny84, but this has no uarts and requires ICSP programming.
Programmming using jtag2udpi on an Ardiono with avrdude.conf setup for the ATTiny8224, running at 5V.

# Voltage

12V measured with a 22K/10K divider.
Current 
100A/75mA shunt
Through a 100K/200K divider to take 75mV to 50mV so that the ranve of the MAX9918 is not exceeded.
MAX9918 Gain  (1 + 47000/(1000)) = 48, FSD 0.05*(1 + 47000/(1000)) = 2.4V 

Max = 2.5 + 2.4 = 4.9
Min = 2.5 - 2.4 = 0.1

The 8224 has a PGA with gains of 1,2,4,8,16 all 12bit single ended.

1x 5V resolution 0.001220703125 mV
2x 2.5V 2.5/4096 0.0006103515625 mV
4x 1.25 1.25/4096 0.0003051757812 mV
8x 0.625 0.625/4096 0.0001525878906 mV
16x 0.3125 0.3125/4096 0.00007629394531 mV

The code will autoscale, so max resolution will be +-2 bits or 
0.00007629394531*5/48 at MAX9918 input
100/0.050*(0.00007629394531*5/48) at MAX9918 input
15mA.
3.1mA per bit.



# debugging

Performed over serial on the second uart at 115200 baud.
Modbus is at 9600 baud 8N1

Building with DEVMODE defined allows the rs485 serial to be diverted to Serial1 and test patterns to be 
pasted from testpatterns.txt over the monitoring serial line.


# Programming

Currently using jtag2udpi, via an Uno, but setting fuses seems hard
Needs a modified avrdude.conf file to add support for the attiny3224
pio run -t fuse -e attiny3224 does appear to work.
Have not tried changing the values of the fuses.
Looking at megaTinyCore support in the Arduino IDE using SerialUDPI is much better and more advanced.

    pio run -t upload -e attiny322 

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

            a = RS485 A
            b = RS485 B
            c = GND
            d = Vin (12v)

            1 = GND
            2 = +5V
            3 = Debug RX
            4 = Debug TX
            5 = UDPI

            A = Shunt -
            B = Shunt +
            C = Battery +Ve
            D = NTC
            E = GND

