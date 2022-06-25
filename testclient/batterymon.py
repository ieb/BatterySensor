#!/usr/bin/env python3

import os
import sys
import struct
from configparser import RawConfigParser
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
from pymodbus.exceptions import ModbusIOException

settings = RawConfigParser()
settings.read(os.path.dirname(os.path.realpath(__file__)) + '/batterymon.cfg')

port = settings.get('query', 'port', fallback='/dev/cu.wchusbserial26210')
unit = int(settings.get('battery.main', 'unit', fallback=2))
print("Using ", port, "unit", unit)
print('Setup Serial Connection... ', end='')
client = ModbusClient(method='rtu', port=port, baudrate=9600, stopbits=1, parity='N', bytesize=8, timeout=1)
client.connect()
print('Done!')

def toAscii(registers):
    b = bytearray(len(registers)*2)
    ashex = list()
    i = 0
    for x in registers:
        ashex.append('{:02x}'.format(x))
        struct.pack_into("!H",b,i,x)
        i+=2
    #print(ashex)
    #print(len(registers), b)
    return b.decode('ascii')

def dumpAsHex(registers):
    ashex = list()
    for x in registers:
        ashex.append('{:02x}'.format(x))
    print(ashex)

def packAscii(value):
    registers = list()
    abytes = value.encode('ascii')
    print(abytes)
    i = 0
    v = 0
    for b in abytes:
        v = v*256
        v = v+b
        if i == 1:
            registers.append(v)
            i = 0
            v = 0
        else:
            i += 1
    if i == 1:
        v = v*256
        registers.append(v)
    return registers
        

def dumpAsAscii(registers):
    b = bytearray(len(registers)*2)
    i = 0
    for x in registers:
        struct.pack_into("!H",b,i,x)
        i+=2
    print(len(registers), b)
    print(b.decode('ascii'))


# get the serial number
row = client.read_holding_registers(0, count=1, unit=unit)
print("Exception", row)
print("Device address:", row.registers[0])
row = client.read_holding_registers(1, count=1, unit=unit)
print("Voltage Offset:", 0.1*row.registers[0], "mV")
row = client.read_holding_registers(2, count=1, unit=unit)
print("Voltage Scale:", 0.0001*row.registers[0])
row = client.read_holding_registers(3, count=1, unit=unit)
print("Current Offset:", 0.1*row.registers[0], "mA")
row = client.read_holding_registers(4, count=1, unit=unit)
print("Current Scale:", 0.0001*row.registers[0])
row = client.read_holding_registers(5, count=1, unit=unit)
print("Temperature Offset:", 0.001*row.registers[0])
row = client.read_holding_registers(6, count=1, unit=unit)
print("Temperature Scale:", 0.001*row.registers[0])
row = client.read_holding_registers(7, count=1, unit=unit)
print("Serial Number:", row.registers[0])


row = client.read_input_registers(1, count=1, unit=unit)
print("Voltage:", 0.01*row.registers[0], "V")
row = client.read_input_registers(2, count=1, unit=unit)
print("Current:", 0.01*row.registers[0], "A")
row = client.read_input_registers(3, count=1, unit=unit)
print("Temperature:", 0.01*row.registers[0], "C")



