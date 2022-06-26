#!/usr/bin/env python3

import os
import sys
import struct
from configparser import RawConfigParser
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
from pymodbus.exceptions import ModbusIOException
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder

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

def toInt(row):
    decoder = BinaryPayloadDecoder.fromRegisters(row.registers, Endian.Big, wordorder=Endian.Little)
    return decoder.decode_16bit_int()


# get the serial number
print("Holding Registers")
row = client.read_holding_registers(0, count=1, unit=unit)
print("          Device address (0000):", row.registers[0])
row = client.read_holding_registers(1, count=1, unit=unit)
print("          Voltage Offset (0001):", 0.1*toInt(row), "mV")
row = client.read_holding_registers(2, count=1, unit=unit)
print("           Voltage Scale (0002):", 0.0001*toInt(row))
row = client.read_holding_registers(3, count=1, unit=unit)
print("          Current Offset (0003):", 0.1*toInt(row), "mA")
row = client.read_holding_registers(4, count=1, unit=unit)
print("           Current Scale (0004):", 0.0001*toInt(row))
row = client.read_holding_registers(5, count=1, unit=unit)
print("      Temperature Offset (0005):", 0.001*toInt(row))
row = client.read_holding_registers(6, count=1, unit=unit)
print("       Temperature Scale (0006):", 0.001*toInt(row))
row = client.read_holding_registers(7, count=1, unit=unit)
print("          Serial Number  (0007):", row.registers[0])


print("Input Registers")

row = client.read_input_registers(0, count=1, unit=unit)
print("                 Voltage (0000):", 0.01*toInt(row), "V")
row = client.read_input_registers(1, count=1, unit=unit)
print("                 Current (0001):", 0.01*toInt(row), "A")
row = client.read_input_registers(2, count=1, unit=unit)
print("             Temperature (0002):", 0.01*toInt(row), "C")

# Modbus stats
print("Modbus Input Registers")
row = client.read_input_registers(1000, count=1, unit=unit)
print("                Recieved (1000):", row.registers[0])
row = client.read_input_registers(1001, count=1, unit=unit)
print("                    Sent (1001):", row.registers[0])
row = client.read_input_registers(1002, count=1, unit=unit)
print("         Errors Recieved (1002):", row.registers[0])
row = client.read_input_registers(1003, count=1, unit=unit)
print("                 Ignored (1003):", row.registers[0])
row = client.read_input_registers(1004, count=1, unit=unit)
print("             Errors Sent (1004):", row.registers[0])
row = client.read_input_registers(1005, count=1, unit=unit)
print("         Buffer Overflow (1005):", row.registers[0])




