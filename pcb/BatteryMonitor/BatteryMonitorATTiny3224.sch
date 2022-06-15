EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L SP3485EN IC101
U 1 1 6293EE38
P 5250 4750
F 0 "IC101" H 5050 5000 40  0000 C CNN
F 1 "MAX3485E" H 5450 4500 40  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 5250 4750 35  0000 C CIN
F 3 "" H 5250 4750 60  0000 C CNN
	1    5250 4750
	1    0    0    -1  
$EndComp
$Comp
L R R109
U 1 1 6293EFF6
P 6050 2950
F 0 "R109" V 6130 2950 40  0000 C CNN
F 1 "100K" V 6057 2951 40  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 5980 2950 30  0001 C CNN
F 3 "" H 6050 2950 30  0000 C CNN
	1    6050 2950
	1    0    0    -1  
$EndComp
$Comp
L R R110
U 1 1 6293F02A
P 6050 3600
F 0 "R110" V 6130 3600 40  0000 C CNN
F 1 "100K" V 6057 3601 40  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 5980 3600 30  0001 C CNN
F 3 "" H 6050 3600 30  0000 C CNN
	1    6050 3600
	1    0    0    -1  
$EndComp
$Comp
L MAX9918 U103
U 1 1 6293F377
P 4550 3350
F 0 "U103" H 4550 3500 60  0000 C CNN
F 1 "MAX9918" H 4550 3350 60  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 4550 3350 60  0001 C CNN
F 3 "" H 4550 3350 60  0001 C CNN
	1    4550 3350
	1    0    0    -1  
$EndComp
Text GLabel 5150 3150 2    60   Input ~ 0
5v
Text GLabel 4850 3800 0    60   Output ~ 0
CurSenseV
$Comp
L R R106
U 1 1 6293F511
P 5300 3800
F 0 "R106" V 5380 3800 40  0000 C CNN
F 1 "100K" V 5307 3801 40  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 5230 3800 30  0001 C CNN
F 3 "" H 5300 3800 30  0000 C CNN
	1    5300 3800
	-1   0    0    1   
$EndComp
$Comp
L R R108
U 1 1 6293F5E6
P 5750 3800
F 0 "R108" V 5830 3800 40  0000 C CNN
F 1 "100R" V 5757 3801 40  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 5680 3800 30  0001 C CNN
F 3 "" H 5750 3800 30  0000 C CNN
	1    5750 3800
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR01
U 1 1 6293F711
P 6050 4000
F 0 "#PWR01" H 6050 4000 30  0001 C CNN
F 1 "GND" H 6050 3930 30  0001 C CNN
F 2 "" H 6050 4000 60  0001 C CNN
F 3 "" H 6050 4000 60  0001 C CNN
	1    6050 4000
	1    0    0    -1  
$EndComp
$Comp
L C C105
U 1 1 6293F7A8
P 2650 1050
F 0 "C105" H 2650 1150 40  0000 L CNN
F 1 "100nF" H 2656 965 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 2688 900 30  0001 C CNN
F 3 "" H 2650 1050 60  0000 C CNN
	1    2650 1050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 6293F7F7
P 2000 1350
F 0 "#PWR02" H 2000 1350 30  0001 C CNN
F 1 "GND" H 2000 1280 30  0001 C CNN
F 2 "" H 2000 1350 60  0001 C CNN
F 3 "" H 2000 1350 60  0001 C CNN
	1    2000 1350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 6293F866
P 4000 3650
F 0 "#PWR03" H 4000 3650 30  0001 C CNN
F 1 "GND" H 4000 3580 30  0001 C CNN
F 2 "" H 4000 3650 60  0001 C CNN
F 3 "" H 4000 3650 60  0001 C CNN
	1    4000 3650
	1    0    0    -1  
$EndComp
Text GLabel 2750 3100 0    60   Input ~ 0
RS+
Text GLabel 2800 5000 0    60   Input ~ 0
RS-
Text GLabel 5400 4350 2    60   Input ~ 0
5v
$Comp
L GND #PWR04
U 1 1 6293FDCD
P 5250 5250
F 0 "#PWR04" H 5250 5250 30  0001 C CNN
F 1 "GND" H 5250 5180 30  0001 C CNN
F 2 "" H 5250 5250 60  0001 C CNN
F 3 "" H 5250 5250 60  0001 C CNN
	1    5250 5250
	1    0    0    -1  
$EndComp
Text GLabel 5800 4650 2    60   BiDi ~ 0
RS485-A
Text GLabel 5800 4850 2    60   BiDi ~ 0
RS485-B
$Comp
L C C106
U 1 1 6293FF59
P 2900 1050
F 0 "C106" H 2900 1150 40  0000 L CNN
F 1 "100nF" H 2906 965 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 2938 900 30  0001 C CNN
F 3 "" H 2900 1050 60  0000 C CNN
	1    2900 1050
	1    0    0    -1  
$EndComp
Text GLabel 4650 4750 0    60   Input ~ 0
TXEN
$Comp
L R R102
U 1 1 62940901
P 3950 6000
F 0 "R102" V 4030 6000 40  0000 C CNN
F 1 "22K" V 3957 6001 40  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 3880 6000 30  0001 C CNN
F 3 "" H 3950 6000 30  0000 C CNN
	1    3950 6000
	1    0    0    -1  
$EndComp
$Comp
L R R103
U 1 1 62940976
P 3950 6650
F 0 "R103" V 4030 6650 40  0000 C CNN
F 1 "10K" V 3957 6651 40  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 3880 6650 30  0001 C CNN
F 3 "" H 3950 6650 30  0000 C CNN
	1    3950 6650
	1    0    0    -1  
$EndComp
Text GLabel 4100 6300 2    60   Output ~ 0
BatSenseV
$Comp
L C C108
U 1 1 62940AA3
P 3700 6650
F 0 "C108" H 3700 6750 40  0000 L CNN
F 1 "C" H 3706 6565 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 3738 6500 30  0001 C CNN
F 3 "" H 3700 6650 60  0000 C CNN
	1    3700 6650
	1    0    0    -1  
$EndComp
Text GLabel 3750 5650 0    60   Input ~ 0
BatV+
$Comp
L GND #PWR05
U 1 1 62940FA6
P 3950 7250
F 0 "#PWR05" H 3950 7250 30  0001 C CNN
F 1 "GND" H 3950 7180 30  0001 C CNN
F 2 "" H 3950 7250 60  0001 C CNN
F 3 "" H 3950 7250 60  0001 C CNN
	1    3950 7250
	1    0    0    -1  
$EndComp
$Comp
L CONN_4 P101
U 1 1 62941286
P 1750 2350
F 0 "P101" V 1700 2350 50  0000 C CNN
F 1 "RS485" V 1800 2350 50  0000 C CNN
F 2 "Connect:bornier4" H 1750 2350 60  0001 C CNN
F 3 "" H 1750 2350 60  0001 C CNN
	1    1750 2350
	-1   0    0    1   
$EndComp
Text GLabel 2250 2500 2    60   BiDi ~ 0
RS485-A
Text GLabel 2250 2400 2    60   BiDi ~ 0
RS485-B
Text GLabel 2300 2200 2    60   Input ~ 0
Vin
$Comp
L GND #PWR06
U 1 1 629414AD
P 2900 2400
F 0 "#PWR06" H 2900 2400 30  0001 C CNN
F 1 "GND" H 2900 2330 30  0001 C CNN
F 2 "" H 2900 2400 60  0001 C CNN
F 3 "" H 2900 2400 60  0001 C CNN
	1    2900 2400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR07
U 1 1 62941584
P 1450 3900
F 0 "#PWR07" H 1450 3900 30  0001 C CNN
F 1 "GND" H 1450 3830 30  0001 C CNN
F 2 "" H 1450 3900 60  0001 C CNN
F 3 "" H 1450 3900 60  0001 C CNN
	1    1450 3900
	1    0    0    -1  
$EndComp
Text GLabel 1400 3650 2    60   Output ~ 0
BatV+
Text GLabel 1400 3750 2    60   BiDi ~ 0
NTC
Text GLabel 1400 3550 2    60   Output ~ 0
RS+
Text GLabel 1400 3450 2    60   Output ~ 0
RS-
$Comp
L R R104
U 1 1 629446B1
P 5300 6000
F 0 "R104" V 5380 6000 40  0000 C CNN
F 1 "10K" V 5307 6001 40  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 5230 6000 30  0001 C CNN
F 3 "" H 5300 6000 30  0000 C CNN
	1    5300 6000
	1    0    0    -1  
$EndComp
Text GLabel 5100 5700 0    60   Input ~ 0
5v
Text GLabel 5500 6350 2    60   BiDi ~ 0
NTC
Text GLabel 4150 4600 0    60   Output ~ 0
RX
Text GLabel 4150 4900 0    60   Input ~ 0
TX
Text GLabel 6250 2650 2    60   Input ~ 0
5v
$Comp
L NCP1117ST50T3G U101
U 1 1 629BA07D
P 1500 750
F 0 "U101" H 1650 554 40  0000 C CNN
F 1 "AMS1117" H 1500 950 40  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-223-3Lead_TabPin2" H 1500 750 60  0001 C CNN
F 3 "" H 1500 750 60  0000 C CNN
	1    1500 750 
	1    0    0    -1  
$EndComp
Text GLabel 750  700  0    60   Input ~ 0
Vin
$Comp
L GND #PWR08
U 1 1 629BA2E3
P 1500 1100
F 0 "#PWR08" H 1500 1100 30  0001 C CNN
F 1 "GND" H 1500 1030 30  0001 C CNN
F 2 "" H 1500 1100 60  0001 C CNN
F 3 "" H 1500 1100 60  0001 C CNN
	1    1500 1100
	1    0    0    -1  
$EndComp
Text GLabel 3350 700  2    60   Output ~ 0
5v
$Comp
L C C101
U 1 1 629BA5E0
P 950 1000
F 0 "C101" H 950 1100 40  0000 L CNN
F 1 "10uF" H 956 915 40  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 988 850 30  0001 C CNN
F 3 "" H 950 1000 60  0000 C CNN
	1    950  1000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR09
U 1 1 629BA92E
P 950 1300
F 0 "#PWR09" H 950 1300 30  0001 C CNN
F 1 "GND" H 950 1230 30  0001 C CNN
F 2 "" H 950 1300 60  0001 C CNN
F 3 "" H 950 1300 60  0001 C CNN
	1    950  1300
	1    0    0    -1  
$EndComp
$Comp
L C C104
U 1 1 629BB261
P 2000 1050
F 0 "C104" H 2000 1150 40  0000 L CNN
F 1 "10uF" H 2006 965 40  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 2038 900 30  0001 C CNN
F 3 "" H 2000 1050 60  0000 C CNN
	1    2000 1050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR010
U 1 1 629BB58E
P 2650 1350
F 0 "#PWR010" H 2650 1350 30  0001 C CNN
F 1 "GND" H 2650 1280 30  0001 C CNN
F 2 "" H 2650 1350 60  0001 C CNN
F 3 "" H 2650 1350 60  0001 C CNN
	1    2650 1350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR011
U 1 1 629BB5D8
P 2900 1350
F 0 "#PWR011" H 2900 1350 30  0001 C CNN
F 1 "GND" H 2900 1280 30  0001 C CNN
F 2 "" H 2900 1350 60  0001 C CNN
F 3 "" H 2900 1350 60  0001 C CNN
	1    2900 1350
	1    0    0    -1  
$EndComp
Text GLabel 1500 4500 2    60   BiDi ~ 0
UDPI
Text GLabel 1500 4800 2    60   Output ~ 0
5v
$Comp
L GND #PWR012
U 1 1 629CCA52
P 1500 5050
F 0 "#PWR012" H 1500 5050 30  0001 C CNN
F 1 "GND" H 1500 4980 30  0001 C CNN
F 2 "" H 1500 5050 60  0001 C CNN
F 3 "" H 1500 5050 60  0001 C CNN
	1    1500 5050
	1    0    0    -1  
$EndComp
$Comp
L C C107
U 1 1 629CD333
P 3150 1050
F 0 "C107" H 3150 1150 40  0000 L CNN
F 1 "1nF" H 3156 965 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 3188 900 30  0001 C CNN
F 3 "" H 3150 1050 60  0000 C CNN
	1    3150 1050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 629CD46D
P 3150 1350
F 0 "#PWR013" H 3150 1350 30  0001 C CNN
F 1 "GND" H 3150 1280 30  0001 C CNN
F 2 "" H 3150 1350 60  0001 C CNN
F 3 "" H 3150 1350 60  0001 C CNN
	1    3150 1350
	1    0    0    -1  
$EndComp
Text Notes 2950 1550 0    60   ~ 0
CPU
Text Notes 1650 1550 0    60   ~ 0
Other Chips as required
$Comp
L ATTINY3224-SS IC102
U 1 1 629CEC36
P 8800 3800
F 0 "IC102" H 8050 4550 60  0000 C CNN
F 1 "ATTINY3224-SS" H 9350 3050 60  0000 C CNN
F 2 "Housings_SOIC:SOIC-14_3.9x8.7mm_Pitch1.27mm" H 8100 3050 60  0001 C CNN
F 3 "" H 8800 3800 60  0001 C CNN
	1    8800 3800
	1    0    0    -1  
$EndComp
Text GLabel 7550 3200 0    60   Input ~ 0
5v
$Comp
L GND #PWR014
U 1 1 629CEEE7
P 10200 3250
F 0 "#PWR014" H 10200 3250 30  0001 C CNN
F 1 "GND" H 10200 3180 30  0001 C CNN
F 2 "" H 10200 3250 60  0001 C CNN
F 3 "" H 10200 3250 60  0001 C CNN
	1    10200 3250
	1    0    0    -1  
$EndComp
Text GLabel 7500 3800 0    60   Input ~ 0
CurSenseV
Text GLabel 7500 4100 0    60   Output ~ 0
TX
Text GLabel 7500 3950 0    60   Input ~ 0
RX
Text GLabel 7550 3350 0    60   Output ~ 0
TXEN
Text GLabel 7500 3650 0    60   Input ~ 0
BatSenseV
Text GLabel 7500 3500 0    60   BiDi ~ 0
NTC
Text GLabel 9950 3500 2    60   Input ~ 0
RXDebug
Text GLabel 9950 3650 2    60   Output ~ 0
TXDebug
Text GLabel 10000 3800 2    60   BiDi ~ 0
UDPI
Text GLabel 10600 4700 2    60   BiDi ~ 0
SDA
Text GLabel 10600 4550 2    60   BiDi ~ 0
SCL
Text GLabel 10600 4850 2    60   BiDi ~ 0
SCL
$Comp
L CONN_5 P102
U 1 1 629D28E4
P 850 3650
F 0 "P102" V 800 3650 50  0000 C CNN
F 1 "Bat" V 900 3650 50  0000 C CNN
F 2 "Connect:bornier5" H 850 3650 60  0001 C CNN
F 3 "" H 850 3650 60  0001 C CNN
	1    850  3650
	-1   0    0    1   
$EndComp
Text GLabel 1500 4700 2    60   Output ~ 0
RXDebug
Text GLabel 1500 4600 2    60   Input ~ 0
TXDebug
$Comp
L CONN_5 P103
U 1 1 629DAFCB
P 900 4700
F 0 "P103" V 850 4700 50  0000 C CNN
F 1 "Prog" V 950 4700 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x05" H 900 4700 60  0001 C CNN
F 3 "" H 900 4700 60  0001 C CNN
	1    900  4700
	-1   0    0    1   
$EndComp
NoConn ~ 9950 4100
NoConn ~ 9950 3950
NoConn ~ 9950 3350
Wire Wire Line
	5000 3250 6050 3250
Wire Wire Line
	6050 3200 6050 3350
Connection ~ 6050 3250
Wire Wire Line
	6050 3850 6050 4000
Wire Wire Line
	6250 2650 6050 2650
Wire Wire Line
	6050 2650 6050 2700
Wire Wire Line
	4000 3450 4000 3650
Wire Wire Line
	4000 3450 4100 3450
Wire Wire Line
	4100 3550 4000 3550
Connection ~ 4000 3550
Wire Wire Line
	3800 3250 4100 3250
Wire Wire Line
	3800 3150 4100 3150
Wire Wire Line
	5250 5250 5250 5100
Wire Wire Line
	5800 4650 5650 4650
Wire Wire Line
	5650 4850 5800 4850
Wire Wire Line
	4850 4700 4750 4700
Wire Wire Line
	4750 4700 4750 4800
Wire Wire Line
	4750 4800 4850 4800
Wire Wire Line
	4650 4750 4750 4750
Connection ~ 4750 4750
Wire Wire Line
	3950 6250 3950 6400
Wire Wire Line
	3700 6300 4100 6300
Connection ~ 3950 6300
Wire Wire Line
	3700 6300 3700 6450
Wire Wire Line
	3700 6850 3700 7100
Wire Wire Line
	3700 7100 3950 7100
Wire Wire Line
	3950 6900 3950 7250
Wire Wire Line
	3750 5650 3950 5650
Wire Wire Line
	3950 5650 3950 5750
Connection ~ 3950 7100
Wire Wire Line
	2300 2200 2100 2200
Wire Wire Line
	2100 2300 2900 2300
Wire Wire Line
	2100 2400 2250 2400
Wire Wire Line
	2100 2500 2250 2500
Wire Wire Line
	5100 5700 5300 5700
Wire Wire Line
	5300 5700 5300 5750
Wire Wire Line
	5500 6350 5300 6350
Wire Wire Line
	5300 6350 5300 6250
Wire Wire Line
	4150 4600 4850 4600
Wire Wire Line
	4150 4900 4850 4900
Wire Wire Line
	1500 1000 1500 1100
Wire Wire Line
	750  700  1100 700 
Wire Wire Line
	1900 700  3350 700 
Wire Wire Line
	950  800  950  700 
Connection ~ 950  700 
Wire Wire Line
	950  1200 950  1300
Wire Wire Line
	5000 3150 5150 3150
Wire Wire Line
	5400 4350 5250 4350
Wire Wire Line
	5250 4350 5250 4400
Wire Wire Line
	2350 850  2350 700 
Connection ~ 2350 700 
Wire Wire Line
	2650 850  2650 700 
Connection ~ 2650 700 
Wire Wire Line
	2900 850  2900 700 
Connection ~ 2900 700 
Wire Wire Line
	2350 1250 2350 1400
Wire Wire Line
	2650 1250 2650 1350
Wire Wire Line
	2900 1250 2900 1350
Wire Wire Line
	3150 700  3150 850 
Wire Wire Line
	3150 1250 3150 1350
Connection ~ 3150 700 
Wire Wire Line
	2900 2300 2900 2400
Wire Wire Line
	5050 3550 5050 4100
Wire Wire Line
	5050 3800 4850 3800
Wire Wire Line
	9950 3650 9850 3650
Wire Wire Line
	9850 3500 9950 3500
Wire Wire Line
	10200 3250 10200 3200
Wire Wire Line
	10200 3200 9850 3200
Wire Wire Line
	7750 3200 7550 3200
Wire Wire Line
	7750 3350 7550 3350
Wire Wire Line
	7750 3500 7500 3500
Wire Wire Line
	7500 3650 7750 3650
Wire Wire Line
	7500 3800 7750 3800
Wire Wire Line
	7500 3950 7750 3950
Wire Wire Line
	7750 4100 7500 4100
Wire Wire Line
	10000 3800 9850 3800
Wire Wire Line
	9950 3350 9850 3350
Wire Wire Line
	9950 3950 9850 3950
Wire Wire Line
	9950 4100 9850 4100
Wire Wire Line
	1450 3900 1450 3850
Wire Wire Line
	1450 3850 1250 3850
Wire Wire Line
	1400 3450 1250 3450
Wire Wire Line
	1400 3550 1250 3550
Wire Wire Line
	1250 3650 1400 3650
Wire Wire Line
	1250 3750 1400 3750
Wire Wire Line
	1500 4500 1300 4500
Wire Wire Line
	1500 4600 1300 4600
Wire Wire Line
	1500 4700 1300 4700
Wire Wire Line
	1500 4800 1300 4800
Wire Wire Line
	1300 4900 1500 4900
Wire Wire Line
	1500 4900 1500 5050
Wire Wire Line
	2000 850  2000 700 
Connection ~ 2000 700 
Wire Wire Line
	2000 1350 2000 1250
$Comp
L C C102
U 1 1 629DCB62
P 2350 1050
F 0 "C102" H 2350 1150 40  0000 L CNN
F 1 "100nF" H 2356 965 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 2388 900 30  0001 C CNN
F 3 "" H 2350 1050 60  0000 C CNN
	1    2350 1050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR015
U 1 1 629DCBB3
P 2350 1400
F 0 "#PWR015" H 2350 1400 30  0001 C CNN
F 1 "GND" H 2350 1330 30  0001 C CNN
F 2 "" H 2350 1400 60  0001 C CNN
F 3 "" H 2350 1400 60  0001 C CNN
	1    2350 1400
	1    0    0    -1  
$EndComp
$Comp
L R R101
U 1 1 629E1937
P 3250 3450
F 0 "R101" V 3330 3450 40  0000 C CNN
F 1 "100K" V 3257 3451 40  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 3180 3450 30  0001 C CNN
F 3 "" H 3250 3450 30  0000 C CNN
	1    3250 3450
	-1   0    0    1   
$EndComp
$Comp
L R R105
U 1 1 629E19AE
P 3250 4050
F 0 "R105" V 3330 4050 40  0000 C CNN
F 1 "100K" V 3257 4051 40  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 3180 4050 30  0001 C CNN
F 3 "" H 3250 4050 30  0000 C CNN
	1    3250 4050
	-1   0    0    1   
$EndComp
$Comp
L R R107
U 1 1 629E1A24
P 3250 4650
F 0 "R107" V 3330 4650 40  0000 C CNN
F 1 "100K" V 3257 4651 40  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 3180 4650 30  0001 C CNN
F 3 "" H 3250 4650 30  0000 C CNN
	1    3250 4650
	-1   0    0    1   
$EndComp
Wire Wire Line
	2800 5000 3250 5000
Wire Wire Line
	3250 5000 3250 4900
Wire Wire Line
	3250 4300 3250 4400
Wire Wire Line
	3250 3200 3250 3100
Wire Wire Line
	2750 3100 3800 3100
Wire Wire Line
	3800 3100 3800 3150
Connection ~ 3250 3100
Wire Wire Line
	3800 3250 3800 4350
Wire Wire Line
	3250 3700 3250 3800
Wire Wire Line
	3800 4350 3250 4350
Connection ~ 3250 4350
Text Notes 3300 3800 0    60   ~ 0
50mV FSD
Text Notes 2250 3900 0    60   ~ 0
75mV == 100A
Wire Wire Line
	5050 4100 5300 4100
Wire Wire Line
	5300 4100 5300 4050
Connection ~ 5050 3800
Wire Wire Line
	5050 3550 5000 3550
Wire Wire Line
	5300 3550 5300 3450
Wire Wire Line
	5000 3450 5550 3450
$Comp
L R R111
U 1 1 629E264E
P 5550 3800
F 0 "R111" V 5630 3800 40  0000 C CNN
F 1 "1K" V 5557 3801 40  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 5480 3800 30  0001 C CNN
F 3 "" H 5550 3800 30  0000 C CNN
	1    5550 3800
	-1   0    0    1   
$EndComp
Wire Wire Line
	5750 4050 5750 4100
Wire Wire Line
	5750 4100 5550 4100
Wire Wire Line
	5550 4100 5550 4050
Wire Wire Line
	5750 3250 5750 3550
Connection ~ 5750 3250
Wire Wire Line
	5550 3450 5550 3550
Connection ~ 5300 3450
$Comp
L C C103
U 1 1 629E472E
P 3500 3700
F 0 "C103" H 3500 3800 40  0000 L CNN
F 1 "10nF" H 3506 3615 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 3538 3550 30  0001 C CNN
F 3 "" H 3500 3700 60  0000 C CNN
	1    3500 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 3900 3500 4350
Connection ~ 3500 4350
Wire Wire Line
	3500 3500 3500 3100
Connection ~ 3500 3100
$EndSCHEMATC
