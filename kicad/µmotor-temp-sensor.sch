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
LIBS:stm32
LIBS:DRV8870
LIBS:INA180A
LIBS:AS5045B
LIBS:TCAN33x
LIBS:mic4606
LIBS:drv8323s
LIBS:FDMD82xx
LIBS:ncp4623
LIBS:TMP006
LIBS:µmotor-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 6 6
Title "µmotor - Micro Motor Controller"
Date "2017-09-14"
Rev "rev1"
Comp "Roboterclub Aachen e.V."
Comment1 "Raphael Lehmann"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 5400 2100 2    60   BiDi ~ 0
SDA
Text HLabel 5400 2000 2    60   Input ~ 0
SCL
$Comp
L TMP006 U?
U 1 1 5A76B190
P 4100 2200
F 0 "U?" H 4100 2470 60  0000 C CNN
F 1 "TMP006" H 4110 2550 60  0000 C CNN
F 2 "" H 4100 2200 60  0001 C CNN
F 3 "" H 4100 2200 60  0001 C CNN
	1    4100 2200
	1    0    0    -1  
$EndComp
NoConn ~ 4550 2200
$Comp
L GND #PWR?
U 1 1 5A76B49A
P 4650 2500
F 0 "#PWR?" H 4650 2250 50  0001 C CNN
F 1 "GND" H 4650 2350 50  0000 C CNN
F 2 "" H 4650 2500 50  0001 C CNN
F 3 "" H 4650 2500 50  0001 C CNN
	1    4650 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 2300 4650 2500
Wire Wire Line
	4650 2400 4550 2400
Wire Wire Line
	4650 2300 4550 2300
Connection ~ 4650 2400
$Comp
L GND #PWR?
U 1 1 5A76B4B9
P 3550 2400
F 0 "#PWR?" H 3550 2150 50  0001 C CNN
F 1 "GND" H 3550 2250 50  0000 C CNN
F 2 "" H 3550 2400 50  0001 C CNN
F 3 "" H 3550 2400 50  0001 C CNN
	1    3550 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 2200 3550 2400
Wire Wire Line
	3550 2300 3650 2300
Wire Wire Line
	3650 2200 3550 2200
Connection ~ 3550 2300
$Comp
L +3V3 #PWR?
U 1 1 5A76B4E0
P 3550 2000
F 0 "#PWR?" H 3550 1850 50  0001 C CNN
F 1 "+3V3" H 3550 2140 50  0000 C CNN
F 2 "" H 3550 2000 50  0001 C CNN
F 3 "" H 3550 2000 50  0001 C CNN
	1    3550 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 2000 3550 2100
Wire Wire Line
	3550 2100 3650 2100
Wire Wire Line
	4550 2000 5400 2000
Wire Wire Line
	4550 2100 5400 2100
$Comp
L TEST_1P J?
U 1 1 5A76B941
P 5200 1900
F 0 "J?" H 5200 2170 50  0000 C CNN
F 1 "SCL" H 5200 2100 50  0000 C CNN
F 2 "" H 5400 1900 50  0001 C CNN
F 3 "" H 5400 1900 50  0001 C CNN
	1    5200 1900
	1    0    0    -1  
$EndComp
$Comp
L TEST_1P J?
U 1 1 5A76B97E
P 5200 2200
F 0 "J?" H 5200 2470 50  0000 C CNN
F 1 "SDA" H 5200 2400 50  0000 C CNN
F 2 "" H 5400 2200 50  0001 C CNN
F 3 "" H 5400 2200 50  0001 C CNN
	1    5200 2200
	-1   0    0    1   
$EndComp
Wire Wire Line
	5200 2200 5200 2100
Connection ~ 5200 2100
Wire Wire Line
	5200 2000 5200 1900
Connection ~ 5200 2000
$EndSCHEMATC
