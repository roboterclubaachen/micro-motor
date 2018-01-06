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
Text HLabel 4950 3450 0    60   Output ~ 0
CAN_RX
Text HLabel 4950 3150 0    60   Input ~ 0
CAN_TX
Text HLabel 6600 3250 2    60   BiDi ~ 0
CAN_H
Text HLabel 6600 3350 2    60   BiDi ~ 0
CAN_L
$Comp
L TCAN330 U601
U 1 1 59BAE765
P 5950 3300
F 0 "U601" H 5950 3600 60  0000 C CNN
F 1 "TCAN330" H 5950 3000 60  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-8" H 5950 3250 60  0001 C CNN
F 3 "" H 5950 3250 60  0001 C CNN
	1    5950 3300
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR027
U 1 1 59BAE85A
P 5400 3350
F 0 "#PWR027" H 5400 3200 50  0001 C CNN
F 1 "+3V3" H 5400 3490 50  0000 C CNN
F 2 "" H 5400 3350 50  0001 C CNN
F 3 "" H 5400 3350 50  0001 C CNN
	1    5400 3350
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR028
U 1 1 59BAE870
P 5200 3250
F 0 "#PWR028" H 5200 3000 50  0001 C CNN
F 1 "GND" H 5200 3100 50  0000 C CNN
F 2 "" H 5200 3250 50  0001 C CNN
F 3 "" H 5200 3250 50  0001 C CNN
	1    5200 3250
	0    1    1    0   
$EndComp
Wire Wire Line
	5200 3250 5500 3250
Wire Wire Line
	5400 3350 5500 3350
Wire Wire Line
	4950 3450 5500 3450
Wire Wire Line
	4950 3150 5500 3150
Wire Wire Line
	6600 3350 6400 3350
Wire Wire Line
	6400 3250 6600 3250
NoConn ~ 6400 3150
NoConn ~ 6400 3450
$Comp
L C C601
U 1 1 59BAEA09
P 5850 4300
F 0 "C601" H 5875 4400 50  0000 L CNN
F 1 "100n" H 5875 4200 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 5888 4150 50  0001 C CNN
F 3 "" H 5850 4300 50  0001 C CNN
	1    5850 4300
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR029
U 1 1 59BAEB8F
P 5850 4050
F 0 "#PWR029" H 5850 3900 50  0001 C CNN
F 1 "+3V3" H 5850 4190 50  0000 C CNN
F 2 "" H 5850 4050 50  0001 C CNN
F 3 "" H 5850 4050 50  0001 C CNN
	1    5850 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 4050 5850 4150
$Comp
L GND #PWR030
U 1 1 59BAEBB1
P 5850 4550
F 0 "#PWR030" H 5850 4300 50  0001 C CNN
F 1 "GND" H 5850 4400 50  0000 C CNN
F 2 "" H 5850 4550 50  0001 C CNN
F 3 "" H 5850 4550 50  0001 C CNN
	1    5850 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 4550 5850 4450
$EndSCHEMATC
