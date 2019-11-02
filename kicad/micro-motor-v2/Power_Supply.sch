EESchema Schematic File Version 4
LIBS:micro-motor-v2-cache
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 5 5
Title "Micro-Motor V2"
Date "2019-09-20"
Rev "0"
Comp "Roboterclub Aachen e.V."
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Regulator_Switching:TPS82150 U501
U 1 1 5DDE5482
P 5950 1900
F 0 "U501" H 5950 2367 50  0000 C CNN
F 1 "TPS82150" H 5950 2276 50  0000 C CNN
F 2 "Package_LGA:Texas_SIL0008D_MicroSiP-8-1EP_2.8x3mm_P0.65mm_EP1.1x1.9mm" H 5950 1250 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tps82150.pdf" H 5950 1150 50  0001 C CNN
	1    5950 1900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R501
U 1 1 5DDE6A4B
P 6650 1950
F 0 "R501" H 6720 1996 50  0000 L CNN
F 1 "150k" H 6720 1905 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 6580 1950 50  0001 C CNN
F 3 "~" H 6650 1950 50  0001 C CNN
	1    6650 1950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R502
U 1 1 5DDE7155
P 6650 2450
F 0 "R502" H 6720 2496 50  0000 L CNN
F 1 "47k" H 6720 2405 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 6580 2450 50  0001 C CNN
F 3 "~" H 6650 2450 50  0001 C CNN
	1    6650 2450
	1    0    0    -1  
$EndComp
$Comp
L Device:C C503
U 1 1 5DDE794A
P 7150 1950
F 0 "C503" H 7265 1996 50  0000 L CNN
F 1 "22µ 10V 0805" H 7265 1905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7188 1800 50  0001 C CNN
F 3 "~" H 7150 1950 50  0001 C CNN
	1    7150 1950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C501
U 1 1 5DDE7CAB
P 3400 1950
F 0 "C501" H 3515 1996 50  0000 L CNN
F 1 "22µ 10V 0805" H 3515 1905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3438 1800 50  0001 C CNN
F 3 "~" H 3400 1950 50  0001 C CNN
	1    3400 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 1900 5450 1900
Wire Wire Line
	5450 1900 5450 1700
Wire Wire Line
	5450 1700 5550 1700
Connection ~ 5450 1700
Wire Wire Line
	6350 1700 6650 1700
Wire Wire Line
	7150 1700 7150 1800
Wire Wire Line
	7150 1700 7700 1700
Connection ~ 7150 1700
Wire Wire Line
	6650 1800 6650 1700
Connection ~ 6650 1700
Wire Wire Line
	6650 1700 7150 1700
Wire Wire Line
	6350 1900 6450 1900
Wire Wire Line
	6450 1900 6450 2200
Wire Wire Line
	6450 2200 6650 2200
Wire Wire Line
	6650 2200 6650 2100
Wire Wire Line
	6650 2200 6650 2300
Connection ~ 6650 2200
Wire Wire Line
	5950 2300 5950 2700
Wire Wire Line
	5950 2700 6650 2700
Wire Wire Line
	6650 2700 6650 2600
Wire Wire Line
	6650 2700 7150 2700
Wire Wire Line
	7150 2700 7150 2100
Connection ~ 6650 2700
Connection ~ 5950 2700
$Comp
L power:GND #PWR0503
U 1 1 5DDE95B9
P 5950 2800
F 0 "#PWR0503" H 5950 2550 50  0001 C CNN
F 1 "GND" H 5955 2627 50  0000 C CNN
F 2 "" H 5950 2800 50  0001 C CNN
F 3 "" H 5950 2800 50  0001 C CNN
	1    5950 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 2800 5950 2700
$Comp
L power:+6V #PWR0501
U 1 1 5DDE9D9C
P 3000 1700
F 0 "#PWR0501" H 3000 1550 50  0001 C CNN
F 1 "+6V" V 3015 1828 50  0000 L CNN
F 2 "" H 3000 1700 50  0001 C CNN
F 3 "" H 3000 1700 50  0001 C CNN
	1    3000 1700
	0    -1   -1   0   
$EndComp
$Comp
L Regulator_Switching:TPS82150 U502
U 1 1 5DDEDF6B
P 5950 4250
F 0 "U502" H 5950 4717 50  0000 C CNN
F 1 "TPS82150" H 5950 4626 50  0000 C CNN
F 2 "Package_LGA:Texas_SIL0008D_MicroSiP-8-1EP_2.8x3mm_P0.65mm_EP1.1x1.9mm" H 5950 3600 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tps82150.pdf" H 5950 3500 50  0001 C CNN
	1    5950 4250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R503
U 1 1 5DDEDF75
P 6650 4300
F 0 "R503" H 6720 4346 50  0000 L CNN
F 1 "27k" H 6720 4255 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 6580 4300 50  0001 C CNN
F 3 "~" H 6650 4300 50  0001 C CNN
	1    6650 4300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R504
U 1 1 5DDEDF7F
P 6650 4800
F 0 "R504" H 6720 4846 50  0000 L CNN
F 1 "5k1" H 6720 4755 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 6580 4800 50  0001 C CNN
F 3 "~" H 6650 4800 50  0001 C CNN
	1    6650 4800
	1    0    0    -1  
$EndComp
$Comp
L Device:C C504
U 1 1 5DDEDF89
P 7150 4300
F 0 "C504" H 7265 4346 50  0000 L CNN
F 1 "22µ 10V 0805" H 7265 4255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7188 4150 50  0001 C CNN
F 3 "~" H 7150 4300 50  0001 C CNN
	1    7150 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 4250 5450 4250
Wire Wire Line
	5450 4250 5450 4050
Wire Wire Line
	5450 4050 5550 4050
Connection ~ 5450 4050
Wire Wire Line
	6350 4050 6650 4050
Wire Wire Line
	7150 4050 7150 4150
Wire Wire Line
	7150 4050 7700 4050
Connection ~ 7150 4050
Wire Wire Line
	6650 4150 6650 4050
Connection ~ 6650 4050
Wire Wire Line
	6650 4050 7150 4050
Wire Wire Line
	6350 4250 6450 4250
Wire Wire Line
	6450 4250 6450 4550
Wire Wire Line
	6450 4550 6650 4550
Wire Wire Line
	6650 4550 6650 4450
Wire Wire Line
	6650 4550 6650 4650
Connection ~ 6650 4550
Wire Wire Line
	5950 4650 5950 5050
Wire Wire Line
	5950 5050 6650 5050
Wire Wire Line
	6650 5050 6650 4950
Wire Wire Line
	6650 5050 7150 5050
Wire Wire Line
	7150 5050 7150 4450
Connection ~ 6650 5050
Connection ~ 5950 5050
$Comp
L power:GND #PWR0504
U 1 1 5DDEDFBB
P 5950 5150
F 0 "#PWR0504" H 5950 4900 50  0001 C CNN
F 1 "GND" H 5955 4977 50  0000 C CNN
F 2 "" H 5950 5150 50  0001 C CNN
F 3 "" H 5950 5150 50  0001 C CNN
	1    5950 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 5150 5950 5050
$Comp
L power:+3V3 #PWR0505
U 1 1 5DDF5BFF
P 7700 1700
F 0 "#PWR0505" H 7700 1550 50  0001 C CNN
F 1 "+3V3" V 7715 1828 50  0000 L CNN
F 2 "" H 7700 1700 50  0001 C CNN
F 3 "" H 7700 1700 50  0001 C CNN
	1    7700 1700
	0    1    1    0   
$EndComp
NoConn ~ 5550 2000
NoConn ~ 5550 2100
NoConn ~ 5550 4450
NoConn ~ 5550 4350
$Comp
L power:+5V #PWR0506
U 1 1 5DDF9E59
P 7700 4050
F 0 "#PWR0506" H 7700 3900 50  0001 C CNN
F 1 "+5V" V 7715 4178 50  0000 L CNN
F 2 "" H 7700 4050 50  0001 C CNN
F 3 "" H 7700 4050 50  0001 C CNN
	1    7700 4050
	0    1    1    0   
$EndComp
Wire Wire Line
	3400 1800 3400 1700
Connection ~ 3400 1700
Wire Wire Line
	3400 1700 3000 1700
$Comp
L power:GND #PWR0118
U 1 1 5DB393BE
P 3400 2200
F 0 "#PWR0118" H 3400 1950 50  0001 C CNN
F 1 "GND" H 3405 2027 50  0000 C CNN
F 2 "" H 3400 2200 50  0001 C CNN
F 3 "" H 3400 2200 50  0001 C CNN
	1    3400 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 2200 3400 2100
$Comp
L Device:C C502
U 1 1 5DBCABDE
P 3400 4350
F 0 "C502" H 3515 4396 50  0000 L CNN
F 1 "22µ 10V 0805" H 3515 4305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3438 4200 50  0001 C CNN
F 3 "~" H 3400 4350 50  0001 C CNN
	1    3400 4350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0507
U 1 1 5DBCABE8
P 3400 4600
F 0 "#PWR0507" H 3400 4350 50  0001 C CNN
F 1 "GND" H 3405 4427 50  0000 C CNN
F 2 "" H 3400 4600 50  0001 C CNN
F 3 "" H 3400 4600 50  0001 C CNN
	1    3400 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 4600 3400 4500
Wire Wire Line
	3400 4200 3400 4050
Wire Wire Line
	3400 4050 5450 4050
Wire Wire Line
	3400 1700 5450 1700
$Comp
L power:+6V #PWR0502
U 1 1 5DBD45BA
P 3050 4050
F 0 "#PWR0502" H 3050 3900 50  0001 C CNN
F 1 "+6V" V 3065 4178 50  0000 L CNN
F 2 "" H 3050 4050 50  0001 C CNN
F 3 "" H 3050 4050 50  0001 C CNN
	1    3050 4050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3050 4050 3400 4050
Connection ~ 3400 4050
$EndSCHEMATC