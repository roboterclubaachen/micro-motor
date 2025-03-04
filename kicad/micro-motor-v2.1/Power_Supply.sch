EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 5 5
Title "Micro-Motor V2.1"
Date "2021-07-31"
Rev "1"
Comp "Roboterclub Aachen e.V."
Comment1 "Raphael Lehmann"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
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
	7150 1700 7150 1800
Wire Wire Line
	7150 1700 7700 1700
Wire Wire Line
	7150 2700 7150 2100
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
	7150 4050 7150 4150
Wire Wire Line
	7150 4050 7700 4050
Wire Wire Line
	7150 5050 7150 4450
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
Wire Wire Line
	3400 1700 3000 1700
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
Wire Wire Line
	3400 4200 3400 4050
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
$Comp
L Regulator_Switching:LMZM23601V3 U501
U 1 1 61099B70
P 5950 1900
F 0 "U501" H 5950 2367 50  0000 C CNN
F 1 "LMZM23601V3" H 5950 2276 50  0000 C CNN
F 2 "Package_LGA:Texas_SIL0010A_MicroSiP-10-1EP_3.8x3mm_P0.6mm_EP0.7x2.9mm" H 5950 1050 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lmzm23601.pdf" H 5950 1150 50  0001 C CNN
	1    5950 1900
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Switching:LMZM23601V5 U502
U 1 1 6109CCB5
P 5950 4250
F 0 "U502" H 5950 4717 50  0000 C CNN
F 1 "LMZM23601V5" H 5950 4626 50  0000 C CNN
F 2 "Package_LGA:Texas_SIL0010A_MicroSiP-10-1EP_3.8x3mm_P0.6mm_EP0.7x2.9mm" H 5950 3400 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lmzm23601.pdf" H 5950 3500 50  0001 C CNN
	1    5950 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 5050 5950 5050
Wire Wire Line
	5950 5050 5950 5150
Wire Wire Line
	5950 5050 5950 4650
Connection ~ 5950 5050
Wire Wire Line
	6450 4050 6550 4050
Connection ~ 7150 4050
Wire Wire Line
	6450 4250 6550 4250
Wire Wire Line
	6550 4250 6550 4050
Connection ~ 6550 4050
Wire Wire Line
	6550 4050 7150 4050
NoConn ~ 6450 4450
Wire Wire Line
	3400 4050 5250 4050
Connection ~ 3400 4050
Wire Wire Line
	5450 4450 5250 4450
Wire Wire Line
	5250 4450 5250 4050
Connection ~ 5250 4050
Wire Wire Line
	5250 4050 5350 4050
Wire Wire Line
	3400 4500 3400 5050
Wire Wire Line
	5450 4250 5350 4250
Wire Wire Line
	5950 2300 5950 2700
Wire Wire Line
	5950 2700 7150 2700
Connection ~ 5950 2700
Wire Wire Line
	5950 2700 5950 2800
Wire Wire Line
	3400 2100 3400 2700
NoConn ~ 6450 2100
Wire Wire Line
	6450 1700 6550 1700
Connection ~ 7150 1700
Wire Wire Line
	6450 1900 6550 1900
Wire Wire Line
	6550 1900 6550 1700
Connection ~ 6550 1700
Wire Wire Line
	6550 1700 7150 1700
Wire Wire Line
	5450 1700 5350 1700
Connection ~ 3400 1700
Wire Wire Line
	5450 2100 5250 2100
Wire Wire Line
	5250 2100 5250 1700
Connection ~ 5250 1700
Wire Wire Line
	5250 1700 3400 1700
Wire Wire Line
	5450 1900 5350 1900
Wire Wire Line
	3400 2700 5950 2700
Wire Wire Line
	5350 1900 5350 1700
Connection ~ 5350 1700
Wire Wire Line
	5350 1700 5250 1700
Wire Wire Line
	3400 5050 5950 5050
Wire Wire Line
	5350 4250 5350 4050
Connection ~ 5350 4050
Wire Wire Line
	5350 4050 5450 4050
$EndSCHEMATC
