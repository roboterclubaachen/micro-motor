EESchema Schematic File Version 4
LIBS:micro-motor-v2-cache
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 5
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
L Interface_CAN_LIN:TCAN332 U?
U 1 1 5DA8F9B4
P 5250 3450
F 0 "U?" H 5250 4031 50  0000 C CNN
F 1 "TCAN332" H 5250 3940 50  0000 C CNN
F 2 "" H 5250 2950 50  0001 C CIN
F 3 "http://www.ti.com/lit/ds/symlink/tcan337.pdf" H 5250 3450 50  0001 C CNN
	1    5250 3450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5DA8FEB1
P 5250 4300
F 0 "#PWR?" H 5250 4050 50  0001 C CNN
F 1 "GND" H 5255 4127 50  0000 C CNN
F 2 "" H 5250 4300 50  0001 C CNN
F 3 "" H 5250 4300 50  0001 C CNN
	1    5250 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 3850 5250 4300
$Comp
L power:+3V3 #PWR?
U 1 1 5DA91693
P 5250 2450
F 0 "#PWR?" H 5250 2300 50  0001 C CNN
F 1 "+3V3" H 5265 2623 50  0000 C CNN
F 2 "" H 5250 2450 50  0001 C CNN
F 3 "" H 5250 2450 50  0001 C CNN
	1    5250 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 3050 5250 2450
Text HLabel 3750 3250 0    50   Input ~ 0
CAN_TX
Text HLabel 3750 3350 0    50   Output ~ 0
CAN_RX
Wire Wire Line
	4750 3350 3750 3350
Wire Wire Line
	3750 3250 4750 3250
$Comp
L Connector_Generic_MountingPin:Conn_01x02_MountingPin J?
U 1 1 5DA925F6
P 7300 3400
F 0 "J?" H 7388 3314 50  0000 L CNN
F 1 "Conn_01x02_MountingPin" H 7388 3223 50  0000 L CNN
F 2 "" H 7300 3400 50  0001 C CNN
F 3 "~" H 7300 3400 50  0001 C CNN
	1    7300 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 3350 6900 3350
Wire Wire Line
	6900 3350 6900 3400
Wire Wire Line
	6900 3400 7100 3400
Wire Wire Line
	5750 3550 7100 3550
Wire Wire Line
	7100 3550 7100 3500
$EndSCHEMATC
