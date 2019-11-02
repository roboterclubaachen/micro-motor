EESchema Schematic File Version 4
LIBS:macro-motor-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 6
Title "macro-motor: micro-motor development hardware"
Date "2018-09-20"
Rev "rev0"
Comp "Roboterclub Aachen e.V."
Comment1 "Raphael Lehmann"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 6850 2000 1200 300 
U 59B9FCA5
F0 "Encoder" 60
F1 "macro-motor-encoder.sch" 60
F2 "A" O L 6850 2100 60 
F3 "B" O L 6850 2200 60 
$EndSheet
$Sheet
S 8200 2400 1400 1900
U 59B9FE17
F0 "Motor_Driver" 60
F1 "macro-motor-motor-driver.sch" 60
F2 "IN_U_P" I L 8200 2500 60 
F3 "IN_U_N" I L 8200 2600 60 
F4 "IN_V_P" I L 8200 2700 60 
F5 "IN_V_N" I L 8200 2800 60 
F6 "IN_W_P" I L 8200 2900 60 
F7 "IN_W_N" I L 8200 3000 60 
F8 "ENABLE" I L 8200 3150 60 
F9 "MOSI" I L 8200 3300 60 
F10 "MISO" O L 8200 3400 60 
F11 "SCK" I L 8200 3500 60 
F12 "~CS" I L 8200 3600 60 
F13 "OUT_U" O R 9600 2500 60 
F14 "OUT_W" O R 9600 2700 60 
F15 "OUT_V" O R 9600 2600 60 
F16 "~FAULT" O L 8200 3750 60 
F17 "CurrentV" O L 8200 4100 50 
F18 "CurrentU" O L 8200 4000 50 
F19 "CurrentW" O L 8200 4200 50 
$EndSheet
$Comp
L Connector:Conn_01x05_Male J101
U 1 1 59B9FEAC
P 1000 2700
F 0 "J101" H 1000 3000 50  0000 C CNN
F 1 "CAN+PWR" V 950 2700 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 1000 2700 50  0001 C CNN
F 3 "" H 1000 2700 50  0001 C CNN
	1    1000 2700
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 59B9FFC7
P 1300 2700
F 0 "#PWR0103" H 1300 2450 50  0001 C CNN
F 1 "GND" H 1300 2550 50  0000 C CNN
F 2 "" H 1300 2700 50  0001 C CNN
F 3 "" H 1300 2700 50  0001 C CNN
	1    1300 2700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1300 2700 1200 2700
Wire Wire Line
	1200 2800 2150 2800
Wire Wire Line
	1200 2900 2150 2900
$Comp
L power:+6V #PWR0105
U 1 1 59BA046F
P 1600 2600
F 0 "#PWR0105" H 1600 2450 50  0001 C CNN
F 1 "+6V" V 1600 2800 50  0000 C CNN
F 2 "" H 1600 2600 50  0001 C CNN
F 3 "" H 1600 2600 50  0001 C CNN
	1    1600 2600
	0    1    1    0   
$EndComp
$Comp
L power:+24V #PWR0104
U 1 1 59BA04C0
P 1600 2500
F 0 "#PWR0104" H 1600 2350 50  0001 C CNN
F 1 "+24V" V 1600 2700 50  0000 C CNN
F 2 "" H 1600 2500 50  0001 C CNN
F 3 "" H 1600 2500 50  0001 C CNN
	1    1600 2500
	0    1    1    0   
$EndComp
Wire Wire Line
	1200 2500 1600 2500
Wire Wire Line
	1600 2600 1200 2600
$Sheet
S 2150 2700 1050 300 
U 59B9FE58
F0 "CAN" 60
F1 "macro-motor-can.sch" 60
F2 "CAN_RX" O R 3200 2900 60 
F3 "CAN_TX" I R 3200 2800 60 
F4 "CAN_H" B L 2150 2800 60 
F5 "CAN_L" B L 2150 2900 60 
$EndSheet
Wire Wire Line
	4800 2800 3200 2800
Wire Wire Line
	3200 2900 4800 2900
$Sheet
S 2600 2000 1350 300 
U 59BA5FB5
F0 "Temperature_Sensor" 60
F1 "macro-motor-temp-sensor.sch" 60
F2 "SDA" B R 3950 2100 60 
F3 "SCL" I R 3950 2200 60 
$EndSheet
Wire Wire Line
	4800 2200 3950 2200
Wire Wire Line
	3950 2100 4800 2100
Wire Wire Line
	6850 2100 6350 2100
Wire Wire Line
	6350 2200 6850 2200
$Comp
L power:+6V #PWR0101
U 1 1 5A768B01
P 950 3950
F 0 "#PWR0101" H 950 3800 50  0001 C CNN
F 1 "+6V" H 950 4090 50  0000 C CNN
F 2 "" H 950 3950 50  0001 C CNN
F 3 "" H 950 3950 50  0001 C CNN
	1    950  3950
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C101
U 1 1 5A768C33
P 1050 4300
F 0 "C101" H 1075 4400 50  0000 L CNN
F 1 "4µ7 16V" H 1075 4200 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1088 4150 50  0001 C CNN
F 3 "" H 1050 4300 50  0001 C CNN
	1    1050 4300
	1    0    0    -1  
$EndComp
$Comp
L Device:C C102
U 1 1 5A769218
P 2900 4300
F 0 "C102" H 2925 4400 50  0000 L CNN
F 1 "2µ2 6V3" H 2925 4200 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 2938 4150 50  0001 C CNN
F 3 "" H 2900 4300 50  0001 C CNN
	1    2900 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 4050 1600 4050
Wire Wire Line
	1600 4050 1600 3950
Connection ~ 1600 3950
Wire Wire Line
	2900 3950 2900 4150
$Comp
L power:GND #PWR0106
U 1 1 5A769C21
P 2100 4650
F 0 "#PWR0106" H 2100 4400 50  0001 C CNN
F 1 "GND" H 2100 4500 50  0000 C CNN
F 2 "" H 2100 4650 50  0001 C CNN
F 3 "" H 2100 4650 50  0001 C CNN
	1    2100 4650
	1    0    0    -1  
$EndComp
Connection ~ 2100 4550
Wire Wire Line
	2900 4550 2900 4450
Connection ~ 2900 3950
Wire Wire Line
	6350 3600 8200 3600
Wire Wire Line
	8200 3500 6350 3500
Wire Wire Line
	6350 3400 8200 3400
Wire Wire Line
	8200 3300 6350 3300
Wire Wire Line
	6350 2500 8200 2500
Wire Wire Line
	8200 2600 6350 2600
Wire Wire Line
	6350 2700 8200 2700
Wire Wire Line
	8200 2800 6350 2800
Wire Wire Line
	6350 2900 8200 2900
Wire Wire Line
	8200 3000 6350 3000
$Comp
L power:+3V3 #PWR0108
U 1 1 5A775FAA
P 3200 3950
F 0 "#PWR0108" H 3200 3800 50  0001 C CNN
F 1 "+3V3" H 3200 4090 50  0000 C CNN
F 2 "" H 3200 3950 50  0001 C CNN
F 3 "" H 3200 3950 50  0001 C CNN
	1    3200 3950
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint J106
U 1 1 5A77D176
P 4250 3800
F 0 "J106" V 4250 4150 50  0000 C CNN
F 1 "U" H 4250 4000 50  0000 C CNN
F 2 "Connector_Pin:Pin_D1.0mm_L10.0mm" H 4450 3800 50  0001 C CNN
F 3 "" H 4450 3800 50  0001 C CNN
	1    4250 3800
	0    -1   -1   0   
$EndComp
$Comp
L Connector:TestPoint J107
U 1 1 5A77D554
P 4250 3900
F 0 "J107" V 4250 4250 50  0000 C CNN
F 1 "V" H 4250 4100 50  0000 C CNN
F 2 "Connector_Pin:Pin_D1.0mm_L10.0mm" H 4450 3900 50  0001 C CNN
F 3 "" H 4450 3900 50  0001 C CNN
	1    4250 3900
	0    -1   -1   0   
$EndComp
$Comp
L Connector:TestPoint J108
U 1 1 5A77D6B7
P 4250 4000
F 0 "J108" V 4250 4350 50  0000 C CNN
F 1 "W" H 4250 4200 50  0000 C CNN
F 2 "Connector_Pin:Pin_D1.0mm_L10.0mm" H 4450 4000 50  0001 C CNN
F 3 "" H 4450 4000 50  0001 C CNN
	1    4250 4000
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C103
U 1 1 5A77E4AF
P 2900 5650
F 0 "C103" H 2925 5750 50  0000 L CNN
F 1 "2µ2 6V3" H 2925 5550 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 2938 5500 50  0001 C CNN
F 3 "" H 2900 5650 50  0001 C CNN
	1    2900 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 5400 1600 5400
Wire Wire Line
	1600 5400 1600 5300
Wire Wire Line
	2900 5300 2900 5500
$Comp
L power:GND #PWR0107
U 1 1 5A77E4BD
P 2100 6000
F 0 "#PWR0107" H 2100 5750 50  0001 C CNN
F 1 "GND" H 2100 5850 50  0000 C CNN
F 2 "" H 2100 6000 50  0001 C CNN
F 3 "" H 2100 6000 50  0001 C CNN
	1    2100 6000
	1    0    0    -1  
$EndComp
Connection ~ 2100 5900
Wire Wire Line
	2900 5900 2900 5800
Connection ~ 2900 5300
$Comp
L power:+5V #PWR0109
U 1 1 5A77E77B
P 3200 5300
F 0 "#PWR0109" H 3200 5150 50  0001 C CNN
F 1 "+5V" H 3200 5440 50  0000 C CNN
F 2 "" H 3200 5300 50  0001 C CNN
F 3 "" H 3200 5300 50  0001 C CNN
	1    3200 5300
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint J103
U 1 1 5A7826F1
P 3100 5200
F 0 "J103" V 3100 5550 50  0000 C CNN
F 1 "5V" H 3100 5400 50  0000 C CNN
F 2 "Connector_Pin:Pin_D1.0mm_L10.0mm" H 3300 5200 50  0001 C CNN
F 3 "" H 3300 5200 50  0001 C CNN
	1    3100 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 5200 3100 5300
Connection ~ 3100 5300
$Comp
L Connector:TestPoint J104
U 1 1 5A786EB9
P 3700 3900
F 0 "J104" V 3700 4250 50  0000 C CNN
F 1 "GND" H 3700 4100 50  0000 C CNN
F 2 "Connector_Pin:Pin_D1.0mm_L10.0mm" H 3900 3900 50  0001 C CNN
F 3 "" H 3900 3900 50  0001 C CNN
	1    3700 3900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 5A7873A5
P 3700 4000
F 0 "#PWR0110" H 3700 3750 50  0001 C CNN
F 1 "GND" H 3700 3850 50  0000 C CNN
F 2 "" H 3700 4000 50  0001 C CNN
F 3 "" H 3700 4000 50  0001 C CNN
	1    3700 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 4000 3700 3900
$Sheet
S 4800 2000 1550 2800
U 59B9FC4A
F0 "Microcontroller" 60
F1 "macro-motor-microcontroller.sch" 60
F2 "I2C_SDA" B L 4800 2100 60 
F3 "I2C_SCL" O L 4800 2200 60 
F4 "CAN_RX" I L 4800 2900 60 
F5 "CAN_TX" O L 4800 2800 60 
F6 "Enc_TIM2_CH1" I R 6350 2200 60 
F7 "Enc_TIM2_CH2" I R 6350 2100 60 
F8 "PWM_CH1N" O R 6350 2600 60 
F9 "PWM_CH1P" O R 6350 2500 60 
F10 "SCK" O R 6350 3500 60 
F11 "MISO" I R 6350 3400 60 
F12 "MOSI" O R 6350 3300 60 
F13 "CS" O R 6350 3600 60 
F14 "PWM_CH2P" O R 6350 2700 60 
F15 "PWM_CH2N" O R 6350 2800 60 
F16 "PWM_CH3P" O R 6350 2900 60 
F17 "PWM_CH3N" O R 6350 3000 60 
F18 "HallU" I L 4800 3800 60 
F19 "HallV" I L 4800 3900 60 
F20 "HallW" I L 4800 4000 60 
F21 "CurrentU" I R 6350 4000 50 
F22 "CurrentV" I R 6350 4100 50 
F23 "CurrentW" I R 6350 4200 50 
F24 "BemfU" I R 6350 4500 50 
F25 "BemfV" I R 6350 4600 50 
F26 "BemfW" I R 6350 4700 50 
F27 "Enable" O R 6350 3150 50 
F28 "Fault" I R 6350 3750 50 
F29 "Gpio" B L 4800 4500 50 
$EndSheet
Wire Wire Line
	1600 3950 1700 3950
Wire Wire Line
	2100 4550 2100 4650
Wire Wire Line
	2100 4550 2900 4550
Wire Wire Line
	2900 3950 3100 3950
Wire Wire Line
	1600 5300 1700 5300
Wire Wire Line
	2100 5900 2100 6000
Wire Wire Line
	2100 5900 2900 5900
Wire Wire Line
	2900 5300 3100 5300
Wire Wire Line
	3100 5300 3200 5300
Wire Wire Line
	6350 4200 8200 4200
Wire Wire Line
	6350 4100 8200 4100
Wire Wire Line
	6350 4000 8200 4000
Wire Wire Line
	6350 3150 8200 3150
Wire Wire Line
	6350 3750 7250 3750
Wire Wire Line
	9600 2500 9700 2500
Wire Wire Line
	9600 2600 9800 2600
Wire Wire Line
	9600 2700 9900 2700
Wire Wire Line
	9700 4500 9700 2500
Connection ~ 9700 2500
Wire Wire Line
	9700 2500 9950 2500
Wire Wire Line
	9800 4600 9800 2600
Connection ~ 9800 2600
Wire Wire Line
	9800 2600 9950 2600
Connection ~ 9900 2700
Wire Wire Line
	9900 2700 9950 2700
Wire Wire Line
	9900 2700 9900 4700
Wire Wire Line
	6350 4600 6700 4600
Connection ~ 6800 4500
Wire Wire Line
	6800 4500 6350 4500
Connection ~ 6700 4600
Connection ~ 6600 4700
Wire Wire Line
	6600 4700 6350 4700
$Comp
L power:GND #PWR0112
U 1 1 5B679454
P 6700 5550
F 0 "#PWR0112" H 6700 5300 50  0001 C CNN
F 1 "GND" H 6705 5377 50  0000 C CNN
F 2 "" H 6700 5550 50  0001 C CNN
F 3 "" H 6700 5550 50  0001 C CNN
	1    6700 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 5450 6600 5450
Wire Wire Line
	6600 5450 6600 5350
Wire Wire Line
	6700 5350 6700 5450
Connection ~ 6700 5450
Wire Wire Line
	6700 5450 6700 5550
Wire Wire Line
	6800 5350 6800 5450
Wire Wire Line
	6800 5450 6700 5450
$Comp
L Connector:TestPoint J105
U 1 1 5B69A41D
P 4500 4500
F 0 "J105" V 4500 4850 50  0000 C CNN
F 1 "Gpio" H 4500 4700 50  0000 C CNN
F 2 "Connector_Pin:Pin_D1.0mm_L10.0mm" H 4700 4500 50  0001 C CNN
F 3 "" H 4700 4500 50  0001 C CNN
	1    4500 4500
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R101
U 1 1 5B69CA3E
P 4350 4350
F 0 "R101" V 4143 4350 50  0000 C CNN
F 1 "10k" V 4234 4350 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 4280 4350 50  0001 C CNN
F 3 "~" H 4350 4350 50  0001 C CNN
	1    4350 4350
	0    1    1    0   
$EndComp
$Comp
L power:+3V3 #PWR0111
U 1 1 5B69CB4A
P 4100 4350
F 0 "#PWR0111" H 4100 4200 50  0001 C CNN
F 1 "+3V3" V 4115 4478 50  0000 L CNN
F 2 "" H 4100 4350 50  0001 C CNN
F 3 "" H 4100 4350 50  0001 C CNN
	1    4100 4350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4100 4350 4200 4350
Wire Wire Line
	4500 4350 4600 4350
Wire Wire Line
	4500 4500 4600 4500
Wire Wire Line
	4600 4350 4600 4500
Connection ~ 4600 4500
Wire Wire Line
	4600 4500 4800 4500
$Comp
L adp7112:ADP7112 U101
U 1 1 5B60830C
P 2100 4050
F 0 "U101" H 2100 4417 50  0000 C CNN
F 1 "ADP7112ACBZ-3.3-R7" H 2100 4326 50  0000 C CNN
F 2 "Package_CSP:WLCSP-6_1.4x1.0mm_P0.4mm" H 2100 3650 50  0001 C CIN
F 3 "http://www.analog.com/media/en/technical-documentation/data-sheets/ADP7112.pdf" H 2100 3550 50  0001 C CNN
	1    2100 4050
	1    0    0    -1  
$EndComp
NoConn ~ 1700 4150
Wire Wire Line
	2500 3950 2600 3950
Wire Wire Line
	2600 3950 2600 4150
Wire Wire Line
	2600 4150 2500 4150
Connection ~ 2600 3950
Wire Wire Line
	2600 3950 2900 3950
Wire Wire Line
	2100 4350 2100 4550
$Comp
L adp7112:ADP7112 U102
U 1 1 5B612C45
P 2100 5400
F 0 "U102" H 2100 5767 50  0000 C CNN
F 1 "ADP7112ACBZ-5.0-R7" H 2100 5676 50  0000 C CNN
F 2 "Package_CSP:WLCSP-6_1.4x1.0mm_P0.4mm" H 2100 5000 50  0001 C CIN
F 3 "http://www.analog.com/media/en/technical-documentation/data-sheets/ADP7112.pdf" H 2100 4900 50  0001 C CNN
	1    2100 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 5300 2600 5300
Wire Wire Line
	2600 5300 2600 5500
Wire Wire Line
	2600 5500 2500 5500
Connection ~ 2600 5300
Wire Wire Line
	2600 5300 2900 5300
Wire Wire Line
	2100 5700 2100 5900
NoConn ~ 1700 5500
Wire Wire Line
	950  3950 1050 3950
Wire Wire Line
	1050 3950 1050 4150
Connection ~ 1050 3950
Wire Wire Line
	1050 4450 1050 4550
$Comp
L power:GND #PWR0102
U 1 1 5B62DEF3
P 1050 4550
F 0 "#PWR0102" H 1050 4300 50  0001 C CNN
F 1 "GND" H 1050 4400 50  0000 C CNN
F 2 "" H 1050 4550 50  0001 C CNN
F 3 "" H 1050 4550 50  0001 C CNN
	1    1050 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1050 3950 1500 3950
Wire Wire Line
	1600 5300 1500 5300
Wire Wire Line
	1500 5300 1500 3950
Connection ~ 1600 5300
Connection ~ 1500 3950
Wire Wire Line
	1500 3950 1600 3950
$Comp
L Connector:TestPoint J102
U 1 1 5BA857CD
P 3100 3850
F 0 "J102" V 3100 4200 50  0000 C CNN
F 1 "3V3" H 3100 4050 50  0000 C CNN
F 2 "Connector_Pin:Pin_D1.0mm_L10.0mm" H 3300 3850 50  0001 C CNN
F 3 "" H 3300 3850 50  0001 C CNN
	1    3100 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 3850 3100 3950
Connection ~ 3100 3950
Wire Wire Line
	3100 3950 3200 3950
$Comp
L Device:R R105
U 1 1 5BA8BE2E
P 7250 4500
F 0 "R105" V 7150 4350 50  0000 C CNN
F 1 "20k" V 7550 4650 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 7180 4500 50  0001 C CNN
F 3 "~" H 7250 4500 50  0001 C CNN
	1    7250 4500
	0    1    1    0   
$EndComp
$Comp
L Device:R R104
U 1 1 5BA8BEC3
P 6800 5200
F 0 "R104" H 6870 5246 50  0000 L CNN
F 1 "1k" H 6870 5155 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 6730 5200 50  0001 C CNN
F 3 "~" H 6800 5200 50  0001 C CNN
	1    6800 5200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R106
U 1 1 5BA8C0C7
P 7250 4600
F 0 "R106" V 7050 4600 50  0000 C CNN
F 1 "20k" V 7450 4600 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 7180 4600 50  0001 C CNN
F 3 "~" H 7250 4600 50  0001 C CNN
	1    7250 4600
	0    1    1    0   
$EndComp
$Comp
L Device:R R107
U 1 1 5BA8C0FF
P 7250 4700
F 0 "R107" V 6950 4850 50  0000 C CNN
F 1 "20k" V 7350 4550 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 7180 4700 50  0001 C CNN
F 3 "~" H 7250 4700 50  0001 C CNN
	1    7250 4700
	0    1    1    0   
$EndComp
Wire Wire Line
	7400 4700 9900 4700
Wire Wire Line
	7400 4600 9800 4600
Wire Wire Line
	7400 4500 9700 4500
Wire Wire Line
	6800 4500 7100 4500
Wire Wire Line
	6700 4600 7100 4600
Wire Wire Line
	6600 4700 7100 4700
$Comp
L Device:R R103
U 1 1 5BA9EBB7
P 6700 5200
F 0 "R103" H 6400 5250 50  0000 L CNN
F 1 "1k" H 6400 5150 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 6630 5200 50  0001 C CNN
F 3 "~" H 6700 5200 50  0001 C CNN
	1    6700 5200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R102
U 1 1 5BA9EBF3
P 6600 5200
F 0 "R102" H 6200 5250 50  0000 L CNN
F 1 "1k" H 6200 5150 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 6530 5200 50  0001 C CNN
F 3 "~" H 6600 5200 50  0001 C CNN
	1    6600 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 4700 6600 5050
Wire Wire Line
	6700 4600 6700 5050
Wire Wire Line
	6800 4500 6800 5050
$Comp
L Connector:Conn_01x03_Female J109
U 1 1 5BA85F68
P 10150 2600
F 0 "J109" H 10177 2626 50  0000 L CNN
F 1 "Conn_01x03_Female" H 10177 2535 50  0000 L CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_PT-1,5-3-3.5-H_1x03_P3.50mm_Horizontal" H 10150 2600 50  0001 C CNN
F 3 "~" H 10150 2600 50  0001 C CNN
	1    10150 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 3800 4600 3800
Wire Wire Line
	4250 3900 4500 3900
Wire Wire Line
	4250 4000 4300 4000
$Comp
L Connector:Conn_01x05_Male J110
U 1 1 5BA9ABBE
P 4500 3200
F 0 "J110" V 4550 2800 50  0000 L CNN
F 1 "HALL" V 4450 3100 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 4500 3200 50  0001 C CNN
F 3 "~" H 4500 3200 50  0001 C CNN
	1    4500 3200
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0113
U 1 1 5BA9AC3B
P 4400 3450
F 0 "#PWR0113" H 4400 3300 50  0001 C CNN
F 1 "+5V" H 4400 3590 50  0000 C CNN
F 2 "" H 4400 3450 50  0001 C CNN
F 3 "" H 4400 3450 50  0001 C CNN
	1    4400 3450
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0114
U 1 1 5BA9AC88
P 4700 3500
F 0 "#PWR0114" H 4700 3250 50  0001 C CNN
F 1 "GND" H 4700 3350 50  0000 C CNN
F 2 "" H 4700 3500 50  0001 C CNN
F 3 "" H 4700 3500 50  0001 C CNN
	1    4700 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 3500 4700 3400
Wire Wire Line
	4400 3450 4400 3400
Wire Wire Line
	4300 3400 4300 4000
Connection ~ 4300 4000
Wire Wire Line
	4300 4000 4800 4000
Wire Wire Line
	4500 3400 4500 3900
Connection ~ 4500 3900
Wire Wire Line
	4500 3900 4800 3900
Wire Wire Line
	4600 3400 4600 3800
Connection ~ 4600 3800
Wire Wire Line
	4600 3800 4800 3800
$Comp
L Device:R R108
U 1 1 5BAC8808
P 7500 3850
F 0 "R108" V 7550 4000 50  0000 L CNN
F 1 "(10k)" V 7500 3750 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 7430 3850 50  0001 C CNN
F 3 "~" H 7500 3850 50  0001 C CNN
	1    7500 3850
	0    1    1    0   
$EndComp
$Comp
L power:+3V3 #PWR0128
U 1 1 5BAC8A75
P 7850 3850
F 0 "#PWR0128" H 7850 3700 50  0001 C CNN
F 1 "+3V3" V 7865 3978 50  0000 L CNN
F 2 "" H 7850 3850 50  0001 C CNN
F 3 "" H 7850 3850 50  0001 C CNN
	1    7850 3850
	0    1    1    0   
$EndComp
Wire Wire Line
	7850 3850 7650 3850
$Comp
L Connector:TestPoint J111
U 1 1 5BACC2C7
P 7150 3850
F 0 "J111" V 7150 4200 50  0000 C CNN
F 1 "Fault" H 7150 4050 50  0000 C CNN
F 2 "Connector_Pin:Pin_D1.0mm_L10.0mm" H 7350 3850 50  0001 C CNN
F 3 "" H 7350 3850 50  0001 C CNN
	1    7150 3850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7150 3850 7250 3850
Wire Wire Line
	7250 3850 7250 3750
Connection ~ 7250 3850
Wire Wire Line
	7250 3850 7350 3850
Connection ~ 7250 3750
Wire Wire Line
	7250 3750 8200 3750
$EndSCHEMATC