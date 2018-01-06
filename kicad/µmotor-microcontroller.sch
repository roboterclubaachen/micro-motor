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
$Comp
L TEST_1P J203
U 1 1 59BA1247
P 3750 1650
F 0 "J203" V 3750 2200 50  0000 C CNN
F 1 "SWDIO" V 3750 1950 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 3950 1650 50  0001 C CNN
F 3 "" H 3950 1650 50  0001 C CNN
	1    3750 1650
	0    -1   -1   0   
$EndComp
$Comp
L TEST_1P J204
U 1 1 59BA12F1
P 3750 1550
F 0 "J204" V 3750 2100 50  0000 C CNN
F 1 "SWCLK" V 3750 1850 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 3950 1550 50  0001 C CNN
F 3 "" H 3950 1550 50  0001 C CNN
	1    3750 1550
	0    -1   -1   0   
$EndComp
$Comp
L TEST_1P J205
U 1 1 59BA1332
P 3750 1450
F 0 "J205" V 3750 2000 50  0000 C CNN
F 1 "GND" V 3750 1750 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 3950 1450 50  0001 C CNN
F 3 "" H 3950 1450 50  0001 C CNN
	1    3750 1450
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR07
U 1 1 59BA1365
P 3850 1350
F 0 "#PWR07" H 3850 1100 50  0001 C CNN
F 1 "GND" H 3850 1200 50  0000 C CNN
F 2 "" H 3850 1350 50  0001 C CNN
F 3 "" H 3850 1350 50  0001 C CNN
	1    3850 1350
	-1   0    0    1   
$EndComp
Wire Notes Line
	2850 1400 2850 1700
Wire Notes Line
	2850 1700 3800 1700
Wire Notes Line
	3800 1700 3800 1400
Wire Notes Line
	3800 1400 2850 1400
Text Notes 2950 1650 1    60   ~ 0
SWD
Text HLabel 3850 3650 0    60   BiDi ~ 0
I2C_SDA
Text HLabel 3850 3750 0    60   Output ~ 0
I2C_SCL
Text HLabel 3850 3950 0    60   Input ~ 0
CAN_RX
Text HLabel 3850 3850 0    60   Output ~ 0
CAN_TX
Text HLabel 3850 3450 0    60   Output ~ 0
PWM_CH1N
Text HLabel 3850 2150 0    60   Output ~ 0
PWM_CH1P
$Comp
L TEST_1P J201
U 1 1 59BA1FC2
P 3750 2750
F 0 "J201" V 3750 3200 50  0000 C CNN
F 1 "TX" V 3750 3000 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 3950 2750 50  0001 C CNN
F 3 "" H 3950 2750 50  0001 C CNN
	1    3750 2750
	0    -1   1    0   
$EndComp
Text Notes 3150 2800 2    60   ~ 0
UART
Text HLabel 3850 2950 0    60   Input ~ 0
Enc_TIM2_CH1
Text HLabel 3850 2850 0    60   Input ~ 0
Enc_TIM2_CH2
$Comp
L C C201
U 1 1 59BAE177
P 4850 7000
F 0 "C201" H 4875 7100 50  0000 L CNN
F 1 "100n" H 4875 6900 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 4888 6850 50  0001 C CNN
F 3 "" H 4850 7000 50  0001 C CNN
	1    4850 7000
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR08
U 1 1 59BAE1E9
P 4850 6750
F 0 "#PWR08" H 4850 6600 50  0001 C CNN
F 1 "+3V3" H 4850 6890 50  0000 C CNN
F 2 "" H 4850 6750 50  0001 C CNN
F 3 "" H 4850 6750 50  0001 C CNN
	1    4850 6750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 6750 4850 6850
$Comp
L GND #PWR09
U 1 1 59BAE239
P 4850 7250
F 0 "#PWR09" H 4850 7000 50  0001 C CNN
F 1 "GND" H 4850 7100 50  0000 C CNN
F 2 "" H 4850 7250 50  0001 C CNN
F 3 "" H 4850 7250 50  0001 C CNN
	1    4850 7250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 7250 4850 7150
$Comp
L R R201
U 1 1 59BB0C75
P 3250 5050
F 0 "R201" V 3330 5050 50  0000 C CNN
F 1 "1k" V 3250 5050 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 3180 5050 50  0001 C CNN
F 3 "" H 3250 5050 50  0001 C CNN
	1    3250 5050
	0    1    1    0   
$EndComp
$Comp
L LED D201
U 1 1 59BB0CC4
P 2450 5050
F 0 "D201" H 2450 5150 50  0000 C CNN
F 1 "Blue" H 2450 4950 50  0000 C CNN
F 2 "LEDs:LED_0603" H 2450 5050 50  0001 C CNN
F 3 "" H 2450 5050 50  0001 C CNN
	1    2450 5050
	-1   0    0    1   
$EndComp
$Comp
L +3V3 #PWR010
U 1 1 59BB0D1C
P 2200 5050
F 0 "#PWR010" H 2200 4900 50  0001 C CNN
F 1 "+3V3" H 2200 5190 50  0000 C CNN
F 2 "" H 2200 5050 50  0001 C CNN
F 3 "" H 2200 5050 50  0001 C CNN
	1    2200 5050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2200 5050 2300 5050
Wire Wire Line
	2600 5050 3100 5050
$Comp
L TEST TP?
U 1 1 5A516FC4
P 1350 6800
F 0 "TP?" H 1350 7100 50  0000 C BNN
F 1 "~RST" H 1350 7050 50  0000 C CNN
F 2 "" H 1350 6800 50  0001 C CNN
F 3 "" H 1350 6800 50  0001 C CNN
	1    1350 6800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1350 6800 1550 6800
Text Label 1350 6800 0    60   ~ 0
~RST
$Comp
L GND #PWR?
U 1 1 5A5173EC
P 5700 7100
F 0 "#PWR?" H 5700 6850 50  0001 C CNN
F 1 "GND" H 5700 6950 50  0000 C CNN
F 2 "" H 5700 7100 50  0001 C CNN
F 3 "" H 5700 7100 50  0001 C CNN
	1    5700 7100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 6600 5700 7100
Wire Wire Line
	5700 6800 5800 6800
Wire Wire Line
	5800 6900 5700 6900
Connection ~ 5700 6900
Wire Wire Line
	5800 7000 5700 7000
Connection ~ 5700 7000
$Comp
L +3V3 #PWR?
U 1 1 5A517858
P 3300 6750
F 0 "#PWR?" H 3300 6600 50  0001 C CNN
F 1 "+3V3" H 3300 6890 50  0000 C CNN
F 2 "" H 3300 6750 50  0001 C CNN
F 3 "" H 3300 6750 50  0001 C CNN
	1    3300 6750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 6750 3300 7150
Wire Wire Line
	3300 7050 3400 7050
Wire Wire Line
	3400 6950 3300 6950
Connection ~ 3300 6950
Wire Wire Line
	3300 6850 3400 6850
Connection ~ 3300 6850
Wire Notes Line
	3800 2800 2850 2800
Wire Notes Line
	2850 2700 3800 2700
Wire Notes Line
	2850 2800 2850 2700
Wire Notes Line
	3800 2700 3800 2800
Wire Wire Line
	3750 1450 3850 1450
Wire Wire Line
	3850 1450 3850 1350
Wire Wire Line
	3750 1550 4150 1550
Wire Wire Line
	4150 1650 3750 1650
Wire Wire Line
	3850 3850 4150 3850
Wire Wire Line
	4150 3950 3850 3950
Text HLabel 3850 4450 0    60   Output ~ 0
SCK
Text HLabel 3850 4350 0    60   Input ~ 0
MISO
Text HLabel 3850 4250 0    60   Output ~ 0
MOSI
Text HLabel 3850 4150 0    60   Output ~ 0
CS
Wire Wire Line
	3850 3650 4150 3650
Wire Wire Line
	4150 3750 3850 3750
Wire Wire Line
	3850 4250 4150 4250
Wire Wire Line
	4150 4350 3850 4350
Wire Wire Line
	3850 4450 4150 4450
Text HLabel 3850 2050 0    60   Output ~ 0
PWM_CH2P
Text HLabel 3850 3350 0    60   Output ~ 0
PWM_CH2N
Text HLabel 3850 1950 0    60   Output ~ 0
PWM_CH3P
Text HLabel 3850 3250 0    60   Output ~ 0
PWM_CH3N
Wire Wire Line
	3850 3250 4150 3250
Wire Wire Line
	4150 3350 3850 3350
Wire Wire Line
	3850 3450 4150 3450
Wire Wire Line
	4150 2150 3850 2150
Wire Wire Line
	3850 2050 4150 2050
Wire Wire Line
	4150 1950 3850 1950
Wire Wire Line
	3750 2750 4150 2750
Text Label 3850 1550 0    60   ~ 0
SWCLK
Text Label 3850 1650 0    60   ~ 0
SWDIO
Text Label 3850 2750 0    60   ~ 0
TX
Text Notes 3200 4950 0    60   ~ 0
Debug LEDs
Text HLabel 3850 2650 0    60   Input ~ 0
SenseAll
Wire Wire Line
	3850 2650 4150 2650
Connection ~ 4050 2650
$Comp
L TEST TP?
U 1 1 5A524E0E
P 3700 2450
F 0 "TP?" V 3700 2950 50  0000 C CNN
F 1 "DAC" V 3700 2750 50  0000 C CNN
F 2 "" H 3700 2450 50  0001 C CNN
F 3 "" H 3700 2450 50  0001 C CNN
	1    3700 2450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4050 2650 4050 2550
Wire Wire Line
	4050 2550 4150 2550
Wire Wire Line
	3700 2450 4150 2450
Text Label 3950 2450 0    60   ~ 0
DAC
Text HLabel 3850 2350 0    60   Input ~ 0
SenseV
Wire Wire Line
	3850 2350 4150 2350
Wire Wire Line
	4150 4150 3850 4150
$Comp
L R R?
U 1 1 5A5277EC
P 3250 5350
F 0 "R?" V 3330 5350 50  0000 C CNN
F 1 "1k" V 3250 5350 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 3180 5350 50  0001 C CNN
F 3 "" H 3250 5350 50  0001 C CNN
	1    3250 5350
	0    1    1    0   
$EndComp
$Comp
L LED D?
U 1 1 5A5277F2
P 2450 5350
F 0 "D?" H 2450 5450 50  0000 C CNN
F 1 "Red" H 2450 5250 50  0000 C CNN
F 2 "LEDs:LED_0603" H 2450 5350 50  0001 C CNN
F 3 "" H 2450 5350 50  0001 C CNN
	1    2450 5350
	-1   0    0    1   
$EndComp
$Comp
L +3V3 #PWR?
U 1 1 5A5277F8
P 2200 5350
F 0 "#PWR?" H 2200 5200 50  0001 C CNN
F 1 "+3V3" H 2200 5490 50  0000 C CNN
F 2 "" H 2200 5350 50  0001 C CNN
F 3 "" H 2200 5350 50  0001 C CNN
	1    2200 5350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2200 5350 2300 5350
Wire Wire Line
	2600 5350 3100 5350
Wire Wire Line
	3400 5050 3850 5050
Wire Wire Line
	3850 5050 3850 5150
Wire Wire Line
	3850 5150 4150 5150
Wire Wire Line
	4150 5250 3850 5250
Wire Wire Line
	3850 5250 3850 5350
Wire Wire Line
	3850 5350 3400 5350
Wire Notes Line
	3800 4850 3800 5500
Wire Notes Line
	1950 5500 1950 4850
Wire Notes Line
	1950 4850 3800 4850
Wire Notes Line
	3800 5500 1950 5500
Text Label 2650 5050 0    60   ~ 0
LedBlueR
Text Label 2650 5350 0    60   ~ 0
LedRedR
Text Label 3400 5050 0    60   ~ 0
LedBlue
Text Label 3400 5350 0    60   ~ 0
LedRed
$Comp
L STM32L433C(B-C)Ux_u U?
U 1 1 5A5362D9
P 7250 2150
F 0 "U?" H 7250 2250 50  0000 C CNN
F 1 "STM32L433C(B-C)Ux_u" H 7250 2050 50  0000 C CNN
F 2 "UFQFPN48" H 7250 1950 50  0000 C CIN
F 3 "" H 7250 2150 50  0000 C CNN
	1    7250 2150
	-1   0    0    1   
$EndComp
$Comp
L STM32L433C(B-C)Ux_u U?
U 2 1 5A536524
P 7550 3950
F 0 "U?" H 7550 4050 50  0000 C CNN
F 1 "STM32L433C(B-C)Ux_u" H 7550 3850 50  0000 C CNN
F 2 "UFQFPN48" H 7550 3750 50  0000 C CIN
F 3 "" H 7550 3950 50  0000 C CNN
	2    7550 3950
	-1   0    0    1   
$EndComp
$Comp
L STM32L433C(B-C)Ux_u U?
U 3 1 5A53663C
P 5850 5150
F 0 "U?" H 5850 5250 50  0000 C CNN
F 1 "STM32L433C(B-C)Ux_u" H 5850 5050 50  0000 C CNN
F 2 "UFQFPN48" H 5850 4950 50  0000 C CIN
F 3 "" H 5850 5150 50  0000 C CNN
	3    5850 5150
	-1   0    0    1   
$EndComp
$Comp
L STM32L433C(B-C)Ux_u U?
U 4 1 5A5367B7
P 5150 5650
F 0 "U?" H 5150 5750 50  0000 C CNN
F 1 "STM32L433C(B-C)Ux_u" H 5150 5550 50  0000 C CNN
F 2 "UFQFPN48" H 5150 5450 50  0000 C CIN
F 3 "" H 5150 5650 50  0000 C CNN
	4    5150 5650
	-1   0    0    1   
$EndComp
$Comp
L STM32L433C(B-C)Ux_u U?
U 5 1 5A536872
P 2050 6700
F 0 "U?" H 2050 6950 50  0000 C CNN
F 1 "STM32L433C(B-C)Ux_u" H 2050 6550 50  0000 C CNN
F 2 "UFQFPN48" H 2050 6500 50  0000 C CIN
F 3 "" H 2050 6700 50  0000 C CNN
	5    2050 6700
	-1   0    0    1   
$EndComp
$Comp
L STM32L433C(B-C)Ux_u U?
U 6 1 5A536C11
P 6400 6800
F 0 "U?" H 6400 6450 50  0000 C CNN
F 1 "STM32L433C(B-C)Ux_u" H 6400 7150 50  0000 C CNN
F 2 "UFQFPN48" V 6150 6900 50  0000 C CIN
F 3 "" H 6400 6800 50  0000 C CNN
	6    6400 6800
	-1   0    0    1   
$EndComp
$Comp
L STM32L433C(B-C)Ux_u U?
U 7 1 5A536CC0
P 4000 6950
F 0 "U?" H 4000 6700 50  0000 C CNN
F 1 "STM32L433C(B-C)Ux_u" H 4000 7300 50  0000 C CNN
F 2 "UFQFPN48" H 4000 6750 50  0000 C CIN
F 3 "" H 4000 6950 50  0000 C CNN
	7    4000 6950
	-1   0    0    1   
$EndComp
$Comp
L +3V3 #PWR?
U 1 1 5A537A1D
P 1450 6600
F 0 "#PWR?" H 1450 6450 50  0001 C CNN
F 1 "+3V3" H 1450 6740 50  0000 C CNN
F 2 "" H 1450 6600 50  0001 C CNN
F 3 "" H 1450 6600 50  0001 C CNN
	1    1450 6600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 6600 1450 6700
Wire Wire Line
	1450 6700 1550 6700
Wire Wire Line
	5700 6600 5800 6600
Connection ~ 5700 6800
Wire Wire Line
	5800 6700 5700 6700
Connection ~ 5700 6700
Wire Wire Line
	3300 7150 3400 7150
Connection ~ 3300 7050
$Comp
L C C?
U 1 1 5A538741
P 5200 7000
F 0 "C?" H 5225 7100 50  0000 L CNN
F 1 "100n" H 5225 6900 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 5238 6850 50  0001 C CNN
F 3 "" H 5200 7000 50  0001 C CNN
	1    5200 7000
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR?
U 1 1 5A538747
P 5200 6750
F 0 "#PWR?" H 5200 6600 50  0001 C CNN
F 1 "+3V3" H 5200 6890 50  0000 C CNN
F 2 "" H 5200 6750 50  0001 C CNN
F 3 "" H 5200 6750 50  0001 C CNN
	1    5200 6750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 6750 5200 6850
$Comp
L GND #PWR?
U 1 1 5A53874E
P 5200 7250
F 0 "#PWR?" H 5200 7000 50  0001 C CNN
F 1 "GND" H 5200 7100 50  0000 C CNN
F 2 "" H 5200 7250 50  0001 C CNN
F 3 "" H 5200 7250 50  0001 C CNN
	1    5200 7250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 7250 5200 7150
Wire Wire Line
	3850 2950 4150 2950
Wire Wire Line
	4150 2850 3850 2850
Text HLabel 3850 4750 0    60   Input ~ 0
SenseW
Wire Wire Line
	3850 4750 4150 4750
$Comp
L TEST TP?
U 1 1 5A76EB3D
P 3700 2250
F 0 "TP?" V 3700 2750 50  0000 C CNN
F 1 "BK" V 3700 2550 50  0000 C CNN
F 2 "" H 3700 2250 50  0001 C CNN
F 3 "" H 3700 2250 50  0001 C CNN
	1    3700 2250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3700 2250 4150 2250
Text Label 3950 2250 0    60   ~ 0
BK
Text HLabel 3850 4650 0    60   BiDi ~ 0
GpioB1
Wire Wire Line
	3850 4650 4150 4650
Text HLabel 3850 4550 0    60   BiDi ~ 0
GpioB2
Wire Wire Line
	3850 4550 4150 4550
Text HLabel 3850 4050 0    60   BiDi ~ 0
GpioB7
Wire Wire Line
	3850 4050 4150 4050
$EndSCHEMATC
