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
LIBS:JLink_Mini
LIBS:STM32L062K8
LIBS:Pad
LIBS:LIS3DH
LIBS:TPS61030
LIBS:MIC5225
LIBS:STM32L0-LQFP48
LIBS:MAX98357
LIBS:Micro SD 503182-1852
LIBS:PN532
LIBS:Oscillator
LIBS:APA102
LIBS:SN74LVC2T45
LIBS:NFC_Toy-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 4
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
L APA102 U6
U 1 1 5AD09EB0
P 2750 2650
F 0 "U6" H 2750 2550 50  0000 C CNN
F 1 "APA102" H 2750 2750 50  0000 C CNN
F 2 "NFC_Toy_Lib:APA102_hand_solder" H 2750 2650 50  0001 C CNN
F 3 "DOCUMENTATION" H 2750 2650 50  0001 C CNN
	1    2750 2650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR062
U 1 1 5AD09F5A
P 3600 3250
F 0 "#PWR062" H 3600 3000 50  0001 C CNN
F 1 "GND" H 3600 3100 50  0000 C CNN
F 2 "" H 3600 3250 50  0001 C CNN
F 3 "" H 3600 3250 50  0001 C CNN
	1    3600 3250
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR063
U 1 1 5AD09F42
P 3600 2850
F 0 "#PWR063" H 3600 2700 50  0001 C CNN
F 1 "+5V" H 3600 2990 50  0000 C CNN
F 2 "" H 3600 2850 50  0001 C CNN
F 3 "" H 3600 2850 50  0001 C CNN
	1    3600 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 2850 3600 2850
Wire Wire Line
	3500 2650 4050 2650
Wire Wire Line
	3500 2550 4050 2550
$Comp
L C C35
U 1 1 5AD09FF8
P 3600 3050
F 0 "C35" H 3625 3150 50  0000 L CNN
F 1 "0.1u" H 3625 2950 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 3638 2900 50  0001 C CNN
F 3 "" H 3600 3050 50  0001 C CNN
	1    3600 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 3250 3600 3200
Wire Wire Line
	3600 2850 3600 2900
$Comp
L GND #PWR064
U 1 1 5AD0A04C
P 1950 2950
F 0 "#PWR064" H 1950 2700 50  0001 C CNN
F 1 "GND" H 1950 2800 50  0000 C CNN
F 2 "" H 1950 2950 50  0001 C CNN
F 3 "" H 1950 2950 50  0001 C CNN
	1    1950 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 2850 1950 2850
Wire Wire Line
	1950 2850 1950 2950
$Comp
L APA102 U9
U 1 1 5AD0A0B2
P 4800 2650
F 0 "U9" H 4800 2550 50  0000 C CNN
F 1 "APA102" H 4800 2750 50  0000 C CNN
F 2 "NFC_Toy_Lib:APA102_hand_solder" H 4800 2650 50  0001 C CNN
F 3 "DOCUMENTATION" H 4800 2650 50  0001 C CNN
	1    4800 2650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR065
U 1 1 5AD0A0B8
P 5650 3250
F 0 "#PWR065" H 5650 3000 50  0001 C CNN
F 1 "GND" H 5650 3100 50  0000 C CNN
F 2 "" H 5650 3250 50  0001 C CNN
F 3 "" H 5650 3250 50  0001 C CNN
	1    5650 3250
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR066
U 1 1 5AD0A0BE
P 5650 2850
F 0 "#PWR066" H 5650 2700 50  0001 C CNN
F 1 "+5V" H 5650 2990 50  0000 C CNN
F 2 "" H 5650 2850 50  0001 C CNN
F 3 "" H 5650 2850 50  0001 C CNN
	1    5650 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 2850 5650 2850
Wire Wire Line
	5550 2650 6100 2650
Wire Wire Line
	5550 2550 6100 2550
$Comp
L C C40
U 1 1 5AD0A0C7
P 5650 3050
F 0 "C40" H 5675 3150 50  0000 L CNN
F 1 "0.1u" H 5675 2950 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 5688 2900 50  0001 C CNN
F 3 "" H 5650 3050 50  0001 C CNN
	1    5650 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 3250 5650 3200
Wire Wire Line
	5650 2850 5650 2900
$Comp
L GND #PWR067
U 1 1 5AD0A0CF
P 4000 2950
F 0 "#PWR067" H 4000 2700 50  0001 C CNN
F 1 "GND" H 4000 2800 50  0000 C CNN
F 2 "" H 4000 2950 50  0001 C CNN
F 3 "" H 4000 2950 50  0001 C CNN
	1    4000 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 2850 4000 2850
Wire Wire Line
	4000 2850 4000 2950
$Comp
L APA102 U13
U 1 1 5AD0A12F
P 6850 2650
F 0 "U13" H 6850 2550 50  0000 C CNN
F 1 "APA102" H 6850 2750 50  0000 C CNN
F 2 "NFC_Toy_Lib:APA102_hand_solder" H 6850 2650 50  0001 C CNN
F 3 "DOCUMENTATION" H 6850 2650 50  0001 C CNN
	1    6850 2650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR068
U 1 1 5AD0A135
P 7700 3250
F 0 "#PWR068" H 7700 3000 50  0001 C CNN
F 1 "GND" H 7700 3100 50  0000 C CNN
F 2 "" H 7700 3250 50  0001 C CNN
F 3 "" H 7700 3250 50  0001 C CNN
	1    7700 3250
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR069
U 1 1 5AD0A13B
P 7700 2850
F 0 "#PWR069" H 7700 2700 50  0001 C CNN
F 1 "+5V" H 7700 2990 50  0000 C CNN
F 2 "" H 7700 2850 50  0001 C CNN
F 3 "" H 7700 2850 50  0001 C CNN
	1    7700 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 2850 7700 2850
Wire Wire Line
	7600 2650 8150 2650
Wire Wire Line
	7600 2550 8150 2550
$Comp
L C C43
U 1 1 5AD0A144
P 7700 3050
F 0 "C43" H 7725 3150 50  0000 L CNN
F 1 "0.1u" H 7725 2950 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 7738 2900 50  0001 C CNN
F 3 "" H 7700 3050 50  0001 C CNN
	1    7700 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 3250 7700 3200
Wire Wire Line
	7700 2850 7700 2900
$Comp
L GND #PWR070
U 1 1 5AD0A14C
P 6050 2950
F 0 "#PWR070" H 6050 2700 50  0001 C CNN
F 1 "GND" H 6050 2800 50  0000 C CNN
F 2 "" H 6050 2950 50  0001 C CNN
F 3 "" H 6050 2950 50  0001 C CNN
	1    6050 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 2850 6050 2850
Wire Wire Line
	6050 2850 6050 2950
$Comp
L APA102 U16
U 1 1 5AD0A29E
P 8900 2650
F 0 "U16" H 8900 2550 50  0000 C CNN
F 1 "APA102" H 8900 2750 50  0000 C CNN
F 2 "NFC_Toy_Lib:APA102_hand_solder" H 8900 2650 50  0001 C CNN
F 3 "DOCUMENTATION" H 8900 2650 50  0001 C CNN
	1    8900 2650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR071
U 1 1 5AD0A2A5
P 9750 3250
F 0 "#PWR071" H 9750 3000 50  0001 C CNN
F 1 "GND" H 9750 3100 50  0000 C CNN
F 2 "" H 9750 3250 50  0001 C CNN
F 3 "" H 9750 3250 50  0001 C CNN
	1    9750 3250
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR072
U 1 1 5AD0A2AB
P 9750 2850
F 0 "#PWR072" H 9750 2700 50  0001 C CNN
F 1 "+5V" H 9750 2990 50  0000 C CNN
F 2 "" H 9750 2850 50  0001 C CNN
F 3 "" H 9750 2850 50  0001 C CNN
	1    9750 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	9650 2850 9750 2850
$Comp
L C C46
U 1 1 5AD0A2B4
P 9750 3050
F 0 "C46" H 9775 3150 50  0000 L CNN
F 1 "0.1u" H 9775 2950 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 9788 2900 50  0001 C CNN
F 3 "" H 9750 3050 50  0001 C CNN
	1    9750 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9750 3250 9750 3200
Wire Wire Line
	9750 2850 9750 2900
$Comp
L GND #PWR073
U 1 1 5AD0A2BD
P 8100 2950
F 0 "#PWR073" H 8100 2700 50  0001 C CNN
F 1 "GND" H 8100 2800 50  0000 C CNN
F 2 "" H 8100 2950 50  0001 C CNN
F 3 "" H 8100 2950 50  0001 C CNN
	1    8100 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8150 2850 8100 2850
Wire Wire Line
	8100 2850 8100 2950
$Comp
L APA102 U7
U 1 1 5AD0A56B
P 2750 4000
F 0 "U7" H 2750 3900 50  0000 C CNN
F 1 "APA102" H 2750 4100 50  0000 C CNN
F 2 "NFC_Toy_Lib:APA102_hand_solder" H 2750 4000 50  0001 C CNN
F 3 "DOCUMENTATION" H 2750 4000 50  0001 C CNN
	1    2750 4000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR074
U 1 1 5AD0A571
P 3600 4600
F 0 "#PWR074" H 3600 4350 50  0001 C CNN
F 1 "GND" H 3600 4450 50  0000 C CNN
F 2 "" H 3600 4600 50  0001 C CNN
F 3 "" H 3600 4600 50  0001 C CNN
	1    3600 4600
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR075
U 1 1 5AD0A577
P 3600 4200
F 0 "#PWR075" H 3600 4050 50  0001 C CNN
F 1 "+5V" H 3600 4340 50  0000 C CNN
F 2 "" H 3600 4200 50  0001 C CNN
F 3 "" H 3600 4200 50  0001 C CNN
	1    3600 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 4200 3600 4200
Wire Wire Line
	3500 4000 4050 4000
Wire Wire Line
	3500 3900 4050 3900
$Comp
L C C36
U 1 1 5AD0A580
P 3600 4400
F 0 "C36" H 3625 4500 50  0000 L CNN
F 1 "0.1u" H 3625 4300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 3638 4250 50  0001 C CNN
F 3 "" H 3600 4400 50  0001 C CNN
	1    3600 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 4600 3600 4550
Wire Wire Line
	3600 4200 3600 4250
$Comp
L GND #PWR076
U 1 1 5AD0A588
P 1950 4300
F 0 "#PWR076" H 1950 4050 50  0001 C CNN
F 1 "GND" H 1950 4150 50  0000 C CNN
F 2 "" H 1950 4300 50  0001 C CNN
F 3 "" H 1950 4300 50  0001 C CNN
	1    1950 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 4200 1950 4200
Wire Wire Line
	1950 4200 1950 4300
$Comp
L APA102 U10
U 1 1 5AD0A590
P 4800 4000
F 0 "U10" H 4800 3900 50  0000 C CNN
F 1 "APA102" H 4800 4100 50  0000 C CNN
F 2 "NFC_Toy_Lib:APA102_hand_solder" H 4800 4000 50  0001 C CNN
F 3 "DOCUMENTATION" H 4800 4000 50  0001 C CNN
	1    4800 4000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR077
U 1 1 5AD0A596
P 5650 4600
F 0 "#PWR077" H 5650 4350 50  0001 C CNN
F 1 "GND" H 5650 4450 50  0000 C CNN
F 2 "" H 5650 4600 50  0001 C CNN
F 3 "" H 5650 4600 50  0001 C CNN
	1    5650 4600
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR078
U 1 1 5AD0A59C
P 5650 4200
F 0 "#PWR078" H 5650 4050 50  0001 C CNN
F 1 "+5V" H 5650 4340 50  0000 C CNN
F 2 "" H 5650 4200 50  0001 C CNN
F 3 "" H 5650 4200 50  0001 C CNN
	1    5650 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 4200 5650 4200
Wire Wire Line
	5550 4000 6100 4000
Wire Wire Line
	5550 3900 6100 3900
$Comp
L C C41
U 1 1 5AD0A5A5
P 5650 4400
F 0 "C41" H 5675 4500 50  0000 L CNN
F 1 "0.1u" H 5675 4300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 5688 4250 50  0001 C CNN
F 3 "" H 5650 4400 50  0001 C CNN
	1    5650 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 4600 5650 4550
Wire Wire Line
	5650 4200 5650 4250
$Comp
L GND #PWR079
U 1 1 5AD0A5AD
P 4000 4300
F 0 "#PWR079" H 4000 4050 50  0001 C CNN
F 1 "GND" H 4000 4150 50  0000 C CNN
F 2 "" H 4000 4300 50  0001 C CNN
F 3 "" H 4000 4300 50  0001 C CNN
	1    4000 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 4200 4000 4200
Wire Wire Line
	4000 4200 4000 4300
$Comp
L APA102 U14
U 1 1 5AD0A5B5
P 6850 4000
F 0 "U14" H 6850 3900 50  0000 C CNN
F 1 "APA102" H 6850 4100 50  0000 C CNN
F 2 "NFC_Toy_Lib:APA102_hand_solder" H 6850 4000 50  0001 C CNN
F 3 "DOCUMENTATION" H 6850 4000 50  0001 C CNN
	1    6850 4000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR080
U 1 1 5AD0A5BB
P 7700 4600
F 0 "#PWR080" H 7700 4350 50  0001 C CNN
F 1 "GND" H 7700 4450 50  0000 C CNN
F 2 "" H 7700 4600 50  0001 C CNN
F 3 "" H 7700 4600 50  0001 C CNN
	1    7700 4600
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR081
U 1 1 5AD0A5C1
P 7700 4200
F 0 "#PWR081" H 7700 4050 50  0001 C CNN
F 1 "+5V" H 7700 4340 50  0000 C CNN
F 2 "" H 7700 4200 50  0001 C CNN
F 3 "" H 7700 4200 50  0001 C CNN
	1    7700 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 4200 7700 4200
Wire Wire Line
	7600 4000 8150 4000
Wire Wire Line
	7600 3900 8150 3900
$Comp
L C C44
U 1 1 5AD0A5CA
P 7700 4400
F 0 "C44" H 7725 4500 50  0000 L CNN
F 1 "0.1u" H 7725 4300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 7738 4250 50  0001 C CNN
F 3 "" H 7700 4400 50  0001 C CNN
	1    7700 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 4600 7700 4550
Wire Wire Line
	7700 4200 7700 4250
$Comp
L GND #PWR082
U 1 1 5AD0A5D2
P 6050 4300
F 0 "#PWR082" H 6050 4050 50  0001 C CNN
F 1 "GND" H 6050 4150 50  0000 C CNN
F 2 "" H 6050 4300 50  0001 C CNN
F 3 "" H 6050 4300 50  0001 C CNN
	1    6050 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 4200 6050 4200
Wire Wire Line
	6050 4200 6050 4300
$Comp
L APA102 U17
U 1 1 5AD0A5DA
P 8900 4000
F 0 "U17" H 8900 3900 50  0000 C CNN
F 1 "APA102" H 8900 4100 50  0000 C CNN
F 2 "NFC_Toy_Lib:APA102_hand_solder" H 8900 4000 50  0001 C CNN
F 3 "DOCUMENTATION" H 8900 4000 50  0001 C CNN
	1    8900 4000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR083
U 1 1 5AD0A5E0
P 9750 4600
F 0 "#PWR083" H 9750 4350 50  0001 C CNN
F 1 "GND" H 9750 4450 50  0000 C CNN
F 2 "" H 9750 4600 50  0001 C CNN
F 3 "" H 9750 4600 50  0001 C CNN
	1    9750 4600
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR084
U 1 1 5AD0A5E6
P 9750 4200
F 0 "#PWR084" H 9750 4050 50  0001 C CNN
F 1 "+5V" H 9750 4340 50  0000 C CNN
F 2 "" H 9750 4200 50  0001 C CNN
F 3 "" H 9750 4200 50  0001 C CNN
	1    9750 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9650 4200 9750 4200
$Comp
L C C47
U 1 1 5AD0A5ED
P 9750 4400
F 0 "C47" H 9775 4500 50  0000 L CNN
F 1 "0.1u" H 9775 4300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 9788 4250 50  0001 C CNN
F 3 "" H 9750 4400 50  0001 C CNN
	1    9750 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	9750 4600 9750 4550
Wire Wire Line
	9750 4200 9750 4250
$Comp
L GND #PWR085
U 1 1 5AD0A5F5
P 8100 4300
F 0 "#PWR085" H 8100 4050 50  0001 C CNN
F 1 "GND" H 8100 4150 50  0000 C CNN
F 2 "" H 8100 4300 50  0001 C CNN
F 3 "" H 8100 4300 50  0001 C CNN
	1    8100 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8150 4200 8100 4200
Wire Wire Line
	8100 4200 8100 4300
$Comp
L APA102 U8
U 1 1 5AD0A765
P 2750 5350
F 0 "U8" H 2750 5250 50  0000 C CNN
F 1 "APA102" H 2750 5450 50  0000 C CNN
F 2 "NFC_Toy_Lib:APA102_hand_solder" H 2750 5350 50  0001 C CNN
F 3 "DOCUMENTATION" H 2750 5350 50  0001 C CNN
	1    2750 5350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR086
U 1 1 5AD0A76B
P 3600 5950
F 0 "#PWR086" H 3600 5700 50  0001 C CNN
F 1 "GND" H 3600 5800 50  0000 C CNN
F 2 "" H 3600 5950 50  0001 C CNN
F 3 "" H 3600 5950 50  0001 C CNN
	1    3600 5950
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR087
U 1 1 5AD0A771
P 3600 5550
F 0 "#PWR087" H 3600 5400 50  0001 C CNN
F 1 "+5V" H 3600 5690 50  0000 C CNN
F 2 "" H 3600 5550 50  0001 C CNN
F 3 "" H 3600 5550 50  0001 C CNN
	1    3600 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 5550 3600 5550
Wire Wire Line
	3500 5350 4050 5350
Wire Wire Line
	3500 5250 4050 5250
$Comp
L C C37
U 1 1 5AD0A77A
P 3600 5750
F 0 "C37" H 3625 5850 50  0000 L CNN
F 1 "0.1u" H 3625 5650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 3638 5600 50  0001 C CNN
F 3 "" H 3600 5750 50  0001 C CNN
	1    3600 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 5950 3600 5900
Wire Wire Line
	3600 5550 3600 5600
$Comp
L GND #PWR088
U 1 1 5AD0A782
P 1950 5650
F 0 "#PWR088" H 1950 5400 50  0001 C CNN
F 1 "GND" H 1950 5500 50  0000 C CNN
F 2 "" H 1950 5650 50  0001 C CNN
F 3 "" H 1950 5650 50  0001 C CNN
	1    1950 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 5550 1950 5550
Wire Wire Line
	1950 5550 1950 5650
$Comp
L APA102 U11
U 1 1 5AD0A78A
P 4800 5350
F 0 "U11" H 4800 5250 50  0000 C CNN
F 1 "APA102" H 4800 5450 50  0000 C CNN
F 2 "NFC_Toy_Lib:APA102_hand_solder" H 4800 5350 50  0001 C CNN
F 3 "DOCUMENTATION" H 4800 5350 50  0001 C CNN
	1    4800 5350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR089
U 1 1 5AD0A790
P 5650 5950
F 0 "#PWR089" H 5650 5700 50  0001 C CNN
F 1 "GND" H 5650 5800 50  0000 C CNN
F 2 "" H 5650 5950 50  0001 C CNN
F 3 "" H 5650 5950 50  0001 C CNN
	1    5650 5950
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR090
U 1 1 5AD0A796
P 5650 5550
F 0 "#PWR090" H 5650 5400 50  0001 C CNN
F 1 "+5V" H 5650 5690 50  0000 C CNN
F 2 "" H 5650 5550 50  0001 C CNN
F 3 "" H 5650 5550 50  0001 C CNN
	1    5650 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 5550 5650 5550
Wire Wire Line
	5550 5350 6100 5350
Wire Wire Line
	5550 5250 6100 5250
$Comp
L C C42
U 1 1 5AD0A79F
P 5650 5750
F 0 "C42" H 5675 5850 50  0000 L CNN
F 1 "0.1u" H 5675 5650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 5688 5600 50  0001 C CNN
F 3 "" H 5650 5750 50  0001 C CNN
	1    5650 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 5950 5650 5900
Wire Wire Line
	5650 5550 5650 5600
$Comp
L GND #PWR091
U 1 1 5AD0A7A7
P 4000 5650
F 0 "#PWR091" H 4000 5400 50  0001 C CNN
F 1 "GND" H 4000 5500 50  0000 C CNN
F 2 "" H 4000 5650 50  0001 C CNN
F 3 "" H 4000 5650 50  0001 C CNN
	1    4000 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 5550 4000 5550
Wire Wire Line
	4000 5550 4000 5650
$Comp
L APA102 U15
U 1 1 5AD0A7AF
P 6850 5350
F 0 "U15" H 6850 5250 50  0000 C CNN
F 1 "APA102" H 6850 5450 50  0000 C CNN
F 2 "NFC_Toy_Lib:APA102_hand_solder" H 6850 5350 50  0001 C CNN
F 3 "DOCUMENTATION" H 6850 5350 50  0001 C CNN
	1    6850 5350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR092
U 1 1 5AD0A7B5
P 7700 5950
F 0 "#PWR092" H 7700 5700 50  0001 C CNN
F 1 "GND" H 7700 5800 50  0000 C CNN
F 2 "" H 7700 5950 50  0001 C CNN
F 3 "" H 7700 5950 50  0001 C CNN
	1    7700 5950
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR093
U 1 1 5AD0A7BB
P 7700 5550
F 0 "#PWR093" H 7700 5400 50  0001 C CNN
F 1 "+5V" H 7700 5690 50  0000 C CNN
F 2 "" H 7700 5550 50  0001 C CNN
F 3 "" H 7700 5550 50  0001 C CNN
	1    7700 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 5550 7700 5550
Wire Wire Line
	7600 5350 8150 5350
Wire Wire Line
	7600 5250 8150 5250
$Comp
L C C45
U 1 1 5AD0A7C4
P 7700 5750
F 0 "C45" H 7725 5850 50  0000 L CNN
F 1 "0.1u" H 7725 5650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 7738 5600 50  0001 C CNN
F 3 "" H 7700 5750 50  0001 C CNN
	1    7700 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 5950 7700 5900
Wire Wire Line
	7700 5550 7700 5600
$Comp
L GND #PWR094
U 1 1 5AD0A7CC
P 6050 5650
F 0 "#PWR094" H 6050 5400 50  0001 C CNN
F 1 "GND" H 6050 5500 50  0000 C CNN
F 2 "" H 6050 5650 50  0001 C CNN
F 3 "" H 6050 5650 50  0001 C CNN
	1    6050 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 5550 6050 5550
Wire Wire Line
	6050 5550 6050 5650
$Comp
L APA102 U18
U 1 1 5AD0A7D4
P 8900 5350
F 0 "U18" H 8900 5250 50  0000 C CNN
F 1 "APA102" H 8900 5450 50  0000 C CNN
F 2 "NFC_Toy_Lib:APA102_hand_solder" H 8900 5350 50  0001 C CNN
F 3 "DOCUMENTATION" H 8900 5350 50  0001 C CNN
	1    8900 5350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR095
U 1 1 5AD0A7DA
P 9750 5950
F 0 "#PWR095" H 9750 5700 50  0001 C CNN
F 1 "GND" H 9750 5800 50  0000 C CNN
F 2 "" H 9750 5950 50  0001 C CNN
F 3 "" H 9750 5950 50  0001 C CNN
	1    9750 5950
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR096
U 1 1 5AD0A7E0
P 9750 5550
F 0 "#PWR096" H 9750 5400 50  0001 C CNN
F 1 "+5V" H 9750 5690 50  0000 C CNN
F 2 "" H 9750 5550 50  0001 C CNN
F 3 "" H 9750 5550 50  0001 C CNN
	1    9750 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	9650 5550 9750 5550
$Comp
L C C48
U 1 1 5AD0A7E7
P 9750 5750
F 0 "C48" H 9775 5850 50  0000 L CNN
F 1 "0.1u" H 9775 5650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 9788 5600 50  0001 C CNN
F 3 "" H 9750 5750 50  0001 C CNN
	1    9750 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	9750 5950 9750 5900
Wire Wire Line
	9750 5550 9750 5600
$Comp
L GND #PWR097
U 1 1 5AD0A7EF
P 8100 5650
F 0 "#PWR097" H 8100 5400 50  0001 C CNN
F 1 "GND" H 8100 5500 50  0000 C CNN
F 2 "" H 8100 5650 50  0001 C CNN
F 3 "" H 8100 5650 50  0001 C CNN
	1    8100 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	8150 5550 8100 5550
Wire Wire Line
	8100 5550 8100 5650
Text GLabel 9950 2500 2    60   Output ~ 0
DotStar_DO_0
Text GLabel 9950 2700 2    60   Output ~ 0
DotStar_CO_0
Wire Wire Line
	9650 2550 9900 2550
Wire Wire Line
	9900 2550 9900 2500
Wire Wire Line
	9900 2500 9950 2500
Wire Wire Line
	9650 2650 9900 2650
Wire Wire Line
	9900 2650 9900 2700
Wire Wire Line
	9900 2700 9950 2700
Text GLabel 1850 3850 0    60   Input ~ 0
DotStar_DO_0
Text GLabel 1850 4050 0    60   Input ~ 0
DotStar_CO_0
Wire Wire Line
	1850 3850 2000 3850
Wire Wire Line
	2000 3850 2000 3900
Wire Wire Line
	1850 4050 2000 4050
Wire Wire Line
	2000 4050 2000 4000
Text GLabel 9950 3850 2    60   Output ~ 0
DotStar_DO_1
Text GLabel 9950 4050 2    60   Output ~ 0
DotStar_CO_1
Wire Wire Line
	9650 3900 9900 3900
Wire Wire Line
	9900 3900 9900 3850
Wire Wire Line
	9900 3850 9950 3850
Wire Wire Line
	9650 4000 9900 4000
Wire Wire Line
	9900 4000 9900 4050
Wire Wire Line
	9900 4050 9950 4050
Text GLabel 1850 5200 0    60   Input ~ 0
DotStar_DO_1
Text GLabel 1850 5400 0    60   Input ~ 0
DotStar_CO_1
Wire Wire Line
	1850 5200 2000 5200
Wire Wire Line
	2000 5200 2000 5250
Wire Wire Line
	1850 5400 2000 5400
Wire Wire Line
	2000 5400 2000 5350
NoConn ~ 9650 5250
NoConn ~ 9650 5350
Text GLabel 6700 1150 2    47   Output ~ 0
DotStar_Data_5V
Text GLabel 6700 1250 2    47   Output ~ 0
DotStar_Clk_5V
Wire Wire Line
	1850 2500 2000 2500
Wire Wire Line
	2000 2500 2000 2550
Wire Wire Line
	1850 2700 2000 2700
Wire Wire Line
	2000 2700 2000 2650
$Comp
L +5V #PWR098
U 1 1 5AD0CD49
P 4000 1050
F 0 "#PWR098" H 4000 900 50  0001 C CNN
F 1 "+5V" H 4000 1190 50  0000 C CNN
F 2 "" H 4000 1050 50  0001 C CNN
F 3 "" H 4000 1050 50  0001 C CNN
	1    4000 1050
	1    0    0    -1  
$EndComp
$Comp
L C C38
U 1 1 5AD0D043
P 4000 1450
F 0 "C38" H 4025 1550 50  0000 L CNN
F 1 "0.1u" H 4025 1350 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 4038 1300 50  0001 C CNN
F 3 "" H 4000 1450 50  0001 C CNN
	1    4000 1450
	1    0    0    -1  
$EndComp
$Comp
L C C39
U 1 1 5AD0D0DD
P 4350 1450
F 0 "C39" H 4375 1550 50  0000 L CNN
F 1 "0.1u" H 4375 1350 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 4388 1300 50  0001 C CNN
F 3 "" H 4350 1450 50  0001 C CNN
	1    4350 1450
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR099
U 1 1 5AD0D148
P 4350 1050
F 0 "#PWR099" H 4350 900 50  0001 C CNN
F 1 "+3.3V" H 4350 1190 50  0000 C CNN
F 2 "" H 4350 1050 50  0001 C CNN
F 3 "" H 4350 1050 50  0001 C CNN
	1    4350 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 1050 4000 1300
Wire Wire Line
	4350 1050 4350 1300
Wire Wire Line
	4000 1250 5200 1250
Connection ~ 4000 1250
Wire Wire Line
	4350 1150 5200 1150
Connection ~ 4350 1150
$Comp
L GND #PWR0100
U 1 1 5AD0D656
P 4350 1700
F 0 "#PWR0100" H 4350 1450 50  0001 C CNN
F 1 "GND" H 4350 1550 50  0000 C CNN
F 2 "" H 4350 1700 50  0001 C CNN
F 3 "" H 4350 1700 50  0001 C CNN
	1    4350 1700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR0101
U 1 1 5AD0D739
P 4000 1700
F 0 "#PWR0101" H 4000 1450 50  0001 C CNN
F 1 "GND" H 4000 1550 50  0000 C CNN
F 2 "" H 4000 1700 50  0001 C CNN
F 3 "" H 4000 1700 50  0001 C CNN
	1    4000 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 1600 4000 1700
Wire Wire Line
	4350 1600 4350 1700
$Comp
L SN74LVC2T45 U12
U 1 1 5AD0DD22
P 5900 1450
F 0 "U12" H 5850 2000 50  0000 L BNN
F 1 "SN74LVC2T45" H 5650 650 50  0000 L BNN
F 2 "NFC_Toy_Lib:SOP50P310X90-8N" H 5900 1450 50  0001 L BNN
F 3 "VFSOP-8 Texas Instruments" H 5900 1450 50  0001 L BNN
F 4 "Texas Instruments" H 5900 1450 50  0001 L BNN "Field4"
F 5 "https://www.digikey.com/product-detail/en/texas-instruments/SN74LVC2T45/296-32331-1-ND/3506170?WT.z_cid=ref_snapeda&utm_source=snapeda&utm_medium=aggregator&utm_campaign=buynow" H 5900 1450 50  0001 L BNN "Field5"
F 6 "296-32331-1-ND" H 5900 1450 50  0001 L BNN "Field6"
F 7 "2-Bit Dual Supply Transceiver with Configurable Voltage-Level Shifting and 3-State Outputs 8-VSSOP -40 to 85" H 5900 1450 50  0001 L BNN "Field7"
F 8 "SN74LVC2T45" H 5900 1450 50  0001 L BNN "Field8"
	1    5900 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 1450 5100 1450
Wire Wire Line
	5100 1450 5100 1150
Connection ~ 5100 1150
$Comp
L GND #PWR0102
U 1 1 5AD0E6A5
P 5100 2000
F 0 "#PWR0102" H 5100 1750 50  0001 C CNN
F 1 "GND" H 5100 1850 50  0000 C CNN
F 2 "" H 5100 2000 50  0001 C CNN
F 3 "" H 5100 2000 50  0001 C CNN
	1    5100 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 2000 5100 1950
Wire Wire Line
	5100 1950 5200 1950
Text GLabel 5200 1650 0    47   Input ~ 0
DotStar_Data_3V
Text GLabel 5200 1750 0    47   Input ~ 0
DotStar_Clk_3V
Wire Wire Line
	6600 1150 6700 1150
Wire Wire Line
	6600 1250 6700 1250
Text GLabel 1850 2500 0    60   Input ~ 0
DotStar_Data_5V
Text GLabel 1850 2700 0    60   Input ~ 0
DotStar_Clk_5V
Wire Wire Line
	6650 1090 6650 1150
Connection ~ 6650 1150
Wire Wire Line
	6650 1310 6650 1250
Connection ~ 6650 1250
$Comp
L TEST TP14
U 1 1 5AD3BF34
P 6650 1090
F 0 "TP14" H 6650 1390 50  0000 C BNN
F 1 "TEST" H 6650 1340 50  0000 C CNN
F 2 "NFC_Toy_Lib:Test_Point" H 6650 1090 50  0001 C CNN
F 3 "" H 6650 1090 50  0001 C CNN
	1    6650 1090
	1    0    0    -1  
$EndComp
$Comp
L TEST TP15
U 1 1 5AD3C1F4
P 6650 1310
F 0 "TP15" H 6650 1610 50  0000 C BNN
F 1 "TEST" H 6650 1560 50  0000 C CNN
F 2 "NFC_Toy_Lib:Test_Point" H 6650 1310 50  0001 C CNN
F 3 "" H 6650 1310 50  0001 C CNN
	1    6650 1310
	-1   0    0    1   
$EndComp
$EndSCHEMATC
