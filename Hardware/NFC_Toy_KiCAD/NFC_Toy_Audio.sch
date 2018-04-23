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
LIBS:mechanical
LIBS:NFC_Toy-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 4
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
L MAX98357 U4
U 1 1 5ACDFE57
P 7040 3630
F 0 "U4" H 7040 3530 50  0000 C CNN
F 1 "MAX98357" H 7040 3730 50  0000 C CNN
F 2 "NFC_Toy_Lib:QFN50P300X300X80-17N" H 7040 3630 50  0001 C CNN
F 3 "DOCUMENTATION" H 7040 3630 50  0001 C CNN
	1    7040 3630
	1    0    0    -1  
$EndComp
Text GLabel 6040 3230 0    47   Input ~ 0
I2S_SD
Text GLabel 6040 3930 0    47   Input ~ 0
I2S_WS
Text GLabel 6040 4030 0    47   Input ~ 0
I2S_CK
Wire Wire Line
	6040 3930 6140 3930
Wire Wire Line
	6040 4030 6140 4030
Wire Wire Line
	6040 3230 6140 3230
NoConn ~ 6140 3530
NoConn ~ 6140 3630
NoConn ~ 6140 3730
NoConn ~ 6140 3830
$Comp
L GND #PWR034
U 1 1 5ACDFE68
P 7040 4580
F 0 "#PWR034" H 7040 4330 50  0001 C CNN
F 1 "GND" H 7040 4430 50  0000 C CNN
F 2 "" H 7040 4580 50  0001 C CNN
F 3 "" H 7040 4580 50  0001 C CNN
	1    7040 4580
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR035
U 1 1 5ACDFE6E
P 7090 2380
F 0 "#PWR035" H 7090 2230 50  0001 C CNN
F 1 "+5V" H 7090 2520 50  0000 C CNN
F 2 "" H 7090 2380 50  0001 C CNN
F 3 "" H 7090 2380 50  0001 C CNN
	1    7090 2380
	1    0    0    -1  
$EndComp
Wire Wire Line
	7040 2730 7040 2380
Wire Wire Line
	7040 2380 7700 2380
Wire Wire Line
	7140 2380 7140 2730
Connection ~ 7090 2380
Wire Wire Line
	7040 4530 7040 4580
Wire Wire Line
	6940 4530 6940 4580
Wire Wire Line
	6940 4580 7290 4580
Wire Wire Line
	7140 4580 7140 4530
Connection ~ 7040 4580
Text GLabel 6040 3330 0    47   Input ~ 0
AUDIO_SD_N
Wire Wire Line
	6040 3330 6140 3330
$Comp
L R R11
U 1 1 5ACDFE7F
P 5390 3230
F 0 "R11" V 5470 3230 50  0000 C CNN
F 1 "DNP" V 5390 3230 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5320 3230 50  0001 C CNN
F 3 "" H 5390 3230 50  0001 C CNN
	1    5390 3230
	1    0    0    -1  
$EndComp
$Comp
L R R12
U 1 1 5ACDFE86
P 5390 3630
F 0 "R12" V 5470 3630 50  0000 C CNN
F 1 "DNP" V 5390 3630 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5320 3630 50  0001 C CNN
F 3 "" H 5390 3630 50  0001 C CNN
	1    5390 3630
	1    0    0    -1  
$EndComp
Wire Wire Line
	5390 3380 5390 3480
Wire Wire Line
	6140 3430 5390 3430
Connection ~ 5390 3430
$Comp
L GND #PWR036
U 1 1 5ACDFE90
P 5390 3830
F 0 "#PWR036" H 5390 3580 50  0001 C CNN
F 1 "GND" H 5390 3680 50  0000 C CNN
F 2 "" H 5390 3830 50  0001 C CNN
F 3 "" H 5390 3830 50  0001 C CNN
	1    5390 3830
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR037
U 1 1 5ACDFE96
P 5390 3030
F 0 "#PWR037" H 5390 2880 50  0001 C CNN
F 1 "+5V" H 5390 3170 50  0000 C CNN
F 2 "" H 5390 3030 50  0001 C CNN
F 3 "" H 5390 3030 50  0001 C CNN
	1    5390 3030
	1    0    0    -1  
$EndComp
Wire Wire Line
	5390 3080 5390 3030
Wire Wire Line
	5390 3780 5390 3830
Text Notes 6290 2980 2    47   ~ 0
GAIN_SLOT:\n    100k to GND - 15dB\n    GND - 12dB\n    NC - 9dB\n    VDD - 6dB\n    100k to VDD - 3dB
$Comp
L Pad J4
U 1 1 5ACE08A3
P 8540 3730
F 0 "J4" H 8540 3530 60  0000 C CNN
F 1 "SPKR-" H 8540 3730 60  0000 C CNN
F 2 "NFC_Toy_Lib:Pad_SMD_Small" H 8540 3730 60  0001 C CNN
F 3 "" H 8540 3730 60  0001 C CNN
	1    8540 3730
	-1   0    0    -1  
$EndComp
$Comp
L Pad J5
U 1 1 5ACE08AA
P 8540 4080
F 0 "J5" H 8540 3880 60  0000 C CNN
F 1 "SPKR+" H 8540 4080 60  0000 C CNN
F 2 "NFC_Toy_Lib:Pad_SMD_Small" H 8540 4080 60  0001 C CNN
F 3 "" H 8540 4080 60  0001 C CNN
	1    8540 4080
	-1   0    0    -1  
$EndComp
$Comp
L 503182-1852 J6
U 1 1 5ACE08EE
P 3240 3580
F 0 "J6" H 3240 4303 50  0000 L BNN
F 1 "503182-1852" H 3240 2757 50  0000 L BNN
F 2 "NFC_Toy_Lib:MOLEX_503182-1852" H 3240 3580 50  0001 L BNN
F 3 "5031821852" H 3240 3580 50  0001 L BNN
F 4 "Unavailable" H 3240 3580 50  0001 L BNN "Field4"
F 5 "1.10mm Pitch microSD Memory Card Connector, Normal Mount Surface Mount, Push-Push Type, 1.45mm Height, Anti-card fly-out" H 3240 3580 50  0001 L BNN "Field5"
F 6 "None" H 3240 3580 50  0001 L BNN "Field6"
F 7 "Molex" H 3240 3580 50  0001 L BNN "Field7"
F 8 "None" H 3240 3580 50  0001 L BNN "Field8"
	1    3240 3580
	-1   0    0    -1  
$EndComp
NoConn ~ 3440 2980
Text GLabel 3690 3080 2    47   Input ~ 0
SPI_NSS
Text GLabel 3690 3380 2    47   Input ~ 0
SPI_SCK
Text GLabel 3690 3580 2    47   Output ~ 0
SPI_MISO
Text GLabel 3690 3180 2    47   Input ~ 0
SPI_MOSI
Wire Wire Line
	3690 3580 3440 3580
Wire Wire Line
	3690 3380 3440 3380
Wire Wire Line
	3690 3180 3440 3180
Wire Wire Line
	3690 3080 3440 3080
$Comp
L +3.3V #PWR038
U 1 1 5ACE0C10
P 3590 2450
F 0 "#PWR038" H 3590 2300 50  0001 C CNN
F 1 "+3.3V" H 3590 2590 50  0000 C CNN
F 2 "" H 3590 2450 50  0001 C CNN
F 3 "" H 3590 2450 50  0001 C CNN
	1    3590 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3590 3280 3440 3280
Wire Wire Line
	3590 2450 3590 3280
Wire Wire Line
	3440 3480 3590 3480
Wire Wire Line
	3590 3480 3590 4230
Text GLabel 3690 3880 2    47   Output ~ 0
SD_DETECT
Text GLabel 3690 3980 2    47   Output ~ 0
SD_SW
NoConn ~ 3440 3680
Wire Wire Line
	3440 3880 3690 3880
Wire Wire Line
	3440 3980 3690 3980
$Comp
L GND #PWR039
U 1 1 5ACE0D8F
P 3590 4230
F 0 "#PWR039" H 3590 3980 50  0001 C CNN
F 1 "GND" H 3590 4080 50  0000 C CNN
F 2 "" H 3590 4230 50  0001 C CNN
F 3 "" H 3590 4230 50  0001 C CNN
	1    3590 4230
	1    0    0    -1  
$EndComp
Wire Wire Line
	3440 4180 3590 4180
Connection ~ 3590 4180
Wire Wire Line
	7940 3930 8090 3930
Wire Wire Line
	8090 3930 8090 3830
Wire Wire Line
	8090 3830 8290 3830
Wire Wire Line
	7940 4030 8090 4030
Wire Wire Line
	8090 4030 8090 4180
Wire Wire Line
	8090 4180 8290 4180
$Comp
L C C14
U 1 1 5AD11D71
P 3780 2640
F 0 "C14" H 3805 2740 50  0000 L CNN
F 1 "0.1u" H 3805 2540 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 3818 2490 50  0001 C CNN
F 3 "" H 3780 2640 50  0001 C CNN
	1    3780 2640
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR040
U 1 1 5AD11F0E
P 3780 2830
F 0 "#PWR040" H 3780 2580 50  0001 C CNN
F 1 "GND" H 3780 2680 50  0000 C CNN
F 2 "" H 3780 2830 50  0001 C CNN
F 3 "" H 3780 2830 50  0001 C CNN
	1    3780 2830
	1    0    0    -1  
$EndComp
Wire Wire Line
	3780 2490 3780 2470
Wire Wire Line
	3780 2470 3590 2470
Connection ~ 3590 2470
Wire Wire Line
	3780 2790 3780 2830
$Comp
L C C15
U 1 1 5AD1240E
P 7370 2570
F 0 "C15" H 7395 2670 50  0000 L CNN
F 1 "0.1u" H 7395 2470 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 7408 2420 50  0001 C CNN
F 3 "" H 7370 2570 50  0001 C CNN
	1    7370 2570
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR041
U 1 1 5AD12414
P 7370 2750
F 0 "#PWR041" H 7370 2500 50  0001 C CNN
F 1 "GND" H 7370 2600 50  0000 C CNN
F 2 "" H 7370 2750 50  0001 C CNN
F 3 "" H 7370 2750 50  0001 C CNN
	1    7370 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7370 2720 7370 2750
$Comp
L C C16
U 1 1 5AD12566
P 7700 2570
F 0 "C16" H 7725 2670 50  0000 L CNN
F 1 "10u" H 7725 2470 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 7738 2420 50  0001 C CNN
F 3 "" H 7700 2570 50  0001 C CNN
	1    7700 2570
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR042
U 1 1 5AD125D7
P 7700 2760
F 0 "#PWR042" H 7700 2510 50  0001 C CNN
F 1 "GND" H 7700 2610 50  0000 C CNN
F 2 "" H 7700 2760 50  0001 C CNN
F 3 "" H 7700 2760 50  0001 C CNN
	1    7700 2760
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 2720 7700 2760
Wire Wire Line
	7370 2380 7370 2420
Connection ~ 7140 2380
Wire Wire Line
	7700 2380 7700 2420
Connection ~ 7370 2380
Connection ~ 7140 4580
Wire Wire Line
	7290 4580 7290 4530
$EndSCHEMATC
