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
LIBS:JLink_Mini
LIBS:JLink_Adpater-cache
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
L JLink_Mini J1
U 1 1 5AF195CD
P 4410 3740
F 0 "J1" H 4410 3640 50  0000 C CNN
F 1 "JLink_Mini" H 4410 3740 50  0000 C CNN
F 2 "NFC_Toy_Lib:JLink_Mini_0p05_inverted" H 4410 3740 50  0001 C CNN
F 3 "DOCUMENTATION" H 4410 3740 50  0001 C CNN
	1    4410 3740
	1    0    0    -1  
$EndComp
$Comp
L JLink_Mini J2
U 1 1 5AF1968D
P 7090 3750
F 0 "J2" H 7090 3650 50  0000 C CNN
F 1 "JLink_Mini" H 7090 3750 50  0000 C CNN
F 2 "NFC_Toy_Lib:J-Link_0p1in_inverted" H 7090 3750 50  0001 C CNN
F 3 "DOCUMENTATION" H 7090 3750 50  0001 C CNN
	1    7090 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3530 3200 6210 3200
Wire Wire Line
	6210 3200 6210 3550
Wire Wire Line
	6210 3550 6340 3550
Wire Wire Line
	3530 3200 3530 3540
Wire Wire Line
	3530 3540 3660 3540
Wire Wire Line
	3660 3640 3430 3640
Wire Wire Line
	3430 3640 3430 3130
Wire Wire Line
	3430 3130 6120 3130
Wire Wire Line
	6120 3130 6120 3650
Wire Wire Line
	6120 3650 6340 3650
Wire Wire Line
	3660 3740 3320 3740
Wire Wire Line
	3320 3740 3320 3030
Wire Wire Line
	3320 3030 6020 3030
Wire Wire Line
	6020 3030 6020 3750
Wire Wire Line
	6020 3750 6340 3750
Wire Wire Line
	3660 3840 3200 3840
Wire Wire Line
	3200 3840 3200 2920
Wire Wire Line
	3200 2920 5890 2920
Wire Wire Line
	5890 2920 5890 3850
Wire Wire Line
	5890 3850 6340 3850
Wire Wire Line
	3660 3940 3060 3940
Wire Wire Line
	3060 3940 3060 2780
Wire Wire Line
	3060 2780 5770 2780
Wire Wire Line
	5770 2780 5770 3950
Wire Wire Line
	5770 3950 6340 3950
Wire Wire Line
	5160 3540 5630 3540
Wire Wire Line
	5630 3540 5630 4240
Wire Wire Line
	5630 4240 8000 4240
Wire Wire Line
	8000 4240 8000 3550
Wire Wire Line
	8000 3550 7840 3550
Wire Wire Line
	5160 3640 5570 3640
Wire Wire Line
	5570 3640 5570 4290
Wire Wire Line
	5570 4290 8060 4290
Wire Wire Line
	8060 4290 8060 3650
Wire Wire Line
	8060 3650 7840 3650
Wire Wire Line
	5160 3740 5500 3740
Wire Wire Line
	5500 3740 5500 4350
Wire Wire Line
	5500 4350 8130 4350
Wire Wire Line
	8130 4350 8130 3750
Wire Wire Line
	8130 3750 7840 3750
Wire Wire Line
	5160 3840 5430 3840
Wire Wire Line
	5430 3840 5430 4420
Wire Wire Line
	5430 4420 8210 4420
Wire Wire Line
	8210 4420 8210 3850
Wire Wire Line
	8210 3850 7840 3850
Wire Wire Line
	5160 3940 5360 3940
Wire Wire Line
	5360 3940 5360 4490
Wire Wire Line
	5360 4490 8300 4490
Wire Wire Line
	8300 4490 8300 3950
Wire Wire Line
	8300 3950 7840 3950
$Comp
L R R1
U 1 1 5B0010A4
P 5480 3380
F 0 "R1" V 5560 3380 50  0000 C CNN
F 1 "100k" V 5480 3380 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5410 3380 50  0001 C CNN
F 3 "" H 5480 3380 50  0001 C CNN
	1    5480 3380
	-1   0    0    1   
$EndComp
Wire Wire Line
	5480 3530 5480 3540
Connection ~ 5480 3540
Wire Wire Line
	5480 3230 5480 3200
Connection ~ 5480 3200
$EndSCHEMATC
