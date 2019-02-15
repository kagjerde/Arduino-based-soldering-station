EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
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
LIBS:modules
LIBS:Weller loddestasjon-cache
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
L Arduino_Nano_v3.x A1
U 1 1 5B0DD0AF
P 4800 2700
F 0 "A1" H 4600 3725 50  0000 R CNN
F 1 "Arduino_Nano_v3.x" H 4600 3650 50  0000 R CNN
F 2 "Modules:Arduino_Nano" H 4950 1750 50  0001 L CNN
F 3 "" H 4800 1700 50  0001 C CNN
	1    4800 2700
	1    0    0    -1  
$EndComp
$Comp
L OPA333xxDBV U1
U 1 1 5B0DE9B7
P 2850 4500
F 0 "U1" H 2850 4700 50  0000 L CNN
F 1 "OPA333xxDBV" H 2850 4300 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-5" H 2850 4500 50  0001 C CNN
F 3 "" H 2850 4700 50  0001 C CNN
	1    2850 4500
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 5B0DF57F
P 2350 4800
F 0 "R7" V 2430 4800 50  0000 C CNN
F 1 "R" V 2350 4800 50  0000 C CNN
F 2 "" V 2280 4800 50  0001 C CNN
F 3 "" H 2350 4800 50  0001 C CNN
	1    2350 4800
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 5B0DF67E
P 1900 4400
F 0 "R4" V 1980 4400 50  0000 C CNN
F 1 "R" V 1900 4400 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 1830 4400 50  0001 C CNN
F 3 "" H 1900 4400 50  0001 C CNN
	1    1900 4400
	0    1    1    0   
$EndComp
$Comp
L R R5
U 1 1 5B0DF73F
P 2200 4800
F 0 "R5" V 2280 4800 50  0000 C CNN
F 1 "R" V 2200 4800 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 2130 4800 50  0001 C CNN
F 3 "" H 2200 4800 50  0001 C CNN
	1    2200 4800
	1    0    0    -1  
$EndComp
$Comp
L R R8
U 1 1 5B0DF7AA
P 3000 3950
F 0 "R8" V 3080 3950 50  0000 C CNN
F 1 "R" V 3000 3950 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 2930 3950 50  0001 C CNN
F 3 "" H 3000 3950 50  0001 C CNN
	1    3000 3950
	0    1    1    0   
$EndComp
$Comp
L R R9
U 1 1 5B0DF83D
P 3500 4500
F 0 "R9" V 3580 4500 50  0000 C CNN
F 1 "R" V 3500 4500 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 3430 4500 50  0001 C CNN
F 3 "" H 3500 4500 50  0001 C CNN
	1    3500 4500
	0    1    1    0   
$EndComp
Wire Wire Line
	3150 4500 3350 4500
Wire Wire Line
	3150 3950 3200 3950
Wire Wire Line
	3200 3950 3200 4500
Connection ~ 3200 4500
Wire Wire Line
	2850 3950 2350 3950
Wire Wire Line
	2350 3950 2350 4650
Wire Wire Line
	2550 4600 2350 4600
Connection ~ 2350 4600
Wire Wire Line
	2050 4400 2550 4400
Wire Wire Line
	2200 4400 2200 4650
Connection ~ 2200 4400
$Comp
L C C2
U 1 1 5B0DFBF3
P 2050 4800
F 0 "C2" H 2075 4900 50  0000 L CNN
F 1 "C" H 2075 4700 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 2088 4650 50  0001 C CNN
F 3 "" H 2050 4800 50  0001 C CNN
	1    2050 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 4650 2050 4550
Wire Wire Line
	2050 4550 2200 4550
Connection ~ 2200 4550
Wire Wire Line
	2200 5050 2200 4950
Wire Wire Line
	1600 5050 3700 5050
Wire Wire Line
	2750 4800 2750 5100
Wire Wire Line
	2350 4950 2350 5050
Connection ~ 2350 5050
Wire Wire Line
	2050 5050 2050 4950
Connection ~ 2200 5050
$Comp
L GND #PWR01
U 1 1 5B0DFE4D
P 2750 5100
F 0 "#PWR01" H 2750 4850 50  0001 C CNN
F 1 "GND" H 2750 4950 50  0000 C CNN
F 2 "" H 2750 5100 50  0001 C CNN
F 3 "" H 2750 5100 50  0001 C CNN
	1    2750 5100
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 5B0DFEA7
P 3700 4800
F 0 "C3" H 3725 4900 50  0000 L CNN
F 1 "C" H 3725 4700 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 3738 4650 50  0001 C CNN
F 3 "" H 3700 4800 50  0001 C CNN
	1    3700 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 5050 3700 4950
Connection ~ 2750 5050
Wire Wire Line
	3700 4650 3700 4500
Wire Wire Line
	3650 4500 4100 4500
Wire Wire Line
	1250 4500 1600 4500
Wire Wire Line
	1600 4500 1600 5050
Connection ~ 2050 5050
Wire Wire Line
	1750 4400 1250 4400
$Comp
L IRF9540N Q1
U 1 1 5B0E0C22
P 1350 3500
F 0 "Q1" H 1600 3575 50  0000 L CNN
F 1 "IRF9540N" H 1600 3500 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-220-3_Vertical" H 1600 3425 50  0001 L CIN
F 3 "" H 1350 3500 50  0001 L CNN
	1    1350 3500
	-1   0    0    1   
$EndComp
$Comp
L BC547 Q2
U 1 1 5B0E0E99
P 1900 3750
F 0 "Q2" H 2100 3825 50  0000 L CNN
F 1 "BC547" H 2100 3750 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Narrow" H 2100 3675 50  0001 L CIN
F 3 "" H 1900 3750 50  0001 L CNN
	1    1900 3750
	-1   0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 5B0E0F24
P 1800 3250
F 0 "R2" V 1880 3250 50  0000 C CNN
F 1 "R" V 1800 3250 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 1730 3250 50  0001 C CNN
F 3 "" H 1800 3250 50  0001 C CNN
	1    1800 3250
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 5B0E0F97
P 2350 3750
F 0 "R6" V 2430 3750 50  0000 C CNN
F 1 "R" V 2350 3750 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 2280 3750 50  0001 C CNN
F 3 "" H 2350 3750 50  0001 C CNN
	1    2350 3750
	0    1    1    0   
$EndComp
Wire Wire Line
	1250 3700 1250 4300
Wire Wire Line
	1800 3400 1800 3550
Wire Wire Line
	1550 3500 1800 3500
Connection ~ 1800 3500
$Comp
L +12V #PWR02
U 1 1 5B0E135F
P 1250 2900
F 0 "#PWR02" H 1250 2750 50  0001 C CNN
F 1 "+12V" H 1250 3040 50  0000 C CNN
F 2 "" H 1250 2900 50  0001 C CNN
F 3 "" H 1250 2900 50  0001 C CNN
	1    1250 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 2900 1250 3300
Wire Wire Line
	1800 3100 1250 3100
Connection ~ 1250 3100
Wire Wire Line
	2200 3750 2100 3750
$Comp
L GND #PWR03
U 1 1 5B0E1529
P 1800 4050
F 0 "#PWR03" H 1800 3800 50  0001 C CNN
F 1 "GND" H 1800 3900 50  0000 C CNN
F 2 "" H 1800 4050 50  0001 C CNN
F 3 "" H 1800 4050 50  0001 C CNN
	1    1800 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 4050 1800 3950
Text GLabel 4100 4500 2    60   Output ~ 0
Temp
Connection ~ 3700 4500
Text GLabel 2600 3750 2    60   Input ~ 0
PWM
Wire Wire Line
	2600 3750 2500 3750
$Comp
L Conn_01x05 J4
U 1 1 5B0E2057
P 8100 2100
F 0 "J4" H 8100 2400 50  0000 C CNN
F 1 "Conn_01x05" H 8100 1800 50  0000 C CNN
F 2 "Connectors_Molex:Molex_KK-6410-05_05x2.54mm_Straight" H 8100 2100 50  0001 C CNN
F 3 "" H 8100 2100 50  0001 C CNN
	1    8100 2100
	1    0    0    -1  
$EndComp
Text GLabel 7750 1900 0    60   BiDi ~ 0
SW
Text GLabel 7750 2100 0    60   BiDi ~ 0
A
Text GLabel 7750 2300 0    60   BiDi ~ 0
B
$Comp
L GND #PWR04
U 1 1 5B0E21B7
P 7450 2300
F 0 "#PWR04" H 7450 2050 50  0001 C CNN
F 1 "GND" H 7450 2150 50  0000 C CNN
F 2 "" H 7450 2300 50  0001 C CNN
F 3 "" H 7450 2300 50  0001 C CNN
	1    7450 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 1900 7750 1900
Wire Wire Line
	7900 2000 7450 2000
Wire Wire Line
	7450 2000 7450 2300
Wire Wire Line
	7900 2200 7450 2200
Connection ~ 7450 2200
Wire Wire Line
	7900 2300 7750 2300
Wire Wire Line
	7900 2100 7750 2100
$Comp
L GND #PWR05
U 1 1 5B0E23F5
P 6450 3000
F 0 "#PWR05" H 6450 2750 50  0001 C CNN
F 1 "GND" H 6450 2850 50  0000 C CNN
F 2 "" H 6450 3000 50  0001 C CNN
F 3 "" H 6450 3000 50  0001 C CNN
	1    6450 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 3000 8150 3000
Text GLabel 6900 3100 0    60   BiDi ~ 0
CS
Text GLabel 6900 3200 0    60   BiDi ~ 0
RESET
Text GLabel 6900 3300 0    60   BiDi ~ 0
A0
Text GLabel 6900 3400 0    60   BiDi ~ 0
SDA
Text GLabel 6900 3500 0    60   BiDi ~ 0
SCK
$Comp
L Conn_01x08 J5
U 1 1 5B0E25F2
P 8350 3200
F 0 "J5" H 8350 3600 50  0000 C CNN
F 1 "Conn_01x08" H 8350 2700 50  0000 C CNN
F 2 "Connectors_Molex:Molex_KK-6410-08_08x2.54mm_Straight" H 8350 3200 50  0001 C CNN
F 3 "" H 8350 3200 50  0001 C CNN
	1    8350 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6900 3100 7100 3100
Wire Wire Line
	6900 3200 7100 3200
Wire Wire Line
	6900 3300 7100 3300
Wire Wire Line
	6900 3400 7100 3400
Wire Wire Line
	6900 3500 7100 3500
Text GLabel 5500 3000 2    60   Input ~ 0
Temp
Wire Wire Line
	5500 3000 5300 3000
$Comp
L +3V3 #PWR06
U 1 1 5B0E28AB
P 8050 2900
F 0 "#PWR06" H 8050 2750 50  0001 C CNN
F 1 "+3V3" H 8050 3040 50  0000 C CNN
F 2 "" H 8050 2900 50  0001 C CNN
F 3 "" H 8050 2900 50  0001 C CNN
	1    8050 2900
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR07
U 1 1 5B0E293F
P 4900 1500
F 0 "#PWR07" H 4900 1350 50  0001 C CNN
F 1 "+3V3" H 4900 1640 50  0000 C CNN
F 2 "" H 4900 1500 50  0001 C CNN
F 3 "" H 4900 1500 50  0001 C CNN
	1    4900 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 1500 4900 1700
Text GLabel 4100 3000 0    60   Output ~ 0
PWM
Wire Wire Line
	4300 3000 4100 3000
$Comp
L R R15
U 1 1 5B0E2AAB
P 7500 3850
F 0 "R15" V 7580 3850 50  0000 C CNN
F 1 "R" V 7500 3850 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 7430 3850 50  0001 C CNN
F 3 "" H 7500 3850 50  0001 C CNN
	1    7500 3850
	1    0    0    -1  
$EndComp
$Comp
L R R16
U 1 1 5B0E2B1D
P 7600 3850
F 0 "R16" V 7680 3850 50  0000 C CNN
F 1 "R" V 7600 3850 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 7530 3850 50  0001 C CNN
F 3 "" H 7600 3850 50  0001 C CNN
	1    7600 3850
	1    0    0    -1  
$EndComp
$Comp
L R R17
U 1 1 5B0E2B6F
P 7700 3850
F 0 "R17" V 7780 3850 50  0000 C CNN
F 1 "R" V 7700 3850 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 7630 3850 50  0001 C CNN
F 3 "" H 7700 3850 50  0001 C CNN
	1    7700 3850
	1    0    0    -1  
$EndComp
$Comp
L R R18
U 1 1 5B0E2BBC
P 7800 3850
F 0 "R18" V 7880 3850 50  0000 C CNN
F 1 "R" V 7800 3850 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 7730 3850 50  0001 C CNN
F 3 "" H 7800 3850 50  0001 C CNN
	1    7800 3850
	1    0    0    -1  
$EndComp
$Comp
L R R10
U 1 1 5B0E2C14
P 7250 3100
F 0 "R10" V 7330 3100 50  0000 C CNN
F 1 "R" V 7250 3100 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 7180 3100 50  0001 C CNN
F 3 "" H 7250 3100 50  0001 C CNN
	1    7250 3100
	0    1    1    0   
$EndComp
$Comp
L R R11
U 1 1 5B0E2CCB
P 7250 3200
F 0 "R11" V 7330 3200 50  0000 C CNN
F 1 "R" V 7250 3200 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 7180 3200 50  0001 C CNN
F 3 "" H 7250 3200 50  0001 C CNN
	1    7250 3200
	0    1    1    0   
$EndComp
$Comp
L R R12
U 1 1 5B0E2D21
P 7250 3300
F 0 "R12" V 7330 3300 50  0000 C CNN
F 1 "R" V 7250 3300 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 7180 3300 50  0001 C CNN
F 3 "" H 7250 3300 50  0001 C CNN
	1    7250 3300
	0    1    1    0   
$EndComp
$Comp
L R R13
U 1 1 5B0E2D7E
P 7250 3400
F 0 "R13" V 7330 3400 50  0000 C CNN
F 1 "R" V 7250 3400 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 7180 3400 50  0001 C CNN
F 3 "" H 7250 3400 50  0001 C CNN
	1    7250 3400
	0    1    1    0   
$EndComp
$Comp
L R R14
U 1 1 5B0E2DDA
P 7250 3500
F 0 "R14" V 7330 3500 50  0000 C CNN
F 1 "R" V 7250 3500 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 7180 3500 50  0001 C CNN
F 3 "" H 7250 3500 50  0001 C CNN
	1    7250 3500
	0    1    1    0   
$EndComp
Wire Wire Line
	7400 3100 8150 3100
Wire Wire Line
	7400 3200 8150 3200
Wire Wire Line
	7400 3300 8150 3300
Wire Wire Line
	7400 3400 8150 3400
Wire Wire Line
	7400 3500 8150 3500
Wire Wire Line
	7500 3700 7500 3100
Connection ~ 7500 3100
Wire Wire Line
	7600 3700 7600 3200
Connection ~ 7600 3200
Wire Wire Line
	7700 3700 7700 3300
Connection ~ 7700 3300
Wire Wire Line
	7800 3700 7800 3400
Connection ~ 7800 3400
$Comp
L R R19
U 1 1 5B0E3486
P 7900 3850
F 0 "R19" V 7980 3850 50  0000 C CNN
F 1 "R" V 7900 3850 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 7830 3850 50  0001 C CNN
F 3 "" H 7900 3850 50  0001 C CNN
	1    7900 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 3700 7900 3500
Connection ~ 7900 3500
Wire Wire Line
	7900 4000 7900 4150
Wire Wire Line
	7500 4100 7900 4100
Wire Wire Line
	7500 4100 7500 4000
Wire Wire Line
	7600 4000 7600 4100
Connection ~ 7600 4100
Wire Wire Line
	7700 4000 7700 4100
Connection ~ 7700 4100
Wire Wire Line
	7800 4000 7800 4100
Connection ~ 7800 4100
$Comp
L GND #PWR08
U 1 1 5B0E3FEE
P 7900 4150
F 0 "#PWR08" H 7900 3900 50  0001 C CNN
F 1 "GND" H 7900 4000 50  0000 C CNN
F 2 "" H 7900 4150 50  0001 C CNN
F 3 "" H 7900 4150 50  0001 C CNN
	1    7900 4150
	1    0    0    -1  
$EndComp
Connection ~ 7900 4100
Wire Wire Line
	8150 2900 8050 2900
$Comp
L +3V3 #PWR09
U 1 1 5B0E4272
P 8050 3700
F 0 "#PWR09" H 8050 3550 50  0001 C CNN
F 1 "+3V3" H 8050 3840 50  0000 C CNN
F 2 "" H 8050 3700 50  0001 C CNN
F 3 "" H 8050 3700 50  0001 C CNN
	1    8050 3700
	-1   0    0    1   
$EndComp
Wire Wire Line
	8150 3600 8050 3600
Wire Wire Line
	8050 3600 8050 3700
Text GLabel 4100 3100 0    60   BiDi ~ 0
CS
Text GLabel 4100 3300 0    60   BiDi ~ 0
RESET
Text GLabel 4100 2900 0    60   BiDi ~ 0
A0
Text GLabel 4100 3200 0    60   BiDi ~ 0
SDA
Text GLabel 4100 3400 0    60   BiDi ~ 0
SCK
Text GLabel 4100 2500 0    60   BiDi ~ 0
SW
Text GLabel 4100 2300 0    60   BiDi ~ 0
A
Text GLabel 4100 2400 0    60   BiDi ~ 0
B
Wire Wire Line
	4100 2300 4300 2300
Wire Wire Line
	4300 2400 4100 2400
Wire Wire Line
	4300 2500 4100 2500
Wire Wire Line
	4300 2900 4100 2900
Wire Wire Line
	4300 3100 4100 3100
Wire Wire Line
	4300 3200 4100 3200
Wire Wire Line
	4300 3300 4100 3300
Wire Wire Line
	4300 3400 4100 3400
$Comp
L GND #PWR010
U 1 1 5B0E4A0C
P 4900 3800
F 0 "#PWR010" H 4900 3550 50  0001 C CNN
F 1 "GND" H 4900 3650 50  0000 C CNN
F 2 "" H 4900 3800 50  0001 C CNN
F 3 "" H 4900 3800 50  0001 C CNN
	1    4900 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 3700 4900 3800
Wire Wire Line
	4800 3700 4800 3750
Wire Wire Line
	4800 3750 4900 3750
Connection ~ 4900 3750
$Comp
L Screw_Terminal_01x02 J3
U 1 1 5B0E4B8E
P 8100 1300
F 0 "J3" H 8100 1400 50  0000 C CNN
F 1 "Screw_Terminal_01x02" H 8100 1100 50  0000 C CNN
F 2 "Connectors_Terminal_Blocks:TerminalBlock_bornier-2_P5.08mm" H 8100 1300 50  0001 C CNN
F 3 "" H 8100 1300 50  0001 C CNN
	1    8100 1300
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR011
U 1 1 5B0E4D41
P 4700 1100
F 0 "#PWR011" H 4700 950 50  0001 C CNN
F 1 "+12V" H 4700 1240 50  0000 C CNN
F 2 "" H 4700 1100 50  0001 C CNN
F 3 "" H 4700 1100 50  0001 C CNN
	1    4700 1100
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR012
U 1 1 5B0E4DD3
P 7650 1250
F 0 "#PWR012" H 7650 1100 50  0001 C CNN
F 1 "+12V" H 7650 1390 50  0000 C CNN
F 2 "" H 7650 1250 50  0001 C CNN
F 3 "" H 7650 1250 50  0001 C CNN
	1    7650 1250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 5B0E4ED3
P 7650 1400
F 0 "#PWR013" H 7650 1150 50  0001 C CNN
F 1 "GND" H 7650 1250 50  0000 C CNN
F 2 "" H 7650 1400 50  0001 C CNN
F 3 "" H 7650 1400 50  0001 C CNN
	1    7650 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 1300 7650 1300
Wire Wire Line
	7900 1400 7650 1400
$Comp
L D D1
U 1 1 5B0E52D1
P 4700 1350
F 0 "D1" H 4700 1450 50  0000 C CNN
F 1 "D" H 4700 1250 50  0000 C CNN
F 2 "Diodes_THT:D_A-405_P7.62mm_Horizontal" H 4700 1350 50  0001 C CNN
F 3 "" H 4700 1350 50  0001 C CNN
	1    4700 1350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4700 1100 4700 1200
Wire Wire Line
	4700 1500 4700 1700
$Comp
L CP C5
U 1 1 5B0E562F
P 4450 1450
F 0 "C5" H 4475 1550 50  0000 L CNN
F 1 "CP" H 4475 1350 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D5.0mm_P2.00mm" H 4488 1300 50  0001 C CNN
F 3 "" H 4450 1450 50  0001 C CNN
	1    4450 1450
	-1   0    0    1   
$EndComp
Wire Wire Line
	4200 1600 4700 1600
Connection ~ 4700 1600
$Comp
L GND #PWR014
U 1 1 5B0E57B8
P 4450 1150
F 0 "#PWR014" H 4450 900 50  0001 C CNN
F 1 "GND" H 4450 1000 50  0000 C CNN
F 2 "" H 4450 1150 50  0001 C CNN
F 3 "" H 4450 1150 50  0001 C CNN
	1    4450 1150
	-1   0    0    1   
$EndComp
Wire Wire Line
	4450 1150 4450 1300
$Comp
L Screw_Terminal_01x03 J1
U 1 1 5B0E5AB2
P 1050 4400
F 0 "J1" H 1050 4600 50  0000 C CNN
F 1 "Screw_Terminal_01x03" H 1050 4200 50  0000 C CNN
F 2 "Connectors_Terminal_Blocks:TerminalBlock_bornier-3_P5.08mm" H 1050 4400 50  0001 C CNN
F 3 "" H 1050 4400 50  0001 C CNN
	1    1050 4400
	-1   0    0    1   
$EndComp
$Comp
L C C4
U 1 1 5B0E5D63
P 4200 1450
F 0 "C4" H 4225 1550 50  0000 L CNN
F 1 "C" H 4225 1350 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 4238 1300 50  0001 C CNN
F 3 "" H 4200 1450 50  0001 C CNN
	1    4200 1450
	1    0    0    -1  
$EndComp
Connection ~ 4450 1600
Wire Wire Line
	4200 1300 4200 1250
Wire Wire Line
	4200 1250 4450 1250
Connection ~ 4450 1250
Text GLabel 4100 2600 0    60   Input ~ 0
SENSE
Wire Wire Line
	4100 2600 4300 2600
$Comp
L Conn_01x01 J2
U 1 1 5B0E638F
P 1100 1800
F 0 "J2" H 1100 1900 50  0000 C CNN
F 1 "Conn_01x01" H 1100 1700 50  0000 C CNN
F 2 "Connectors:1pin" H 1100 1800 50  0001 C CNN
F 3 "" H 1100 1800 50  0001 C CNN
	1    1100 1800
	-1   0    0    1   
$EndComp
Text GLabel 2100 1800 2    60   Output ~ 0
SENSE
$Comp
L R R1
U 1 1 5B0E662F
P 1550 1800
F 0 "R1" V 1630 1800 50  0000 C CNN
F 1 "R" V 1550 1800 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 1480 1800 50  0001 C CNN
F 3 "" H 1550 1800 50  0001 C CNN
	1    1550 1800
	0    1    1    0   
$EndComp
$Comp
L R R3
U 1 1 5B0E66F1
P 1900 1550
F 0 "R3" V 1980 1550 50  0000 C CNN
F 1 "R" V 1900 1550 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 1830 1550 50  0001 C CNN
F 3 "" H 1900 1550 50  0001 C CNN
	1    1900 1550
	-1   0    0    1   
$EndComp
$Comp
L VCC #PWR015
U 1 1 5B0E677B
P 5000 1500
F 0 "#PWR015" H 5000 1350 50  0001 C CNN
F 1 "VCC" H 5000 1650 50  0000 C CNN
F 2 "" H 5000 1500 50  0001 C CNN
F 3 "" H 5000 1500 50  0001 C CNN
	1    5000 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 1500 5000 1700
$Comp
L VCC #PWR016
U 1 1 5B0E687F
P 1900 1300
F 0 "#PWR016" H 1900 1150 50  0001 C CNN
F 1 "VCC" H 1900 1450 50  0000 C CNN
F 2 "" H 1900 1300 50  0001 C CNN
F 3 "" H 1900 1300 50  0001 C CNN
	1    1900 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 1800 1400 1800
Wire Wire Line
	1700 1800 2100 1800
Wire Wire Line
	1900 1700 1900 1900
Connection ~ 1900 1800
$Comp
L C C1
U 1 1 5B0E6AA1
P 1900 2050
F 0 "C1" H 1925 2150 50  0000 L CNN
F 1 "C" H 1925 1950 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 1938 1900 50  0001 C CNN
F 3 "" H 1900 2050 50  0001 C CNN
	1    1900 2050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR017
U 1 1 5B0E6B67
P 1900 2300
F 0 "#PWR017" H 1900 2050 50  0001 C CNN
F 1 "GND" H 1900 2150 50  0000 C CNN
F 2 "" H 1900 2300 50  0001 C CNN
F 3 "" H 1900 2300 50  0001 C CNN
	1    1900 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 2200 1900 2300
$Comp
L VCC #PWR018
U 1 1 5B0E765C
P 2750 4150
F 0 "#PWR018" H 2750 4000 50  0001 C CNN
F 1 "VCC" H 2750 4300 50  0000 C CNN
F 2 "" H 2750 4150 50  0001 C CNN
F 3 "" H 2750 4150 50  0001 C CNN
	1    2750 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 4200 2750 4150
Wire Wire Line
	7650 1300 7650 1250
Wire Wire Line
	1900 1400 1900 1300
$EndSCHEMATC