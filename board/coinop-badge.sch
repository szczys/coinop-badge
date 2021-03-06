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
LIBS:MF_Aesthetics
LIBS:MF_Connectors
LIBS:MF_Discrete_Semiconductor
LIBS:MF_Displays
LIBS:MF_Frequency_Control
LIBS:MF_IC_Analog
LIBS:MF_IC_Digital
LIBS:MF_IC_Power
LIBS:MF_LEDs
LIBS:MF_Passives
LIBS:MF_Sensors
LIBS:MF_Switches
LIBS:coinop-badge-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Coin-Op Badge"
Date ""
Rev "2.1"
Comp "Mike Szczys - Jumptuck.com"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ATMEGA48A-AU U1
U 1 1 5B2FEE56
P 3050 3350
F 0 "U1" H 2300 4600 50  0000 L BNN
F 1 "ATMEGA48A-AU" H 3450 1950 50  0000 L BNN
F 2 "Housings_QFP:TQFP-32_7x7mm_Pitch0.8mm" H 3050 3350 50  0001 C CIN
F 3 "" H 3050 3350 50  0001 C CNN
	1    3050 3350
	1    0    0    -1  
$EndComp
$Comp
L LED D1
U 1 1 5B2FEF10
P 7600 1550
F 0 "D1" H 7600 1650 50  0000 C CNN
F 1 "LED" H 7600 1450 50  0000 C CNN
F 2 "LEDs:LED_0603" H 7600 1550 50  0001 C CNN
F 3 "" H 7600 1550 50  0001 C CNN
	1    7600 1550
	-1   0    0    1   
$EndComp
$Comp
L LED D2
U 1 1 5B2FEFA3
P 7600 1850
F 0 "D2" H 7600 1950 50  0000 C CNN
F 1 "LED" H 7600 1750 50  0000 C CNN
F 2 "LEDs:LED_0603" H 7600 1850 50  0001 C CNN
F 3 "" H 7600 1850 50  0001 C CNN
	1    7600 1850
	-1   0    0    1   
$EndComp
$Comp
L LED D3
U 1 1 5B2FEFE0
P 7600 2150
F 0 "D3" H 7600 2250 50  0000 C CNN
F 1 "LED" H 7600 2050 50  0000 C CNN
F 2 "LEDs:LED_0603" H 7600 2150 50  0001 C CNN
F 3 "" H 7600 2150 50  0001 C CNN
	1    7600 2150
	-1   0    0    1   
$EndComp
$Comp
L LED D4
U 1 1 5B2FF018
P 7600 2600
F 0 "D4" H 7600 2700 50  0000 C CNN
F 1 "LED" H 7600 2500 50  0000 C CNN
F 2 "LEDs:LED_0603" H 7600 2600 50  0001 C CNN
F 3 "" H 7600 2600 50  0001 C CNN
	1    7600 2600
	-1   0    0    1   
$EndComp
$Comp
L LED D5
U 1 1 5B2FF068
P 7600 2900
F 0 "D5" H 7600 3000 50  0000 C CNN
F 1 "LED" H 7600 2800 50  0000 C CNN
F 2 "LEDs:LED_0603" H 7600 2900 50  0001 C CNN
F 3 "" H 7600 2900 50  0001 C CNN
	1    7600 2900
	-1   0    0    1   
$EndComp
$Comp
L LED D6
U 1 1 5B2FF092
P 7600 3200
F 0 "D6" H 7600 3300 50  0000 C CNN
F 1 "LED" H 7600 3100 50  0000 C CNN
F 2 "LEDs:LED_0603" H 7600 3200 50  0001 C CNN
F 3 "" H 7600 3200 50  0001 C CNN
	1    7600 3200
	-1   0    0    1   
$EndComp
$Comp
L LED D7
U 1 1 5B2FF0DF
P 7600 3650
F 0 "D7" H 7600 3750 50  0000 C CNN
F 1 "LED" H 7600 3550 50  0000 C CNN
F 2 "LEDs:LED_0603" H 7600 3650 50  0001 C CNN
F 3 "" H 7600 3650 50  0001 C CNN
	1    7600 3650
	-1   0    0    1   
$EndComp
$Comp
L LED D8
U 1 1 5B2FF179
P 7600 3950
F 0 "D8" H 7600 4050 50  0000 C CNN
F 1 "LED" H 7600 3850 50  0000 C CNN
F 2 "LEDs:LED_0603" H 7600 3950 50  0001 C CNN
F 3 "" H 7600 3950 50  0001 C CNN
	1    7600 3950
	-1   0    0    1   
$EndComp
$Comp
L LED D9
U 1 1 5B2FF1AC
P 7600 4250
F 0 "D9" H 7600 4350 50  0000 C CNN
F 1 "LED" H 7600 4150 50  0000 C CNN
F 2 "LEDs:LED_0603" H 7600 4250 50  0001 C CNN
F 3 "" H 7600 4250 50  0001 C CNN
	1    7600 4250
	-1   0    0    1   
$EndComp
$Comp
L LED D10
U 1 1 5B2FF1E2
P 7600 4700
F 0 "D10" H 7600 4800 50  0000 C CNN
F 1 "LED" H 7600 4600 50  0000 C CNN
F 2 "LEDs:LED_0603" H 7600 4700 50  0001 C CNN
F 3 "" H 7600 4700 50  0001 C CNN
	1    7600 4700
	-1   0    0    1   
$EndComp
$Comp
L LED D11
U 1 1 5B2FF230
P 7600 5000
F 0 "D11" H 7600 5100 50  0000 C CNN
F 1 "LED" H 7600 4900 50  0000 C CNN
F 2 "LEDs:LED_0603" H 7600 5000 50  0001 C CNN
F 3 "" H 7600 5000 50  0001 C CNN
	1    7600 5000
	-1   0    0    1   
$EndComp
$Comp
L LED D12
U 1 1 5B2FF26C
P 7600 5300
F 0 "D12" H 7600 5400 50  0000 C CNN
F 1 "LED" H 7600 5200 50  0000 C CNN
F 2 "LEDs:LED_0603" H 7600 5300 50  0001 C CNN
F 3 "" H 7600 5300 50  0001 C CNN
	1    7600 5300
	-1   0    0    1   
$EndComp
$Comp
L LED D21
U 1 1 5B2FF37B
P 10050 2000
F 0 "D21" H 10050 2100 50  0000 C CNN
F 1 "LED" H 10050 1900 50  0000 C CNN
F 2 "LEDs:LED_0603" H 10050 2000 50  0001 C CNN
F 3 "" H 10050 2000 50  0001 C CNN
	1    10050 2000
	0    -1   -1   0   
$EndComp
$Comp
L LED D22
U 1 1 5B2FF3F5
P 10500 2000
F 0 "D22" H 10500 2100 50  0000 C CNN
F 1 "LED" H 10500 1900 50  0000 C CNN
F 2 "LEDs:LED_0603" H 10500 2000 50  0001 C CNN
F 3 "" H 10500 2000 50  0001 C CNN
	1    10500 2000
	0    1    1    0   
$EndComp
$Comp
L LED D13
U 1 1 5B2FF43A
P 10450 3550
F 0 "D13" H 10450 3650 50  0000 C CNN
F 1 "LED" H 10450 3450 50  0000 C CNN
F 2 "LEDs:LED_0603" H 10450 3550 50  0001 C CNN
F 3 "" H 10450 3550 50  0001 C CNN
	1    10450 3550
	-1   0    0    1   
$EndComp
$Comp
L LED D14
U 1 1 5B2FF482
P 10450 3850
F 0 "D14" H 10450 3950 50  0000 C CNN
F 1 "LED" H 10450 3750 50  0000 C CNN
F 2 "LEDs:LED_0603" H 10450 3850 50  0001 C CNN
F 3 "" H 10450 3850 50  0001 C CNN
	1    10450 3850
	-1   0    0    1   
$EndComp
$Comp
L LED D15
U 1 1 5B2FF511
P 10450 4150
F 0 "D15" H 10450 4250 50  0000 C CNN
F 1 "LED" H 10450 4050 50  0000 C CNN
F 2 "LEDs:LED_0603" H 10450 4150 50  0001 C CNN
F 3 "" H 10450 4150 50  0001 C CNN
	1    10450 4150
	-1   0    0    1   
$EndComp
$Comp
L LED D16
U 1 1 5B2FF577
P 10450 4450
F 0 "D16" H 10450 4550 50  0000 C CNN
F 1 "LED" H 10450 4350 50  0000 C CNN
F 2 "LEDs:LED_0603" H 10450 4450 50  0001 C CNN
F 3 "" H 10450 4450 50  0001 C CNN
	1    10450 4450
	-1   0    0    1   
$EndComp
$Comp
L VCC #PWR01
U 1 1 5B30039F
P 1400 1950
F 0 "#PWR01" H 1400 1800 50  0001 C CNN
F 1 "VCC" H 1400 2100 50  0000 C CNN
F 2 "" H 1400 1950 50  0001 C CNN
F 3 "" H 1400 1950 50  0001 C CNN
	1    1400 1950
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR02
U 1 1 5B3004AD
P 2450 1200
F 0 "#PWR02" H 2450 1050 50  0001 C CNN
F 1 "VCC" H 2450 1350 50  0000 C CNN
F 2 "" H 2450 1200 50  0001 C CNN
F 3 "" H 2450 1200 50  0001 C CNN
	1    2450 1200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 5B30052E
P 2450 1700
F 0 "#PWR03" H 2450 1450 50  0001 C CNN
F 1 "GND" H 2450 1550 50  0000 C CNN
F 2 "" H 2450 1700 50  0001 C CNN
F 3 "" H 2450 1700 50  0001 C CNN
	1    2450 1700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR04
U 1 1 5B30057F
P 1400 3500
F 0 "#PWR04" H 1400 3250 50  0001 C CNN
F 1 "GND" H 1400 3350 50  0000 C CNN
F 2 "" H 1400 3500 50  0001 C CNN
F 3 "" H 1400 3500 50  0001 C CNN
	1    1400 3500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 5B3005F3
P 1400 4850
F 0 "#PWR05" H 1400 4600 50  0001 C CNN
F 1 "GND" H 1400 4700 50  0000 C CNN
F 2 "" H 1400 4850 50  0001 C CNN
F 3 "" H 1400 4850 50  0001 C CNN
	1    1400 4850
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 5B30099F
P 2450 1450
F 0 "C1" H 2475 1550 50  0000 L CNN
F 1 "0.1uF" H 2475 1350 50  0000 L CNN
F 2 "LEDs:LED_0603" H 2488 1300 50  0001 C CNN
F 3 "" H 2450 1450 50  0001 C CNN
	1    2450 1450
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 5B300BA0
P 1400 3150
F 0 "C3" H 1425 3250 50  0000 L CNN
F 1 "0.1uF" H 1425 3050 50  0000 L CNN
F 2 "LEDs:LED_0603" H 1438 3000 50  0001 C CNN
F 3 "" H 1400 3150 50  0001 C CNN
	1    1400 3150
	1    0    0    -1  
$EndComp
$Comp
L R_Pack08 RN1
U 1 1 5B30162C
P 6750 2450
F 0 "RN1" V 6250 2450 50  0000 C CNN
F 1 "39-Ohm_Pack08" V 7150 2450 50  0000 C CNN
F 2 "Szczys:EXB-2HV390JV" V 7225 2450 50  0001 C CNN
F 3 "" H 6750 2450 50  0001 C CNN
	1    6750 2450
	0    -1   1    0   
$EndComp
$Comp
L R_Pack08 RN2
U 1 1 5B3016E7
P 6800 4500
F 0 "RN2" V 6300 4500 50  0000 C CNN
F 1 "39-Ohm_Pack08" V 7200 4500 50  0000 C CNN
F 2 "Szczys:EXB-2HV390JV" V 7275 4500 50  0001 C CNN
F 3 "" H 6800 4500 50  0001 C CNN
	1    6800 4500
	0    1    1    0   
$EndComp
$Comp
L GND #PWR06
U 1 1 5B302698
P 10850 4650
F 0 "#PWR06" H 10850 4400 50  0001 C CNN
F 1 "GND" H 10850 4500 50  0000 C CNN
F 2 "" H 10850 4650 50  0001 C CNN
F 3 "" H 10850 4650 50  0001 C CNN
	1    10850 4650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR07
U 1 1 5B3028C5
P 8050 5650
F 0 "#PWR07" H 8050 5400 50  0001 C CNN
F 1 "GND" H 8050 5500 50  0000 C CNN
F 2 "" H 8050 5650 50  0001 C CNN
F 3 "" H 8050 5650 50  0001 C CNN
	1    8050 5650
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR08
U 1 1 5B3055EB
P 3300 1100
F 0 "#PWR08" H 3300 950 50  0001 C CNN
F 1 "VCC" H 3300 1250 50  0000 C CNN
F 2 "" H 3300 1100 50  0001 C CNN
F 3 "" H 3300 1100 50  0001 C CNN
	1    3300 1100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR09
U 1 1 5B305647
P 3300 1800
F 0 "#PWR09" H 3300 1550 50  0001 C CNN
F 1 "GND" H 3300 1650 50  0000 C CNN
F 2 "" H 3300 1800 50  0001 C CNN
F 3 "" H 3300 1800 50  0001 C CNN
	1    3300 1800
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR010
U 1 1 5B305BAA
P 2800 1200
F 0 "#PWR010" H 2800 1050 50  0001 C CNN
F 1 "VCC" H 2800 1350 50  0000 C CNN
F 2 "" H 2800 1200 50  0001 C CNN
F 3 "" H 2800 1200 50  0001 C CNN
	1    2800 1200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR011
U 1 1 5B305BB0
P 2800 1700
F 0 "#PWR011" H 2800 1450 50  0001 C CNN
F 1 "GND" H 2800 1550 50  0000 C CNN
F 2 "" H 2800 1700 50  0001 C CNN
F 3 "" H 2800 1700 50  0001 C CNN
	1    2800 1700
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 5B305BB6
P 2800 1450
F 0 "C2" H 2825 1550 50  0000 L CNN
F 1 "0.1uF" H 2825 1350 50  0000 L CNN
F 2 "LEDs:LED_0603" H 2838 1300 50  0001 C CNN
F 3 "" H 2800 1450 50  0001 C CNN
	1    2800 1450
	1    0    0    -1  
$EndComp
$Comp
L SW_Push SW2
U 1 1 5B306224
P 2650 5600
F 0 "SW2" H 2700 5700 50  0000 L CNN
F 1 "SW_Push" H 2650 5540 50  0000 C CNN
F 2 "Szczys:Switches-TACT-PTS645" H 2650 5800 50  0001 C CNN
F 3 "" H 2650 5800 50  0001 C CNN
	1    2650 5600
	0    1    1    0   
$EndComp
$Comp
L SW_Push SW1
U 1 1 5B3062CE
P 2150 5600
F 0 "SW1" H 2200 5700 50  0000 L CNN
F 1 "SW_Push" H 2150 5540 50  0000 C CNN
F 2 "Szczys:Switches-TACT-PTS645" H 2150 5800 50  0001 C CNN
F 3 "" H 2150 5800 50  0001 C CNN
	1    2150 5600
	0    1    1    0   
$EndComp
Text GLabel 6100 2050 0    60   Input ~ 0
PEW-1-1
Text GLabel 6100 2200 0    60   Input ~ 0
PEW-1-2
Text GLabel 6100 2350 0    60   Input ~ 0
PEW-1-3
Text GLabel 6100 2650 0    60   Input ~ 0
PEW-2-1
Text GLabel 6100 2800 0    60   Input ~ 0
PEW-2-2
Text GLabel 6100 2950 0    60   Input ~ 0
PEW-2-3
Text GLabel 6100 4700 0    60   Input ~ 0
PEW-4-1
Text GLabel 6100 4400 0    60   Input ~ 0
PEW-3-3
Text GLabel 6100 4250 0    60   Input ~ 0
PEW-3-2
Text GLabel 6100 4100 0    60   Input ~ 0
PEW-3-1
Text GLabel 6100 4850 0    60   Input ~ 0
PEW-4-2
Text GLabel 6100 5000 0    60   Input ~ 0
PEW-4-3
Text GLabel 9000 4150 0    60   Input ~ 0
SHIELD-3
Text GLabel 9000 4450 0    60   Input ~ 0
SHIELD-4
Text GLabel 9000 3850 0    60   Input ~ 0
SHIELD-2
Text GLabel 9000 3550 0    60   Input ~ 0
SHIELD-1
$Comp
L LED D17
U 1 1 5B309428
P 9100 1700
F 0 "D17" H 9100 1800 50  0000 C CNN
F 1 "LED" H 9100 1600 50  0000 C CNN
F 2 "LEDs:LED_0603" H 9100 1700 50  0001 C CNN
F 3 "" H 9100 1700 50  0001 C CNN
	1    9100 1700
	0    -1   -1   0   
$EndComp
$Comp
L LED D18
U 1 1 5B309579
P 9550 1700
F 0 "D18" H 9550 1800 50  0000 C CNN
F 1 "LED" H 9550 1600 50  0000 C CNN
F 2 "LEDs:LED_0603" H 9550 1700 50  0001 C CNN
F 3 "" H 9550 1700 50  0001 C CNN
	1    9550 1700
	0    1    1    0   
$EndComp
$Comp
L LED D19
U 1 1 5B3095F1
P 9100 2300
F 0 "D19" H 9100 2400 50  0000 C CNN
F 1 "LED" H 9100 2200 50  0000 C CNN
F 2 "LEDs:LED_0603" H 9100 2300 50  0001 C CNN
F 3 "" H 9100 2300 50  0001 C CNN
	1    9100 2300
	0    -1   -1   0   
$EndComp
$Comp
L LED D20
U 1 1 5B30966A
P 9550 2300
F 0 "D20" H 9550 2400 50  0000 C CNN
F 1 "LED" H 9550 2200 50  0000 C CNN
F 2 "LEDs:LED_0603" H 9550 2300 50  0001 C CNN
F 3 "" H 9550 2300 50  0001 C CNN
	1    9550 2300
	0    1    1    0   
$EndComp
$Comp
L AVR-ISP-6 CON1
U 1 1 5B309A5E
P 5200 5900
F 0 "CON1" H 5095 6140 50  0000 C CNN
F 1 "AVR-ISP-6" H 4935 5670 50  0000 L BNN
F 2 "Szczys:SIL_1x06_SMD_Pin_Header" V 4680 5940 50  0001 C CNN
F 3 "" H 5175 5900 50  0001 C CNN
	1    5200 5900
	1    0    0    -1  
$EndComp
Text GLabel 8750 1550 0    60   Input ~ 0
ENG-1
Text GLabel 8750 2000 0    60   Input ~ 0
ENG-2
Text GLabel 8750 2450 0    60   Input ~ 0
ENG-3
Text GLabel 6100 1500 0    60   Input ~ 0
CHARLIE-1
Text GLabel 6100 1650 0    60   Input ~ 0
CHARLIE-2
Text GLabel 7050 1300 1    60   Input ~ 0
ENG-1
Text GLabel 7200 1300 1    60   Input ~ 0
ENG-2
Text GLabel 6850 3700 1    60   Input ~ 0
ENG-3
Text GLabel 1900 5900 0    60   Input ~ 0
SW-POWER
Text GLabel 2900 5900 2    60   Input ~ 0
SW-SELECT
Text GLabel 6100 3550 0    60   Input ~ 0
CHARLIE-3
Text GLabel 4200 2250 2    60   Input ~ 0
PEW-1-1
Text GLabel 4750 2350 2    60   Input ~ 0
PEW-1-2
Text GLabel 4200 2450 2    60   Input ~ 0
PEW-1-3
Text GLabel 4750 2550 2    60   Input ~ 0
PEW-2-1
Text GLabel 4200 2650 2    60   Input ~ 0
PEW-2-2
Text GLabel 4900 2750 2    60   Input ~ 0
PEW-2-3
Text GLabel 4750 2950 2    60   Input ~ 0
PEW-3-1
Text GLabel 4200 2850 2    60   Input ~ 0
PEW-3-2
Text GLabel 4750 3600 2    60   Input ~ 0
PEW-3-3
Text GLabel 4200 3500 2    60   Input ~ 0
PEW-4-1
Text GLabel 4750 3400 2    60   Input ~ 0
PEW-4-2
Text GLabel 4200 3300 2    60   Input ~ 0
PEW-4-3
Text GLabel 4150 3100 2    60   Input ~ 0
SW-POWER
Text GLabel 4750 3200 2    60   Input ~ 0
SW-SELECT
Text GLabel 4750 4150 2    60   Input ~ 0
CHARLIE-1
Text GLabel 4150 4250 2    60   Input ~ 0
CHARLIE-2
Text GLabel 4750 3950 2    60   Input ~ 0
CHARLIE-3
Text GLabel 4150 4050 2    60   Input ~ 0
SHIELD-1
Text GLabel 4750 4350 2    60   Input ~ 0
SHIELD-2
Text GLabel 4200 4450 2    60   Input ~ 0
SHIELD-3
Text GLabel 4750 4550 2    60   Input ~ 0
SHIELD-4
Text GLabel 4650 5800 0    60   Input ~ 0
MISO
Text GLabel 5700 5900 2    60   Input ~ 0
MOSI
Text GLabel 4650 6000 0    60   Input ~ 0
RST
Text GLabel 4300 5900 0    60   Input ~ 0
SCK
$Comp
L GND #PWR012
U 1 1 5B32336C
P 5700 6150
F 0 "#PWR012" H 5700 5900 50  0001 C CNN
F 1 "GND" H 5700 6000 50  0000 C CNN
F 2 "" H 5700 6150 50  0001 C CNN
F 3 "" H 5700 6150 50  0001 C CNN
	1    5700 6150
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR013
U 1 1 5B323533
P 5700 5600
F 0 "#PWR013" H 5700 5450 50  0001 C CNN
F 1 "VCC" H 5700 5750 50  0000 C CNN
F 2 "" H 5700 5600 50  0001 C CNN
F 3 "" H 5700 5600 50  0001 C CNN
	1    5700 5600
	1    0    0    -1  
$EndComp
Text GLabel 4100 1900 1    60   Input ~ 0
MISO
Text GLabel 4400 1900 1    60   Input ~ 0
SCK
Text GLabel 4250 1900 1    60   Input ~ 0
MOSI
Text GLabel 4150 3700 2    60   Input ~ 0
RST
NoConn ~ 2150 3600
NoConn ~ 2150 3700
$Comp
L R R1
U 1 1 5B3269D5
P 2150 6150
F 0 "R1" V 2230 6150 50  0000 C CNN
F 1 "4k7" V 2150 6150 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 2080 6150 50  0001 C CNN
F 3 "" H 2150 6150 50  0001 C CNN
	1    2150 6150
	-1   0    0    1   
$EndComp
$Comp
L R R2
U 1 1 5B326E5D
P 2650 6150
F 0 "R2" V 2730 6150 50  0000 C CNN
F 1 "4k7" V 2650 6150 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 2580 6150 50  0001 C CNN
F 3 "" H 2650 6150 50  0001 C CNN
	1    2650 6150
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR014
U 1 1 5B326EE1
P 2400 6500
F 0 "#PWR014" H 2400 6250 50  0001 C CNN
F 1 "GND" H 2400 6350 50  0000 C CNN
F 2 "" H 2400 6500 50  0001 C CNN
F 3 "" H 2400 6500 50  0001 C CNN
	1    2400 6500
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR015
U 1 1 5B32819C
P 2400 5200
F 0 "#PWR015" H 2400 5050 50  0001 C CNN
F 1 "VCC" H 2400 5350 50  0000 C CNN
F 2 "" H 2400 5200 50  0001 C CNN
F 3 "" H 2400 5200 50  0001 C CNN
	1    2400 5200
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x02 J1
U 1 1 5B329C89
P 1900 1250
F 0 "J1" H 1900 1350 50  0000 C CNN
F 1 "Conn_01x02" H 1900 1050 50  0000 C CNN
F 2 "Szczys:SIL_1x02_SMD_Pin_Header" H 1900 1250 50  0001 C CNN
F 3 "" H 1900 1250 50  0001 C CNN
	1    1900 1250
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR016
U 1 1 5B329D27
P 1550 1100
F 0 "#PWR016" H 1550 950 50  0001 C CNN
F 1 "VCC" H 1550 1250 50  0000 C CNN
F 2 "" H 1550 1100 50  0001 C CNN
F 3 "" H 1550 1100 50  0001 C CNN
	1    1550 1100
	1    0    0    -1  
$EndComp
Text Notes 900  1500 0    60   ~ 0
Cut-trace\nfor optional\npower jumper
$Comp
L BATTERY_BAT-HLD-001 BATT1
U 1 1 5B3313CB
P 3300 1450
F 0 "BATT1" H 3060 1570 45  0000 L BNN
F 1 "BATTERY_BAT-HLD-001" V 3500 1030 45  0000 L BNN
F 2 "MF_Connectors:MF_Connectors-BAT-HLD-001" V 3540 1038 20  0001 L BNN
F 3 "" H 3300 1450 60  0000 C CNN
	1    3300 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 3300 4200 3300
Wire Wire Line
	4750 3400 4050 3400
Wire Wire Line
	4050 3500 4200 3500
Wire Wire Line
	4750 3600 4050 3600
Wire Wire Line
	4050 3200 4750 3200
Wire Wire Line
	4050 3100 4150 3100
Wire Wire Line
	3300 1650 3300 1800
Wire Wire Line
	3300 1100 3300 1250
Wire Wire Line
	1400 1950 1400 2550
Wire Wire Line
	1400 2850 2150 2850
Wire Wire Line
	1400 3000 1400 2850
Wire Wire Line
	1400 3300 1400 3500
Connection ~ 1550 1250
Wire Wire Line
	1700 1250 1550 1250
Wire Wire Line
	1550 1350 1700 1350
Wire Wire Line
	1550 1100 1550 1350
Connection ~ 2400 5300
Wire Wire Line
	2400 5200 2400 5300
Wire Wire Line
	2150 5300 2150 5400
Wire Wire Line
	2150 5300 2650 5300
Wire Wire Line
	2650 5300 2650 5400
Connection ~ 2400 6400
Wire Wire Line
	2400 6400 2400 6500
Wire Wire Line
	2150 6400 2150 6300
Wire Wire Line
	2150 6400 2650 6400
Wire Wire Line
	2650 6400 2650 6300
Connection ~ 2650 5900
Wire Wire Line
	2900 5900 2650 5900
Wire Wire Line
	2650 5800 2650 6000
Connection ~ 2150 5900
Wire Wire Line
	2150 5800 2150 6000
Wire Wire Line
	1900 5900 2150 5900
Connection ~ 4750 2750
Wire Wire Line
	4400 2050 4400 1900
Wire Wire Line
	5300 2050 4400 2050
Wire Wire Line
	5300 2650 5300 2050
Wire Wire Line
	4750 2650 5300 2650
Wire Wire Line
	4750 2750 4750 2650
Connection ~ 4150 2550
Wire Wire Line
	4250 2050 4250 1900
Wire Wire Line
	4150 2050 4250 2050
Wire Wire Line
	4150 2550 4150 2050
Wire Wire Line
	4050 3700 4150 3700
Connection ~ 4100 2650
Wire Wire Line
	4100 2650 4100 1900
Wire Wire Line
	5700 6000 5700 6150
Wire Wire Line
	5300 6000 5700 6000
Wire Wire Line
	5700 5900 5300 5900
Wire Wire Line
	5700 5800 5700 5600
Wire Wire Line
	5300 5800 5700 5800
Wire Wire Line
	4650 6000 5050 6000
Wire Wire Line
	4300 5900 5050 5900
Wire Wire Line
	4650 5800 5050 5800
Wire Wire Line
	4750 2350 4050 2350
Wire Wire Line
	4050 2550 4750 2550
Wire Wire Line
	4050 2250 4200 2250
Wire Wire Line
	4050 2450 4200 2450
Wire Wire Line
	4050 2650 4200 2650
Wire Wire Line
	4050 2750 4900 2750
Wire Wire Line
	4750 4550 4050 4550
Wire Wire Line
	4050 4450 4200 4450
Wire Wire Line
	4750 4350 4050 4350
Wire Wire Line
	6850 3900 6850 3700
Wire Wire Line
	7000 3900 6850 3900
Wire Wire Line
	7000 4100 7000 3900
Wire Wire Line
	6550 4100 6600 4100
Wire Wire Line
	6550 3550 6550 4100
Wire Wire Line
	6100 3550 6550 3550
Connection ~ 10850 4450
Wire Wire Line
	10850 4450 10600 4450
Connection ~ 10850 4150
Wire Wire Line
	10850 4150 10600 4150
Connection ~ 10850 3850
Wire Wire Line
	10600 3850 10850 3850
Wire Wire Line
	10850 3550 10850 4650
Wire Wire Line
	10600 3550 10850 3550
Wire Wire Line
	7250 4250 7450 4250
Wire Wire Line
	7250 4500 7250 4250
Wire Wire Line
	7000 4500 7250 4500
Wire Wire Line
	7200 4400 7000 4400
Wire Wire Line
	7200 3950 7200 4400
Wire Wire Line
	7450 3950 7200 3950
Wire Wire Line
	7150 4300 7000 4300
Wire Wire Line
	7150 3650 7150 4300
Wire Wire Line
	7450 3650 7150 3650
Wire Wire Line
	7250 2150 7450 2150
Wire Wire Line
	7250 2450 7250 2150
Wire Wire Line
	6950 2450 7250 2450
Wire Wire Line
	7200 1850 7450 1850
Wire Wire Line
	7200 2350 7200 1850
Wire Wire Line
	6950 2350 7200 2350
Wire Wire Line
	7150 1550 7450 1550
Wire Wire Line
	7150 2250 7150 1550
Wire Wire Line
	6950 2250 7150 2250
Wire Wire Line
	7200 1500 7200 1300
Wire Wire Line
	7100 1500 7200 1500
Wire Wire Line
	7100 2150 7100 1500
Wire Wire Line
	6950 2150 7100 2150
Wire Wire Line
	7050 2050 7050 1300
Wire Wire Line
	6950 2050 7050 2050
Wire Wire Line
	6500 2050 6550 2050
Wire Wire Line
	6500 1500 6500 2050
Wire Wire Line
	6100 1500 6500 1500
Wire Wire Line
	6450 1650 6100 1650
Wire Wire Line
	6450 2150 6450 1650
Wire Wire Line
	6550 2150 6450 2150
Connection ~ 9100 2450
Connection ~ 9100 1550
Connection ~ 10050 2450
Wire Wire Line
	10050 2450 10050 2150
Connection ~ 9550 2000
Connection ~ 9100 2000
Wire Wire Line
	8750 2000 9550 2000
Wire Wire Line
	9550 1850 9550 2150
Wire Wire Line
	9100 1850 9100 2150
Connection ~ 9550 2450
Wire Wire Line
	8750 2450 10500 2450
Wire Wire Line
	10500 2450 10500 2150
Connection ~ 10050 1550
Wire Wire Line
	10050 1550 10050 1850
Connection ~ 9550 1550
Wire Wire Line
	10500 1550 10500 1850
Wire Wire Line
	8750 1550 10500 1550
Wire Wire Line
	6400 2250 6550 2250
Wire Wire Line
	6400 2050 6400 2250
Wire Wire Line
	6100 2050 6400 2050
Wire Wire Line
	6300 2350 6550 2350
Wire Wire Line
	6300 2200 6300 2350
Wire Wire Line
	6100 2200 6300 2200
Wire Wire Line
	6200 2450 6550 2450
Wire Wire Line
	6200 2350 6200 2450
Wire Wire Line
	6100 2350 6200 2350
Wire Wire Line
	6400 2750 6550 2750
Wire Wire Line
	6400 2950 6400 2750
Wire Wire Line
	6100 2950 6400 2950
Wire Wire Line
	6300 2650 6550 2650
Wire Wire Line
	6300 2800 6300 2650
Wire Wire Line
	6100 2800 6300 2800
Wire Wire Line
	6200 2550 6550 2550
Wire Wire Line
	6200 2650 6200 2550
Wire Wire Line
	6100 2650 6200 2650
Wire Wire Line
	6400 4300 6600 4300
Wire Wire Line
	6400 4100 6400 4300
Wire Wire Line
	6100 4100 6400 4100
Wire Wire Line
	6300 4400 6600 4400
Wire Wire Line
	6300 4250 6300 4400
Wire Wire Line
	6100 4250 6300 4250
Wire Wire Line
	6200 4500 6600 4500
Wire Wire Line
	6200 4400 6200 4500
Wire Wire Line
	6100 4400 6200 4400
Wire Wire Line
	6400 4800 6600 4800
Wire Wire Line
	6400 5000 6400 4800
Wire Wire Line
	6100 5000 6400 5000
Wire Wire Line
	6300 4700 6600 4700
Wire Wire Line
	6300 4850 6300 4700
Wire Wire Line
	6100 4850 6300 4850
Wire Wire Line
	6200 4600 6600 4600
Wire Wire Line
	6200 4700 6200 4600
Wire Wire Line
	6100 4700 6200 4700
Connection ~ 1400 2550
Connection ~ 1400 2350
Wire Wire Line
	1400 2550 2150 2550
Wire Wire Line
	2800 1600 2800 1700
Wire Wire Line
	2800 1300 2800 1200
Connection ~ 8050 5300
Wire Wire Line
	7750 5300 8050 5300
Connection ~ 8050 5000
Wire Wire Line
	7750 5000 8050 5000
Connection ~ 8050 4700
Wire Wire Line
	7750 4700 8050 4700
Connection ~ 8050 4250
Wire Wire Line
	7750 4250 8050 4250
Connection ~ 8050 3950
Wire Wire Line
	7750 3950 8050 3950
Connection ~ 8050 3650
Wire Wire Line
	7750 3650 8050 3650
Connection ~ 8050 3200
Wire Wire Line
	7750 3200 8050 3200
Connection ~ 8050 2900
Wire Wire Line
	7750 2900 8050 2900
Connection ~ 8050 2600
Wire Wire Line
	7750 2600 8050 2600
Connection ~ 8050 2150
Wire Wire Line
	7750 2150 8050 2150
Connection ~ 8050 1850
Wire Wire Line
	7750 1850 8050 1850
Wire Wire Line
	8050 1550 7750 1550
Wire Wire Line
	8050 1550 8050 5650
Wire Wire Line
	7100 5300 7450 5300
Wire Wire Line
	7100 4800 7100 5300
Wire Wire Line
	7000 4800 7100 4800
Wire Wire Line
	7150 5000 7450 5000
Wire Wire Line
	7150 4700 7150 5000
Wire Wire Line
	7000 4700 7150 4700
Wire Wire Line
	7200 4700 7450 4700
Wire Wire Line
	7200 4600 7200 4700
Wire Wire Line
	7000 4600 7200 4600
Wire Wire Line
	7050 2750 6950 2750
Wire Wire Line
	7050 3200 7050 2750
Wire Wire Line
	7450 3200 7050 3200
Wire Wire Line
	7100 2650 6950 2650
Wire Wire Line
	7100 2900 7100 2650
Wire Wire Line
	7450 2900 7100 2900
Wire Wire Line
	7150 2550 6950 2550
Wire Wire Line
	7150 2600 7150 2550
Wire Wire Line
	7450 2600 7150 2600
Wire Wire Line
	2450 1600 2450 1700
Wire Wire Line
	2450 1300 2450 1200
Connection ~ 1400 2250
Wire Wire Line
	1400 2350 2150 2350
Wire Wire Line
	2150 2250 1400 2250
Connection ~ 1400 4550
Wire Wire Line
	1400 4550 2150 4550
Connection ~ 1400 4450
Wire Wire Line
	2150 4450 1400 4450
Wire Wire Line
	1400 4350 1400 4850
Wire Wire Line
	2150 4350 1400 4350
$Comp
L GND #PWR017
U 1 1 5B32B651
P 4450 1300
F 0 "#PWR017" H 4450 1050 50  0001 C CNN
F 1 "GND" H 4450 1150 50  0000 C CNN
F 2 "" H 4450 1300 50  0001 C CNN
F 3 "" H 4450 1300 50  0001 C CNN
	1    4450 1300
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG018
U 1 1 5B32B352
P 4450 1100
F 0 "#FLG018" H 4450 1175 50  0001 C CNN
F 1 "PWR_FLAG" H 4450 1250 50  0000 C CNN
F 2 "" H 4450 1100 50  0001 C CNN
F 3 "" H 4450 1100 50  0001 C CNN
	1    4450 1100
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG019
U 1 1 5B32AAE3
P 4000 1100
F 0 "#FLG019" H 4000 1175 50  0001 C CNN
F 1 "PWR_FLAG" H 4000 1250 50  0000 C CNN
F 2 "" H 4000 1100 50  0001 C CNN
F 3 "" H 4000 1100 50  0001 C CNN
	1    4000 1100
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR020
U 1 1 5B32B3CC
P 4000 1300
F 0 "#PWR020" H 4000 1150 50  0001 C CNN
F 1 "VCC" H 4000 1450 50  0000 C CNN
F 2 "" H 4000 1300 50  0001 C CNN
F 3 "" H 4000 1300 50  0001 C CNN
	1    4000 1300
	-1   0    0    1   
$EndComp
Wire Wire Line
	4450 1100 4450 1300
Wire Wire Line
	4000 1100 4000 1300
Wire Wire Line
	4050 4250 4150 4250
Wire Wire Line
	4050 3850 4150 3850
Wire Wire Line
	4750 4150 4050 4150
Wire Wire Line
	4750 3950 4050 3950
Wire Wire Line
	4050 4050 4150 4050
Wire Wire Line
	4750 2950 4050 2950
Wire Wire Line
	4050 2850 4200 2850
$Comp
L R_Pack04 RN3
U 1 1 5B35462A
P 9700 4050
F 0 "RN3" V 9400 4050 50  0000 C CNN
F 1 "15-Ohm_Pack04" V 9900 4050 50  0000 C CNN
F 2 "Szczys:EXB-38V150JV" V 9975 4050 50  0001 C CNN
F 3 "" H 9700 4050 50  0001 C CNN
	1    9700 4050
	0    -1   1    0   
$EndComp
Wire Wire Line
	9000 3850 9200 3850
Wire Wire Line
	9200 3850 9200 3950
Wire Wire Line
	9200 3950 9500 3950
Wire Wire Line
	9000 4150 9200 4150
Wire Wire Line
	9200 4150 9200 4050
Wire Wire Line
	9200 4050 9500 4050
Wire Wire Line
	9000 4450 9300 4450
Wire Wire Line
	9300 4450 9300 4150
Wire Wire Line
	9300 4150 9500 4150
Wire Wire Line
	9000 3550 9300 3550
Wire Wire Line
	9300 3550 9300 3850
Wire Wire Line
	9300 3850 9500 3850
Wire Wire Line
	9900 3950 10100 3950
Wire Wire Line
	10100 3950 10100 3850
Wire Wire Line
	10100 3850 10300 3850
Wire Wire Line
	9900 4050 10100 4050
Wire Wire Line
	10100 4050 10100 4150
Wire Wire Line
	10100 4150 10300 4150
Wire Wire Line
	10300 3550 10000 3550
Wire Wire Line
	10000 3550 10000 3850
Wire Wire Line
	10000 3850 9900 3850
Wire Wire Line
	10300 4450 10000 4450
Wire Wire Line
	10000 4450 10000 4150
Wire Wire Line
	10000 4150 9900 4150
NoConn ~ 4150 3850
NoConn ~ 6600 4200
NoConn ~ 7000 4200
$Comp
L VCC #PWR021
U 1 1 5B484985
P 4400 6800
F 0 "#PWR021" H 4400 6650 50  0001 C CNN
F 1 "VCC" H 4400 6950 50  0000 C CNN
F 2 "" H 4400 6800 50  0001 C CNN
F 3 "" H 4400 6800 50  0001 C CNN
	1    4400 6800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR022
U 1 1 5B4849FF
P 4400 7150
F 0 "#PWR022" H 4400 6900 50  0001 C CNN
F 1 "GND" H 4400 7000 50  0000 C CNN
F 2 "" H 4400 7150 50  0001 C CNN
F 3 "" H 4400 7150 50  0001 C CNN
	1    4400 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 6900 4400 6900
Wire Wire Line
	4400 6900 4400 6800
Wire Wire Line
	4950 7000 4400 7000
Wire Wire Line
	4400 7000 4400 7150
NoConn ~ 5450 6900
NoConn ~ 5450 7000
$Comp
L Conn_02x02_Odd_Even J2
U 1 1 5B486069
P 5150 6900
F 0 "J2" H 5200 7000 50  0000 C CNN
F 1 "Conn_02x02_Odd_Even" H 5200 6700 50  0000 C CNN
F 2 "Szczys:SAO_Pin_Header_Straight_2x02_Pitch2.54mm_SMD" H 5150 6900 50  0001 C CNN
F 3 "" H 5150 6900 50  0001 C CNN
	1    5150 6900
	1    0    0    -1  
$EndComp
$EndSCHEMATC
