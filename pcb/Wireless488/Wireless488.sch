EESchema Schematic File Version 4
LIBS:Wireless488-cache
EELAYER 26 0
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
L MCU_Module:Arduino_Nano_v3.x A1
U 1 1 5E3C853D
P 5850 3350
F 0 "A1" H 5850 4800 50  0000 C CNN
F 1 "Arduino_Nano_v3.x" H 5900 4900 50  0000 C CNN
F 2 "Module:Arduino_Nano" H 6000 2400 50  0001 L CNN
F 3 "http://www.mouser.com/pdfdocs/Gravitech_Arduino_Nano3_0.pdf" H 5850 2350 50  0001 C CNN
	1    5850 3350
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x12_Top_Bottom J1
U 1 1 5E3CB727
P 8650 2800
F 0 "J1" H 8700 3517 50  0000 C CNN
F 1 "Centronics 24 Pin" H 8700 3426 50  0000 C CNN
F 2 "ESP-GPIB:SCSI_Plug_24_RA" H 8650 2800 50  0001 C CNN
F 3 "~" H 8650 2800 50  0001 C CNN
	1    8650 2800
	1    0    0    -1  
$EndComp
Text Label 7950 2300 0    50   ~ 0
DIO1
Text Label 7950 2400 0    50   ~ 0
DIO2
Text Label 7950 2500 0    50   ~ 0
DIO3
Text Label 7950 2600 0    50   ~ 0
DIO4
Text Label 9100 2300 0    50   ~ 0
DIO5
Text Label 9100 2400 0    50   ~ 0
DIO6
Text Label 9100 2500 0    50   ~ 0
DIO7
Text Label 9100 2600 0    50   ~ 0
DIO8
Text Label 7950 2700 0    50   ~ 0
EOI
Text Label 7950 2800 0    50   ~ 0
DAV
Text Label 7950 2900 0    50   ~ 0
NRFD
Text Label 7950 3000 0    50   ~ 0
NDAC
Text Label 7950 3100 0    50   ~ 0
IFC
Text Label 7950 3200 0    50   ~ 0
SRQ
Text Label 7950 3300 0    50   ~ 0
ATN
Text Label 7950 3400 0    50   ~ 0
ChassisGND
Text Label 9100 2700 0    50   ~ 0
REN
Wire Wire Line
	8950 2800 9100 2800
Wire Wire Line
	9100 2800 9100 2900
Wire Wire Line
	9100 3400 8950 3400
Wire Wire Line
	8950 3300 9100 3300
Connection ~ 9100 3300
Wire Wire Line
	9100 3300 9100 3400
Wire Wire Line
	8950 3200 9100 3200
Connection ~ 9100 3200
Wire Wire Line
	9100 3200 9100 3300
Wire Wire Line
	8950 3100 9100 3100
Connection ~ 9100 3100
Wire Wire Line
	9100 3100 9100 3200
Wire Wire Line
	8950 3000 9100 3000
Connection ~ 9100 3000
Wire Wire Line
	9100 3000 9100 3100
Wire Wire Line
	8950 2900 9100 2900
Connection ~ 9100 2900
Wire Wire Line
	9100 2900 9100 3000
Wire Wire Line
	9100 2700 8950 2700
Wire Wire Line
	9100 2600 8950 2600
Wire Wire Line
	8950 2500 9100 2500
Wire Wire Line
	8950 2400 9100 2400
Wire Wire Line
	8950 2300 9100 2300
Wire Wire Line
	7950 2300 8450 2300
Wire Wire Line
	8450 2400 7950 2400
Wire Wire Line
	7950 2500 8450 2500
Wire Wire Line
	8450 2600 7950 2600
Wire Wire Line
	7950 2700 8450 2700
Wire Wire Line
	8450 2800 7950 2800
Wire Wire Line
	7950 2900 8450 2900
Wire Wire Line
	8450 3000 7950 3000
Wire Wire Line
	7950 3100 8450 3100
Wire Wire Line
	8450 3200 7950 3200
Wire Wire Line
	7950 3300 8450 3300
Wire Wire Line
	8450 3400 7950 3400
Text Notes 7900 5500 0    50   ~ 0
Pin # 	Signal names 	Signal Description 	Pin # 	Signal names 	Signal Description\n1 	DIO1 	Data Input/Output Bit 1 	13 	DIO5 	Data Input/Output Bit 5\n2 	DIO2 	Data Input/Output Bit 2 	14 	DIO6 	Data Input/Output Bit 6\n3 	DIO3 	Data Input/Output Bit 3 	15 	DIO7 	Data Input/Output Bit 7\n4 	DIO4 	Data Input/Output Bit 4 	16 	DIO8 	Data Input/Output Bit 8\n5 	EIO 	End-Or-Identify 	17 	REN 	Remote Enable\n6 	DAV 	Data Valid 	18 	Shield 	Ground (DAV)\n7 	NRFD 	Not Ready For Data 	19 	Shield 	Ground (NRFD)\n8 	NDAC 	Not Data Accepted 	20 	Shield 	Ground (NDAC)\n9 	IFC 	Interface Clear 	21 	Shield 	Ground (IFC)\n10 	SRQ 	Service Request 	22 	Shield 	Ground (SRQ)\n11 	ATN 	Attention 	23 	Shield 	Ground (ATN)\n12 	Shield 	Chassis Ground 	24 	Single GND 	Single Ground\n\nhttp://www.interfacebus.com/Design_Connector_GPIB.html\n\n\n12 goes to cable shield, might be NC
Text Notes 4850 6600 0    50   ~ 0
From AR488 Project\n\n#define DIO1  A0  /* GPIB 1  : PORTC bit 0 */\n#define DIO2  A1  /* GPIB 2  : PORTC bit 1 */\n#define DIO3  A2  /* GPIB 3  : PORTC bit 2 */\n#define DIO4  A3  /* GPIB 4  : PORTC bit 3 */\n#define DIO5  A4  /* GPIB 13 : PORTC bit 4 */\n#define DIO6  A5  /* GPIB 14 : PORTC bit 5 */\n#define DIO7   4  /* GPIB 15 : PORTD bit 4 */\n#define DIO8   5  /* GPIB 16 : PORTD bit 5 */\n\n#define IFC    8  /* GPIB 9  : PORTB bit 0 */\n#define NDAC   9  /* GPIB 8  : PORTB bit 1 */\n#define NRFD  10  /* GPIB 7  : PORTB bit 2 */\n#define DAV   11  /* GPIB 6  : PORTB bit 3 */\n#define EOI   12  /* GPIB 5  : PORTB bit 4 */\n\n#define SRQ    2  /* GPIB 10 : PORTD bit 2 */\n#define REN    3  /* GPIB 17 : PORTD bit 3 */\n#define ATN    7  /* GPIB 11 : PORTD bit 7 */
Text Label 6500 3350 0    50   ~ 0
DIO1
Text Label 6500 3450 0    50   ~ 0
DIO2
Text Label 6500 3550 0    50   ~ 0
DIO3
Text Label 6500 3650 0    50   ~ 0
DIO4
Text Label 6500 3750 0    50   ~ 0
DIO5
Text Label 6500 3850 0    50   ~ 0
DIO6
Text Label 5050 3150 0    50   ~ 0
DIO7
Wire Wire Line
	5050 3150 5350 3150
Text Label 5050 3250 0    50   ~ 0
DIO8
Wire Wire Line
	5050 3250 5350 3250
Text Label 5050 3550 0    50   ~ 0
IFC
Wire Wire Line
	5050 3550 5350 3550
Text Label 5050 3650 0    50   ~ 0
NDAC
Wire Wire Line
	5050 3650 5350 3650
Text Label 5050 3750 0    50   ~ 0
NRFD
Wire Wire Line
	5050 3750 5350 3750
Text Label 5050 3850 0    50   ~ 0
DAV
Wire Wire Line
	5050 3850 5350 3850
Text Label 5050 3950 0    50   ~ 0
EOI
Wire Wire Line
	5050 3950 5350 3950
Text Label 5050 2950 0    50   ~ 0
SRQ
Wire Wire Line
	5050 2950 5350 2950
Text Label 5050 3050 0    50   ~ 0
REN
Wire Wire Line
	5050 3050 5350 3050
Wire Wire Line
	5050 3450 5350 3450
Text Label 5050 3450 0    50   ~ 0
ATN
Wire Wire Line
	6500 3350 6350 3350
Wire Wire Line
	6500 3850 6350 3850
Wire Wire Line
	6350 3750 6500 3750
Wire Wire Line
	6500 3650 6350 3650
Wire Wire Line
	6350 3550 6500 3550
Wire Wire Line
	6500 3450 6350 3450
$Comp
L power:GND #PWR0102
U 1 1 5E3E72AD
P 5900 4500
F 0 "#PWR0102" H 5900 4250 50  0001 C CNN
F 1 "GND" H 5905 4327 50  0000 C CNN
F 2 "" H 5900 4500 50  0001 C CNN
F 3 "" H 5900 4500 50  0001 C CNN
	1    5900 4500
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0103
U 1 1 5E3E962D
P 6050 2250
F 0 "#PWR0103" H 6050 2100 50  0001 C CNN
F 1 "+5V" H 6065 2423 50  0000 C CNN
F 2 "" H 6050 2250 50  0001 C CNN
F 3 "" H 6050 2250 50  0001 C CNN
	1    6050 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 2250 6050 2350
$Comp
L power:+5V #PWR04
U 1 1 5E3F86EA
P 2650 1900
F 0 "#PWR04" H 2650 1750 50  0001 C CNN
F 1 "+5V" H 2665 2073 50  0000 C CNN
F 2 "" H 2650 1900 50  0001 C CNN
F 3 "" H 2650 1900 50  0001 C CNN
	1    2650 1900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 5E3F871C
P 2600 4700
F 0 "#PWR03" H 2600 4450 50  0001 C CNN
F 1 "GND" H 2605 4527 50  0000 C CNN
F 2 "" H 2600 4700 50  0001 C CNN
F 3 "" H 2600 4700 50  0001 C CNN
	1    2600 4700
	1    0    0    -1  
$EndComp
$Comp
L MCU_ESPModules:ESP32_DevKitC U1
U 1 1 5E40ABEA
P 2650 3200
F 0 "U1" H 1750 4600 50  0000 C CNN
F 1 "ESP32_DevKitC" H 2000 4450 50  0000 C CNN
F 2 "ESP-GPIB:ESP32_DevKit_C" H 2650 3200 50  0001 C CNN
F 3 "" H 2650 3200 50  0001 C CNN
	1    2650 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 1900 2650 2100
$Comp
L Connector:Conn_01x02_Female J2
U 1 1 5E5FCBAC
P 1000 1050
F 0 "J2" H 894 1235 50  0000 C CNN
F 1 "Power" H 894 1144 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 1000 1050 50  0001 C CNN
F 3 "~" H 1000 1050 50  0001 C CNN
	1    1000 1050
	-1   0    0    -1  
$EndComp
$Comp
L power:+5V #PWR01
U 1 1 5E5FCC48
P 1400 900
F 0 "#PWR01" H 1400 750 50  0001 C CNN
F 1 "+5V" H 1415 1073 50  0000 C CNN
F 2 "" H 1400 900 50  0001 C CNN
F 3 "" H 1400 900 50  0001 C CNN
	1    1400 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 1050 1400 1050
Wire Wire Line
	1400 1050 1400 900 
Wire Wire Line
	1400 1250 1400 1150
Wire Wire Line
	1400 1150 1200 1150
NoConn ~ 1850 2400
NoConn ~ 1850 2500
NoConn ~ 1850 2700
NoConn ~ 1850 2800
NoConn ~ 1850 2900
NoConn ~ 1850 3000
NoConn ~ 1850 3100
NoConn ~ 1850 3200
NoConn ~ 1850 3400
NoConn ~ 1850 3500
NoConn ~ 1850 3600
NoConn ~ 3500 4200
NoConn ~ 3500 4100
NoConn ~ 3500 4000
NoConn ~ 3500 3900
NoConn ~ 3500 3750
NoConn ~ 3500 3650
NoConn ~ 3500 3550
NoConn ~ 3500 3400
NoConn ~ 3500 3300
NoConn ~ 3500 3200
NoConn ~ 3500 3050
NoConn ~ 3500 2950
NoConn ~ 3500 2850
NoConn ~ 3500 2750
NoConn ~ 3500 2650
NoConn ~ 3500 2500
NoConn ~ 5350 3350
NoConn ~ 5350 4050
NoConn ~ 6350 3150
NoConn ~ 6350 2850
NoConn ~ 6350 2750
NoConn ~ 6350 3950
NoConn ~ 6350 4050
NoConn ~ 1850 3900
NoConn ~ 2450 2100
NoConn ~ 2850 2100
NoConn ~ 5750 2350
NoConn ~ 5950 2350
NoConn ~ 7950 3400
$Comp
L power:GND #PWR02
U 1 1 5E5FCC13
P 1400 1250
F 0 "#PWR02" H 1400 1000 50  0001 C CNN
F 1 "GND" H 1405 1077 50  0000 C CNN
F 2 "" H 1400 1250 50  0001 C CNN
F 3 "" H 1400 1250 50  0001 C CNN
	1    1400 1250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5E61069E
P 3750 2400
F 0 "R1" V 3800 2550 50  0000 C CNN
F 1 "1k" V 3750 2400 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 3680 2400 50  0001 C CNN
F 3 "~" H 3750 2400 50  0001 C CNN
	1    3750 2400
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 5E6106F5
P 3750 2300
F 0 "R2" V 3700 2450 50  0000 C CNN
F 1 "0R" V 3750 2300 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 3680 2300 50  0001 C CNN
F 3 "~" H 3750 2300 50  0001 C CNN
	1    3750 2300
	0    1    1    0   
$EndComp
Wire Wire Line
	2550 4400 2550 4550
Wire Wire Line
	2550 4550 2600 4550
Wire Wire Line
	2650 4550 2650 4400
Wire Wire Line
	2650 4550 2750 4550
Wire Wire Line
	2750 4550 2750 4400
Connection ~ 2650 4550
Wire Wire Line
	2600 4550 2600 4700
Connection ~ 2600 4550
Wire Wire Line
	2600 4550 2650 4550
$Comp
L power:GND #PWR0101
U 1 1 5E61F5D2
P 9100 3700
F 0 "#PWR0101" H 9100 3450 50  0001 C CNN
F 1 "GND" H 9105 3527 50  0000 C CNN
F 2 "" H 9100 3700 50  0001 C CNN
F 3 "" H 9100 3700 50  0001 C CNN
	1    9100 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	9100 3700 9100 3400
Connection ~ 9100 3400
Wire Wire Line
	5850 4350 5850 4400
Wire Wire Line
	5850 4400 5900 4400
Wire Wire Line
	5950 4400 5950 4350
Wire Wire Line
	5900 4400 5900 4500
Connection ~ 5900 4400
Wire Wire Line
	5900 4400 5950 4400
Wire Wire Line
	3500 2300 3600 2300
Wire Wire Line
	3500 2400 3600 2400
NoConn ~ 1850 3700
NoConn ~ 1850 3800
Wire Wire Line
	4750 2300 4750 2750
Wire Wire Line
	4750 2750 5350 2750
Wire Wire Line
	3900 2300 4750 2300
Wire Wire Line
	5350 2850 4650 2850
Wire Wire Line
	4650 2850 4650 2400
Wire Wire Line
	3900 2400 4650 2400
$Comp
L Graphic:SYM_Radio_Waves_Small SYM1
U 1 1 5E995B99
P 10800 6300
F 0 "SYM1" H 10800 6440 50  0001 C CNN
F 1 "Logo" H 10800 6175 50  0001 C CNN
F 2 "ESP-GPIB:MC" H 10800 6125 50  0001 C CNN
F 3 "~" H 10830 6100 50  0001 C CNN
	1    10800 6300
	1    0    0    -1  
$EndComp
$EndSCHEMATC
