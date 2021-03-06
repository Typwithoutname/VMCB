EESchema Schematic File Version 4
EELAYER 30 0
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
L Device:R R3
U 1 1 5EE3AF75
P 8400 3100
F 0 "R3" H 8470 3146 50  0000 L CNN
F 1 "150 Ohm" H 8470 3055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 8330 3100 50  0001 C CNN
F 3 "~" H 8400 3100 50  0001 C CNN
	1    8400 3100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5EE3B4D7
P 8400 1700
F 0 "R2" H 8470 1746 50  0000 L CNN
F 1 "120 Ohm" H 8470 1655 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 8330 1700 50  0001 C CNN
F 3 "~" H 8400 1700 50  0001 C CNN
	1    8400 1700
	1    0    0    -1  
$EndComp
$Comp
L Device:LED LED2
U 1 1 5EE899FD
P 8400 2300
F 0 "LED2" V 8439 2182 50  0000 R CNN
F 1 "yellow" V 8348 2182 50  0000 R CNN
F 2 "LED_SMD:LED_PLCC-2" H 8400 2300 50  0001 C CNN
F 3 "~" H 8400 2300 50  0001 C CNN
	1    8400 2300
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED LED3
U 1 1 5EE89DD8
P 8400 3650
F 0 "LED3" V 8439 3532 50  0000 R CNN
F 1 "green" V 8348 3532 50  0000 R CNN
F 2 "LED_SMD:LED_PLCC-2" H 8400 3650 50  0001 C CNN
F 3 "~" H 8400 3650 50  0001 C CNN
	1    8400 3650
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED LED1
U 1 1 5EE8A614
P 9100 2300
F 0 "LED1" V 9139 2182 50  0000 R CNN
F 1 "red" V 9048 2182 50  0000 R CNN
F 2 "LED_SMD:LED_PLCC-2" H 9100 2300 50  0001 C CNN
F 3 "~" H 9100 2300 50  0001 C CNN
	1    9100 2300
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R1
U 1 1 5EE8AD00
P 9100 1700
F 0 "R1" H 9170 1746 50  0000 L CNN
F 1 "120 Ohm" H 9170 1655 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 9030 1700 50  0001 C CNN
F 3 "~" H 9100 1700 50  0001 C CNN
	1    9100 1700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5EE8BC61
P 1100 1500
F 0 "#PWR0101" H 1100 1250 50  0001 C CNN
F 1 "GND" H 1105 1327 50  0000 C CNN
F 2 "" H 1100 1500 50  0001 C CNN
F 3 "" H 1100 1500 50  0001 C CNN
	1    1100 1500
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0102
U 1 1 5EE8CDA8
P 2700 1300
F 0 "#PWR0102" H 2700 1150 50  0001 C CNN
F 1 "+3.3V" H 2715 1473 50  0000 C CNN
F 2 "" H 2700 1300 50  0001 C CNN
F 3 "" H 2700 1300 50  0001 C CNN
	1    2700 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5EE8E3F7
P 1000 4400
F 0 "#PWR0103" H 1000 4150 50  0001 C CNN
F 1 "GND" V 1005 4272 50  0000 R CNN
F 2 "" H 1000 4400 50  0001 C CNN
F 3 "" H 1000 4400 50  0001 C CNN
	1    1000 4400
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0104
U 1 1 5EE8EBCB
P 1000 4300
F 0 "#PWR0104" H 1000 4150 50  0001 C CNN
F 1 "+3.3V" H 1015 4473 50  0000 C CNN
F 2 "" H 1000 4300 50  0001 C CNN
F 3 "" H 1000 4300 50  0001 C CNN
	1    1000 4300
	1    0    0    -1  
$EndComp
Text GLabel 2100 4200 0    50   Input ~ 0
PB7-LEDs
Text GLabel 5200 4500 2    50   Input ~ 0
PA8-LEDs
$Comp
L power:+3.3V #PWR0105
U 1 1 5EE94D4B
P 8750 1350
F 0 "#PWR0105" H 8750 1200 50  0001 C CNN
F 1 "+3.3V" H 8765 1523 50  0000 C CNN
F 2 "" H 8750 1350 50  0001 C CNN
F 3 "" H 8750 1350 50  0001 C CNN
	1    8750 1350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 5EE95484
P 8400 4050
F 0 "#PWR0106" H 8400 3800 50  0001 C CNN
F 1 "GND" H 8405 3877 50  0000 C CNN
F 2 "" H 8400 4050 50  0001 C CNN
F 3 "" H 8400 4050 50  0001 C CNN
	1    8400 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	8400 3800 8400 4050
Wire Wire Line
	8400 3250 8400 3500
Wire Wire Line
	8400 2950 8400 2650
Connection ~ 8400 2650
Wire Wire Line
	8400 2650 8400 2450
Wire Wire Line
	8400 1850 8400 2150
Wire Wire Line
	8400 1550 8400 1350
Wire Wire Line
	8400 1350 8750 1350
Wire Wire Line
	9100 1350 9100 1550
Wire Wire Line
	9100 1850 9100 2150
Text GLabel 9100 2950 3    50   Input ~ 0
PB7-LEDs
Text GLabel 8050 2650 0    50   Input ~ 0
PA8-LEDs
Wire Wire Line
	8400 2650 8050 2650
Wire Wire Line
	9100 2450 9100 2950
Connection ~ 8750 1350
Wire Wire Line
	8750 1350 9100 1350
$Comp
L Device:R R4
U 1 1 5EEFE0D3
P 4700 1950
F 0 "R4" H 4770 1996 50  0000 L CNN
F 1 "4.7 MOhm" H 4770 1905 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 4630 1950 50  0001 C CNN
F 3 "~" H 4700 1950 50  0001 C CNN
	1    4700 1950
	1    0    0    -1  
$EndComp
Text GLabel 5200 4400 2    50   Input ~ 0
PA12ReceivePin-kS
Text GLabel 1800 4700 0    50   Input ~ 0
PA1SendPin-kS
Text GLabel 4200 1700 0    50   Input ~ 0
PA1SendPin-kS
Text GLabel 4200 2200 0    50   Input ~ 0
PA12ReceivePin-kS
Wire Wire Line
	4200 1700 4700 1700
Wire Wire Line
	4700 1700 4700 1800
Wire Wire Line
	4200 2200 4700 2200
Wire Wire Line
	4700 2100 4700 2200
$Comp
L Connector_Generic:Conn_01x01 J1
U 1 1 5EF35B4C
P 5800 2200
F 0 "J1" H 5880 2242 50  0000 L CNN
F 1 "Conn_01x01" H 5880 2151 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 5800 2200 50  0001 C CNN
F 3 "~" H 5800 2200 50  0001 C CNN
	1    5800 2200
	1    0    0    -1  
$EndComp
$Comp
L Stimmungslicht-rescue:HU2032-LF-HU2032-LF U1
U 1 1 5EF3D88A
P 1550 1350
F 0 "U1" H 2000 1615 50  0000 C CNN
F 1 "HU2032-LF" H 2000 1524 50  0000 C CNN
F 2 "libs_fp:HU2032LF" H 2300 1450 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/2/HU2032-LF.pdf" H 2300 1350 50  0001 L CNN
F 4 "RENATA - HU2032-LF - HOLDER, BATTERY, THT, CR2032" H 2300 1250 50  0001 L CNN "Description"
F 5 "5.35" H 2300 1150 50  0001 L CNN "Height"
F 6 "RENATA" H 2300 1050 50  0001 L CNN "Manufacturer_Name"
F 7 "HU2032-LF" H 2300 950 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "HU2032-LF" H 2300 850 50  0001 L CNN "Arrow Part Number"
F 9 "" H 2300 750 50  0001 L CNN "Arrow Price/Stock"
F 10 "614-HU2032-LF" H 2300 650 50  0001 L CNN "Mouser Part Number"
F 11 "https://www.mouser.co.uk/ProductDetail/Renata/HU2032-LF?qs=oUsD4qhOtFwA99zRPu3JIQ%3D%3D" H 2300 550 50  0001 L CNN "Mouser Price/Stock"
	1    1550 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 1350 2700 1350
Wire Wire Line
	2700 1350 2700 1300
Wire Wire Line
	1550 1350 1100 1350
Wire Wire Line
	1100 1350 1100 1450
Wire Wire Line
	1550 1450 1100 1450
Connection ~ 1100 1450
Wire Wire Line
	1100 1450 1100 1500
$Comp
L power:+3.3V #PWR0107
U 1 1 5EF43F2C
P 7650 5600
F 0 "#PWR0107" H 7650 5450 50  0001 C CNN
F 1 "+3.3V" H 7665 5773 50  0000 C CNN
F 2 "" H 7650 5600 50  0001 C CNN
F 3 "" H 7650 5600 50  0001 C CNN
	1    7650 5600
	1    0    0    -1  
$EndComp
Text GLabel 8100 5700 0    50   Input ~ 0
on-off
$Comp
L Stimmungslicht-rescue:STM32G031J6M6-STM32G031J6M6 IC1
U 1 1 5EE382F6
P 2100 4200
F 0 "IC1" H 3650 4465 50  0000 C CNN
F 1 "STM32G031J6M6" H 3650 4374 50  0000 C CNN
F 2 "libs_fp:SOIC127P600X175-8N" H 5050 4300 50  0001 L CNN
F 3 "https://www.mouser.com/datasheet/2/389/en.DM00371828-1620887.pdf" H 5050 4200 50  0001 L CNN
F 4 "ARM Microcontrollers - MCU" H 5050 4100 50  0001 L CNN "Description"
F 5 "1.75" H 5050 4000 50  0001 L CNN "Height"
F 6 "STMicroelectronics" H 5050 3900 50  0001 L CNN "Manufacturer_Name"
F 7 "STM32G031J6M6" H 5050 3800 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "STM32G031J6M6" H 5050 3700 50  0001 L CNN "Arrow Part Number"
F 9 "https://www.arrow.com/en/products/stm32g031j6m6/stmicroelectronics" H 5050 3600 50  0001 L CNN "Arrow Price/Stock"
F 10 "511-STM32G031J6M6" H 5050 3500 50  0001 L CNN "Mouser Part Number"
F 11 "https://www.mouser.co.uk/ProductDetail/STMicroelectronics/STM32G031J6M6?qs=uwxL4vQweFOrzBadnQ6n1g%3D%3D" H 5050 3400 50  0001 L CNN "Mouser Price/Stock"
	1    2100 4200
	1    0    0    -1  
$EndComp
Text GLabel 1800 4600 0    50   Input ~ 0
on-off
Text GLabel 5700 6100 2    50   Input ~ 0
Conn-RST
Text GLabel 6250 5800 2    50   Input ~ 0
Conn-SWDIO
Text GLabel 6250 5900 2    50   Input ~ 0
Conn-SWCLK
$Comp
L power:GND #PWR0110
U 1 1 5EF4FC6C
P 6450 6050
F 0 "#PWR0110" H 6450 5800 50  0001 C CNN
F 1 "GND" H 6455 5877 50  0000 C CNN
F 2 "" H 6450 6050 50  0001 C CNN
F 3 "" H 6450 6050 50  0001 C CNN
	1    6450 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 6000 6450 6050
Wire Wire Line
	6450 6000 5700 6000
Text GLabel 5200 4300 2    50   Input ~ 0
Conn-SWDIO
Text GLabel 5200 4200 2    50   Input ~ 0
Conn-SWCLK
Text GLabel 1800 4500 0    50   Input ~ 0
Conn-RST
Wire Wire Line
	1000 4300 2100 4300
Wire Wire Line
	1000 4400 2100 4400
Wire Wire Line
	2100 4500 1950 4500
Wire Wire Line
	1950 4500 1950 4600
Wire Wire Line
	1950 4600 1800 4600
Connection ~ 1950 4500
Wire Wire Line
	1950 4500 1800 4500
Wire Wire Line
	1950 4600 1950 4700
Wire Wire Line
	1950 4700 1800 4700
Connection ~ 1950 4600
$Comp
L Connector:Conn_01x04_Male J2
U 1 1 5F0F49B6
P 5500 5900
F 0 "J2" H 5608 6181 50  0000 C CNN
F 1 "Conn_01x04_Male" H 5608 6090 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 5500 5900 50  0001 C CNN
F 3 "~" H 5500 5900 50  0001 C CNN
	1    5500 5900
	1    0    0    -1  
$EndComp
$Comp
L Stimmungslicht-rescue:JS102011SAQN-JS102011SAQN S1
U 1 1 5F0FD9F0
P 8100 5700
F 0 "S1" H 8500 5965 50  0000 C CNN
F 1 "JS102011SAQN" H 8500 5874 50  0000 C CNN
F 2 "libs_fp:JS102011SAQN" H 8750 5800 50  0001 L CNN
F 3 "http://www.ckswitches.com/media/1422/js.pdf" H 8750 5700 50  0001 L CNN
F 4 "C & K COMPONENTS - JS102011SAQN - SWITCH, SPDT, 0.6A, 6VDC, SIDE, SMD" H 8750 5600 50  0001 L CNN "Description"
F 5 "" H 8750 5500 50  0001 L CNN "Height"
F 6 "C & K COMPONENTS" H 8750 5400 50  0001 L CNN "Manufacturer_Name"
F 7 "JS102011SAQN" H 8750 5300 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "JS102011SAQN" H 8750 5200 50  0001 L CNN "Arrow Part Number"
F 9 "https://www.arrow.com/en/products/js102011saqn/ck" H 8750 5100 50  0001 L CNN "Arrow Price/Stock"
F 10 "611-JS102011SAQN" H 8750 5000 50  0001 L CNN "Mouser Part Number"
F 11 "https://www.mouser.co.uk/ProductDetail/CK/JS102011SAQN?qs=LgMIjt8LuD%252B69bNM9a%2FozQ%3D%3D" H 8750 4900 50  0001 L CNN "Mouser Price/Stock"
	1    8100 5700
	1    0    0    1   
$EndComp
Wire Wire Line
	7650 5600 8100 5600
NoConn ~ 8900 5700
Wire Wire Line
	4700 2200 5200 2200
Wire Wire Line
	5200 2200 5200 1950
Wire Wire Line
	5200 1950 5350 1950
Connection ~ 4700 2200
Wire Wire Line
	5500 2150 5500 2200
Wire Wire Line
	5500 2200 5600 2200
Wire Wire Line
	5500 1700 5500 1750
$Comp
L Device:R R101
U 1 1 5F113686
P 6000 5800
F 0 "R101" V 5793 5800 50  0000 C CNN
F 1 "0" V 5884 5800 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 5930 5800 50  0001 C CNN
F 3 "~" H 6000 5800 50  0001 C CNN
	1    6000 5800
	0    1    1    0   
$EndComp
$Comp
L Device:R R102
U 1 1 5F113F53
P 6000 5900
F 0 "R102" V 5793 5900 50  0000 C CNN
F 1 "0" V 5884 5900 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 5930 5900 50  0001 C CNN
F 3 "~" H 6000 5900 50  0001 C CNN
	1    6000 5900
	0    1    1    0   
$EndComp
Wire Wire Line
	5700 5900 5850 5900
Wire Wire Line
	5700 5800 5850 5800
Wire Wire Line
	6150 5800 6250 5800
Wire Wire Line
	6250 5900 6150 5900
$Comp
L Jumper:SolderJumper_3_Bridged12 JP101
U 1 1 5F101610
P 5500 1950
F 0 "JP101" V 5546 2018 50  0000 L CNN
F 1 "SolderJumper_3_Bridged12" V 5455 2018 50  0000 L CNN
F 2 "Jumper:SolderJumper-3_P1.3mm_Bridged12_RoundedPad1.0x1.5mm" H 5500 1950 50  0001 C CNN
F 3 "~" H 5500 1950 50  0001 C CNN
	1    5500 1950
	0    1    1    0   
$EndComp
$Comp
L Stimmungslicht-rescue:Touch_Sens_Pad-Touch_Sens_Pad U101
U 1 1 5F1231CE
P 5650 1600
F 0 "U101" H 5950 1650 50  0000 R CNN
F 1 "Touch_Sens_Pad" H 6450 1550 50  0000 R CNN
F 2 "libs_fp:Touch_Sens_Pad" H 5650 1600 50  0001 C CNN
F 3 "" H 5650 1600 50  0001 C CNN
	1    5650 1600
	0    -1   -1   0   
$EndComp
$Comp
L Stimmungslicht-rescue:Touch_Sens_Pad-Touch_Sens_Pad U102
U 1 1 5F123C1E
P 5850 1600
F 0 "U102" H 6150 1650 50  0000 R CNN
F 1 "Touch_Sens_Pad" H 6650 1550 50  0000 R CNN
F 2 "libs_fp:Touch_Sens_Pad" H 5850 1600 50  0001 C CNN
F 3 "" H 5850 1600 50  0001 C CNN
	1    5850 1600
	0    -1   -1   0   
$EndComp
$Comp
L Stimmungslicht-rescue:Touch_Sens_Pad-Touch_Sens_Pad U103
U 1 1 5F123EC0
P 6050 1600
F 0 "U103" H 6350 1650 50  0000 R CNN
F 1 "Touch_Sens_Pad" H 6850 1550 50  0000 R CNN
F 2 "libs_fp:Touch_Sens_Pad" H 6050 1600 50  0001 C CNN
F 3 "" H 6050 1600 50  0001 C CNN
	1    6050 1600
	0    -1   -1   0   
$EndComp
$Comp
L Stimmungslicht-rescue:Touch_Sens_Pad-Touch_Sens_Pad U104
U 1 1 5F1240DE
P 6250 1600
F 0 "U104" H 6550 1650 50  0000 R CNN
F 1 "Touch_Sens_Pad" H 7050 1550 50  0000 R CNN
F 2 "libs_fp:Touch_Sens_Pad" H 6250 1600 50  0001 C CNN
F 3 "" H 6250 1600 50  0001 C CNN
	1    6250 1600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5500 1700 5650 1700
Wire Wire Line
	5650 1700 5850 1700
Connection ~ 5650 1700
Wire Wire Line
	5850 1700 6050 1700
Connection ~ 5850 1700
Wire Wire Line
	6050 1700 6250 1700
Connection ~ 6050 1700
$EndSCHEMATC
