# OBD II
Automotive  


OBD-II Protocols:

  - ISO 15765 (CAN – Controller Area Network) – 2008

        . Feature	Description
        . CAN HIGH (CAN H)	Pin 6
        . CAN LOW (CAN L)	Pin 14
        . 12V	Pin 16
        . GND	Pins 4, 5
        . Bus State:	Active when CANH pulled HIGH, CANL pulled LOW. Idle when signals are floating.
        . CANH Signal Voltage:	+3.5V
        . CANL Signal Voltage:	+1.5V
        . Maximum Signal Voltage:	CANH = +4.5V, CANL = +2.25V
        . Minimum Signal Voltage:	CANH = +2.75V, CANL = +0.5V
        . Number of bytes:	L
        . Bit Timing:	250kbit/sec or 500kbit/sec

  - ISO 9141-2 
        . Feature	Description
        . K Line (bidirectional)	Pin 7
        . L Line (unidirectional, optional)	Pin 15
        . 12V	Pin 16
        . GND	Pins 4, 5
        . Bus State:	K Line idles HIGH. Bus is active when driven LOW.
        . Maximum Signal Voltage:	+12V
        . Minimum Signal Voltage:	0V
        . Number of bytes:	Message: 260, Data: 255
        . Bit Timing:	UART: 10400bps, 8-N-1

  - ISO 14230 KWP2000 (Keyword Protocol 2000)
        . Feature	Description
        . K Line (bidirectional)	Pin 7
        . L Line (unidirectional, optional)	Pin 15
        . 12V	Pin 16
        . GND	Pins 4, 5
        . Bus State:	Active when driven LOW.
        . Maximum Signal Voltage:	+12V
        . Minimum Signal Voltage:	0V
        . Number of bytes:	Data: 255
        . Bit Timing:	UART: 10400bps, 8-N-1

  - SAE J1850 PWM (Pulse Width Modulation)
        . Feature	Description
        . BUS +	Pin 2
        . BUS -	Pin 10
        . 12V	Pin 16
        . GND	Pins 4, 5
        . Bus State:	Active when BUS + is pulled HIGH, BUS - is pulled LOW
        . Maximum Signal Voltage:	5V
        . Minimum Signal Voltage:	0V
        . Number of bytes:	12
        . Bit Timing:	'1' bit - 8uS, '0' bit - 16uS, Start of Frame - 48uS

  - SAE J1850 VPW (Variable Pulse Width)
        . Feature	Description
        . BUS +	Pin 2
        . 12V	Pin 16
        . GND	Pins 4, 5
        . Bus State:	Bus idles low
        . Maximum Signal Voltage:	+7V
        . Decision Signal Voltage:	+3.5V
        . Minimum Signal Voltage:	0V
        . Number of bytes:	12
        . Bit Timing:	'1' bit -HIGH 64uS, '0' bit -HIGH 128uS, Start of Frame - HIGH 200uS

   
Work in progress (not tested) 

Referencias:

    
    
    - http://www.onboarddiagnostics.com/page03.htm
