;**********************************************************************
;  Filename: OBD2                                                     *
;  Uses PIC16F876A                                                    *
;  Date: 13.05.2018                                                   *
;  Author:  Jose Jesus                                                *
;                                                                     *
;  Uses:
;        
;                                                                     *
;        RC2004A  20*4 Display interface
;        MCP2515  CAN Controller
;        MCP2551  CAN Transceiver
;        MAX232   USART data manager
;                                                                     *
;  4 MHz Osc                                                         *
;                                                                     *
;                                                                     *
;                                                                     *
;                                                                     *
; Tosc = (1/4  MHz)   = 0.00000025s = 250ns                           *
; 1 instruction  = TOsc * 4 = 1us                                     *
;
;
;                                                                     *
;                                                                     *
;**********************************************************************
;                                                                     *
;**********************************************************************
;                                                                     *                                    
;    Pin assignments:                                                 *                                    
;     2 - RA0-0             = D7 Display                              *                                 
;     3 - RA1-0             = D6 Display         
;     4 - RA2-0             = D5 Display                              *                                  
;     5 - RA3-0             = D4 Display                              *
;     6 - RA4-0             = EN Display                              *
;     7 - RA5-0             = RS Display                              *
;    21 - RB0-1             =  INT CAN                             *
;    22 - RB1-1             =  button left          !!                    *
;    23 - RB2-1             =  button up                            *
;    24 - RB3-1             =  Button right                             *
;    25 - RB4-1             =  Button down                            *                                
;    26 - RB5-1             =  Button enter                           *
;        RB6   K in
;        RB7   out CAN/ISO select
;        RC0   K Line
;        RC1   L Line
;        RC2   CS
;        RC3   SCK
;        RC4   SI   
;        RC5   SO
;        RC6   TX
;        RC7   RX   
;
;                                                                     *
;                                                                     *
;*************************************************************************


 




dVersion equ 1
dRelease equ 0


	PROCESSOR 16F876a
    
 
    LIST r=dec, x=on, t=off  

	#include "p16f876a.inc"
    
 

	; embed Configuration Data within .asm File.
	__CONFIG   _WRT_OFF & _CPD_OFF & _CP_OFF &  _WDT_OFF & _LVP_OFF & _BODEN_ON & _PWRTE_ON & _XT_OSC 
    __IDLOCS (dVersion<<8)|dRelease; version: vvrr; vv - version, rr - release



    TMR_COUNT equ 0xd9


       cblock 0x30
temp
baudRate
tmr0
SData
count
      endc




       org 0x00
    
      nop     
      goto main

       org 0x04
     
      goto isr




isr
    nop
    iret

    #include "Lcd.inc"
;    #include "Can_lib.inc"
    #include "Iso_lib.inc"
main
     call SetupISO

     call LCDinit  

 ;    call ISO_9141_2
     call ISO_14230
 
      
 
     
wait
     nop
     goto wait






GetStrAddr:
    addwf PCL,F
    retlw Str0-Str0
    retlw Str1-Str0
    retlw Str2-Str0
    retlw Str3-Str0
    retlw Str4-Str0
    retlw Str5-Str0
    retlw Str6-Str0
    retlw Str7-Str0
    retlw Str8-Str0
    retlw Str9-Str0



Strings
    addwf PCL, F
Str0: dt " O B D 2 ",0x00
Str1: dt "ISO 9141",0x00  
Str2: dt "CAN    bits/    Kbps",0x00 
Str3: dt "msg ID",0x00 
Str4: dt "Normal mode",0x00     
Str5: dt "Sleep mode",0x00     
Str6: dt "LoopBack mode",0x00     
Str7: dt "Listen mode",0x00     
Str8: dt "Config mode",0x00  
Str9: dt "  Electro-Solucoes",0x00   




SetupISO
 
    ; Cfg_ISO

    
    clrwdt
    BANK1

     movlw b'00000110'
     movwf ADCON1        ; Set PORTA Digital I/O
   



  ; TRM0 config 
     movlw b'10000101'       ; configure Timer0
           ; 1-------       PORTB pull-up resistor dissable 
           ; -0------       Interrupt falling edge RB0
           ; --0-----       TMR0 source clock internal(TOCS = 0)
           ; --------       TMR0 source edge of RA4/INT
           ; ----0---       prescaler assigned to Timer0 (PSA = 0)
           ; -----101       prescale = 1:64 (PS = 101)
                          ; TMR0 0.27127 * 64 = 17.361 
     movwf OPTION_REG

  ; Clear BANK0
     movlw 0x20
     movwf FSR
jInitClr1
     clrf INDF
     incf FSR,F
     jmpClr FSR,7, jInitClr1        

   ; Clear BANK1
     movlw 0xA0
     movwf FSR
jInitClr2
     clrf INDF
     incf FSR,F
     jmpClr FSR,7, jInitClr2        

     call InitIO
    ; config TIMER1
     BANK1
     bsf _TMR1IE_P   ; 00000001

     BANK0
     clrf TMR1H
     clrf TMR1L

     
;     movlw 0x40
;     call CheckHD
;     call LCD_String
     
   
    return




;************************************************

InitIO
     BANK0
     clrf PORTA
     clrf PORTB
     clrf PORTC
     bsf PORTC,2

     BANK1
     
     ; PORTA
     ;      0     display d4       
     ;      1     display d5       
     ;      2     display d6       
     ;      3     display d7       
     ;      4     display en       
     ;      5     display rs  
     clrf TRISA     

     ; PORTB
     ;       0 in key left 
     ;       1 in key left
     ;       2 in key enter
     ;       3 in key right
     ;       4 in key down 
     ;       5 in key up 
     ;       6 in  K Line 
     ;       7 - CAN/ISO selection
     movlw b'01111111'
     movwf TRISB
   
     ; PORTC
     ;      0   out K Line
     ;      1   out L Line  
     ;      2   X out CS  
     ;      3   X out SPI clock - master 
     ;      4   X in SPI data - SO
     ;      5   X out SPI data out - SI
     ;      6   TX 
     ;      7   RX 
     movlw b'11111100'
     movwf TRISC
     BANK0
     return



;**************************************************

ISO_14230

    bcf PORTB,7   ; Enable ISO
    bcf PORTC,0   ; Bus hidle hi
    
   
   ; Set baudRate to 5bits per second (T=200ms)
    movlw 0xC8
    movwf baudRate

    movlw 0xff       ; 256ms
    call delay_ms
    movlw 0x2C       ;  44ms
    call delay_ms
rep
    movlw 0x33   ; Init sequence
    call ISO_Write
    goto rep

    call ISO_Read
    sublw 0x55
    btfss STATUS,Z
    goto InitErr
    ; Init ISO success
    ; set baudRate to 10.4 Kbps
    movlw 0x60
    movwf baudRate
    ; wait for key bytes
    call ISO_Read
    btfsc STATUS,Z
    goto InitErr
 ;   test received byte
    call ISO_Read
    btfsc STATUS,Z
    goto InitErr
    movwf temp
    swapf temp,W
    call ISO_Write
    LCD_Text 1,"ISO 14230_4 OK"
    retlw 1
InitErr 
    LCD_Text 1,"ISO 14230-4 Fail"  
    retlw 0


     end