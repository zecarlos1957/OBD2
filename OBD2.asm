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
;    27 - RB6-1             =                            *
;    28 - RB7-1             =                             *
;    11 - RC0-1             = 
;    12 - RC1-0             =                           *
;    13 - RC2-0             = CS  CAN 
;    14 - RC3-0             = SCK CAN 
;    15 - RC4-0             = SO  CAN
;    16 - RC5-O             = SI  CAN                               *
;    17 - RC6- TX           =                        *
;    18 - RC7- RX           =                       *
;
;                                                                     *
;                                                                     *
;*************************************************************************






;***** COMPILATION MESSAGES & WARNINGS *****

	ERRORLEVEL -207 	; Found label after column 1.
	ERRORLEVEL -302 	; Register in operand not in bank 0.
;    ERRORLEVEL -305     ; using default destination

;***** PROCESSOR DECLARATION & CONFIGURATION *****

dVersion equ 1
dRelease equ 0


 	PROCESSOR 16F876a

    LIST r=dec, x=on, t=off  

	#include "p16f876a.inc"

   #include "MACROS16.inc"

	; embed Configuration Data within .asm File.
	__CONFIG   _WRT_OFF & _CPD_OFF & _CP_OFF &  _WDT_OFF & _LVP_OFF & _BODEN_ON & _PWRTE_ON & _XT_OSC 
    __IDLOCS (dVersion<<8)|dRelease; version: vvrr; vv - version, rr - release

     #define menu_stage CanType,7
     #define data_nibble CanType,6
     #define config_mode CanType,5
     #define ertx_flag  CanType,4
     #define baudrate_flag CanType, 1
     #define frametype_flag CanType,0

     #define tp2510_CS_ PORTC,2   ; Chip select

;***** CONSTANT DECLARATION *****

	CONSTANT BASE = 0x20		; base address of user file registers

;***** CONSTANT DECLARATION *****

	IFNDEF	LCDLINENUM	; use default value, if unspecified
;		CONSTANT LCDLINENUM = 0x02	; by default, 2 lines
		CONSTANT LCDLINENUM = 0x04
	ENDIF
	IFNDEF	LCDTYPE		; use default value, if unspecified
	;	CONSTANT LCDTYPE = 0x00		; standard HD44780 LCD
		CONSTANT LCDTYPE = 0x01	; EADIP204-4 (w/ KS0073)
	ENDIF
	IFNDEF	LCDSPEED	; use default value, if unspecified
		;CONSTANT LCDSPEED = 0x00	; clk in [0..9] MHz
		CONSTANT LCDSPEED = 0x01	; clk in [9..20] MHz, default
	ENDIF
	IFNDEF	LCDWAIT		; use default value, if unspecified
;		CONSTANT LCDWAIT = 0x01		; for Tosc <= 5 MHz
		CONSTANT LCDWAIT = 0x04		; for Tosc = 10 MHz
	ENDIF
	IFNDEF	LCDCLRWAIT	; use default value, if unspecified
		CONSTANT LCDCLRWAIT = 0x08	; wait after LCDCLR until LCD is ready again
	ENDIF

;***** REGISTER DECLARATION *****

OldLATH	set	BASE+d'0'	; wait cycle counter

LCDbuf	set	BASE+d'1'	; LCD data buffer
LCDtemp	set	BASE+d'2'	; LCD temporary register

           ; m_lcdv08
LO	equ	BASE+d'3'
HI	equ	BASE+d'4'
LO_TEMP	set	BASE+d'5'
HI_TEMP	set	BASE+d'6'

TEMP1	equ	BASE+d'7'	; Universal Temporary Register
TEMP3	equ	BASE+d'8'
TEMP2	equ	BASE+d'9'
TEMP4	equ	BASE+d'10'
TEMP5	equ	BASE+d'11'

LCDFLAGreg	equ	BASE+d'12'

STR_NUM equ BASE+d'13'
INDEX   equ BASE+d'14'

	#define	BCflag   LCDFLAGreg,0x05

	;*** LCD module versions for fixed ports (e.g. PortB) ***
LCDtris	equ	TRISA
LCDport	equ	PORTA

    #define	LCD_EN     PORTA,0x04	; Enable Output / "CLK"
    #define	LCD_RS     PORTA,0x05	; Register Select

;***** LCD COMMANDS *****

;*** Standard LCD COMMANDS for INIT ***	( HI-NIBBLE only )

	; for 4 bit mode: send only one nibble as high-nibble [DB7:DB4]
	CONSTANT  LCDEM8  = b'0011'	; entry mode set: 8 bit mode, 2 lines
	CONSTANT  LCDEM4  = b'0010'	; entry mode set: 4 bit mode, 2 lines
	CONSTANT  LCDDZ   = b'1000'	; set Display Data Ram Address to zero

  ;*** Standard LCD COMMANDS ***		( HI- / LO-NIBBLE )
	; USE THESE COMMANDS BELOW AS FOLLOW: "LCDcmd LCDCLR"
	CONSTANT  LCDCLR  = b'00000001'	; 0x01 clear display: resets address counter & cursor
	CONSTANT  LCDCH   = b'00000010'	; 0x02 cursor home
	CONSTANT  LCDCR   = b'00000110'	; 0x06 entry mode set: cursor moves right, display auto-shift off
	CONSTANT  LCDCL   = b'00000100'	; 0x04 entry mode set: cursor moves left, display auto-shift off
	CONSTANT  LCDCONT = b'00001100'	; 0x0C display control: display on, cursor off, blinking off
	CONSTANT  LCDMCL  = b'00010000'	; 0x10 cursor/disp control: move cursor left
	CONSTANT  LCDMCR  = b'00010100'	; 0x14 cursor/disp control: move cursor right
	CONSTANT  LCDSL   = b'00011000'	; 0x18 cursor/disp control: shift display content left
	CONSTANT  LCDSR   = b'00011100'	; 0x1C cursor/disp control: shift display content right
	CONSTANT  LCD2L   = b'00101000'	; 0x28 function set: 4 bit mode, 2 lines, 5x7 dots
	IF (LCDLINENUM == 0x2)
	CONSTANT  LCDL1 = b'10000000'	; DDRAM address: 0x00, selects line 1 (2xXX LCD)
	CONSTANT  LCDL2 = b'11000000'	; DDRAM address: 0x40, selects line 2 (2xXX LCD)
	CONSTANT  LCDL3 = b'10010100'	; (DDRAM address: 0x14, fallback)
	CONSTANT  LCDL4 = b'11010100'	; (DDRAM address: 0x54, fallback)
	ELSE
	CONSTANT  LCDL1 = b'10000000'	; DDRAM address: 0x00, selects line 1 (4xXX LCD)
	CONSTANT  LCDL2 = b'11000000'	; DDRAM address: 0x40, selects line 2 (4xXX LCD)
	CONSTANT  LCDL3 = b'10010100'	; DDRAM address: 0x14, selects line 3 (4xXX LCD)
	CONSTANT  LCDL4 = b'11010100'	; DDRAM address: 0x54, selects line 4 (4xXX LCD)
	ENDIF
	; special configuration for EA DIP204-4
	CONSTANT  LCDEXT  = b'00001001'	; 0x09 extended function set EA DIP204-4
	CONSTANT  LCD2L_A = b'00101100'	; 0x2C enter ext. function set: 4 bit mode, 2 lines, 5x7 dots
	CONSTANT  LCD2L_B = b'00101000'	; 0x28 exit ext. function set: 4 bit mode, 2 lines, 5x7 dots

;***** MACROS *****

 kkLCD_Text macro LineN, TXMessage
    LOCAL Message
    LOCAL START
    LOCAL EXIT
    LOCAL I=0
       goto START
Message dt TXMessage
       dt 0
START
    IF LineN==1
        LCDcmd LCDL1
    ELSE
    IF LineN==2
        LCDcmd LCDL2
    ELSE
    IF LineN==3
        LCDcmd LCDL3
    ELSE
    IF LineN==4
        LCDcmd LCDL4
    ENDIF
    ENDIF
    ENDIF
    ENDIF
    WHILE I<20
         call (Message+I)
         addlw 0
         bz EXIT
         call LCDdata
         I=I+1 
    ENDW
    EXIT
    endm


LCDw	macro			; write content of w to LCD
	call	LCDdata
	endm

LCDcmd	macro	LCDcommand	; write command to LCD
	movlw	LCDcommand
	call	LCDcomd
	endm

LCD_DDAdr macro	DDRamAddress
	Local	value = DDRamAddress | b'10000000'	; mask command
	IF (DDRamAddress > 0x67)
		ERROR "Wrong DD-RAM-Address specified in LCD_DDAdr"
	ELSE
		movlw	value
		call	LCDcomd
	ENDIF
	endm

LCD_CGAdr macro	CGRamAddress
	Local	value = CGRamAddress | b'01000000'	; mask command
	IF (CGRamAddress > b'00111111')
		ERROR "Wrong CG-RAM-Address specified in LCD_CGAdr"
	ELSE
		movlw	value
		call	LCDcomd
	ENDIF
	endm

clrLCDport macro		; clear/reset LCD data lines
	movlw	b'11110000'	; get mask
	andwf	LCDport,f	; clear data lines only
	endm

WAIT	macro	timeconst_1
	IF (timeconst_1 != 0)
	    movlw	timeconst_1
	    call delay_ms
	ENDIF
	endm

;***** MEMORY STRUCTURE *****

	ORG     0x00			; processor reset vector

    nop
    goto HardStart

 	ORG     0x04			; interrupt vector location

   ; Save context
    movwf bIntSaveW0
    swapf STATUS,W
    BANK0
    movwf bIntSaveSt
    movfw FSR
    movwf bIntSaveFSR
    movf PCLATH,W
    movwf bIntSavePCLATH
    clrf PCLATH

    ; SPI interrupt
    btfsc _SSPIF
    goto IntSPI

    ; RB0 interrupt by INT CAN
    btfsc INTCON,INTF
    goto MCP2515_ISR

    ; Timer0 overflow interrupt flag
    jmpSet INTCON, TMR0IF, jIntTimer0

    ; Timer1 overflow interrupt flag
    jmpSet _TMR1IF, jIntTimer1

IntReturn
    BANK0
    movf bIntSavePCLATH,W
    movwf PCLATH
    movf bIntSaveFSR,W
    movwf FSR
    swapf bIntSaveSt,W
    movwf STATUS
    swapf bIntSaveW0,F
    swapf bIntSaveW0,W
    retfie

#include "can_lib.asm"

MCP2515_ISR
    movfw PORTB
    bcf INTCON,INTF
    movlw 'E'
    call LCDdata
    goto IntReturn

;*********************************************
;IntSPI
IntSPI
    bcf _SSPIF

    ; transfer received byte to the next location in the buffer

    movf pSPIBuf, W
    movwf FSR
    incf pSPIBuf, F

    movfw SSPBUF
    movwf INDF

    decfsz bSPICnt,F
    goto jIntSPI2

    ; last transaction complete
    bsf tp2510_CS_
    goto IntReturn

jIntSPI2
    incf   FSR, F
    movfw INDF
    movwf SSPBUF
    goto IntReturn

;***********************************************

jIntTimer0
     bcf INTCON, TMR0IF
     goto IntReturn

;********************************************

jIntTimer1
     bcf _TMR1IF
     incf iTimer1
     goto IntReturn

;**************************************************

;jKey_ISR
;     bcf INTCON,RBIF
;     clrf cdata
;     clrf PORTB
;     movlw 0x10
;     movwf cbit
;keyLoop
;     movfw cbit
;     movwf PORTB
;     movfw PORTB
;     andlw 0x20
;     btfss _Z
;     movfw cbit
;     iorwf cdata,F
;     bcf _C
;     rrf cbit,F
;     btfss _C
;     goto keyLoop
  ;   goto IntReturn
;     return

;************** MAIN **************

HardStart
    call Init
    bsf tp2510_CS_
                    ;       bit 0 = 0 ->CAN 11 bits
    movlw 2         ;       bit 0 = 1 ->CAN 29 bits
    movwf CanType   ;       bit 1 = 0 ->250Khz
                    ;       bit 1 = 1 ->500Khz
    call LCDinit

    movlw LCDL2+2
    call LCDcomd
    movlw 9
    call LCDText
;    LCD_Text 1," Diagnostics"

    movlw 8
    movwf SaveReg
p1  movlw 0xff
    call delay_ms
    decfsz SaveReg
    goto p1

    movlw LCDCLR
    call LCDcomd

    movlw LCDL1+5
    call LCDcomd
    movlw 0
    call LCDText 
;    LCD_Text 1,"      O B D II"

    call InitSPIPort

         ; Wait 28ms for MCP2515 to initialize
    movlw 28
    call delay_ms

    movlw LCDL3 
    call LCDcomd

;   call Init2510

    call Init1

;     call jSendFrame

 ;    SPI_Read TXB0CTRL
 ;    andlw 0x40
 ;    btfss _Z
 ;    goto jErr
 ;    kkLCD_Text LCDL4,"Success"
 ;    goto jMainLoop
;jErr
;     kkLCD_Text LCDL4,"Error"

jMainLoop
     bcf _C
     bcf _Z
     call ParseKey
     call CheckCANMsg
     addlw 0
     btfsc _Z
     goto jMainLoop
     ; printf msg id

    movlw LCDL3
    call LCDcomd
    movlw 3
    call LCDText 
;      LCD_Text 3, "Msg ID "

    btfss CanType,0
    goto jRecStd

    movfw iRecEX_L
    call LCD_Hex
    movfw iRecEX_H
    call LCD_Hex
jRecStd
    movfw iRecID_H
    call LCD_Hex
    movfw iRecID_L
    call LCD_Hex

    goto jMainLoop

;*************************************

ParseKey
    rrf PORTB,W
    xorlw 0x7f
    btfsc _Z
    return 
    movwf SaveReg
waitK

    rrf PORTB,W
    xorlw 0xff
    btfss _Z
    iorwf SaveReg
    btfss _Z
    goto waitK      ; wait for key depress
    movfw SaveReg
    sublw 0x14
    btfsc _Z
    goto CfgFunc 
    btfss config_mode
    return
    btfsc SaveReg,0
    goto Key_Left
    btfsc SaveReg,1
    goto Key_Up
    btfsc SaveReg,2
    goto Key_Right
    btfsc SaveReg,3
    goto Key_Down
    btfsc SaveReg,4
    goto Key_Enter
    return

ParseKey1
    rrf PORTB,W
    xorlw 0x7f
    btfsc _Z
    return 
    movwf SaveReg
waitK1
    rrf PORTB,W
    xorlw 0xff
    btfss _Z
    goto waitK      ; wait for key depress

    btfsc SaveReg,0
    goto Key_Left
    btfsc SaveReg,1
    goto Key_Up
    btfsc SaveReg,2
    goto Key_Right
    btfsc SaveReg,3
    goto Key_Down
    btfsc SaveReg,4
    goto Key_Enter
    return

;******************************************************

;ParseError
;     SPI_Read CANINTF
;     andlw 0xA0
;     btfsc _Z
;     return

;      movlw LCDL4
;      call LCDcomd
;      movlw 3
;      call OutText  ; CAN Error
;      SPI_Read EFLG
;      call LCDval08

    ; Clear interrupt flag
;     bL2bV 0xA0, b2510RegMask
;     bL2bV 0x00, b2510RegData
;     movlw CANINTF
;     call BitMod2510

;     bL2bV 0xC0, b2510RegMask
;     bL2bV 0x00, b2510RegData
;     movlw EFLG
;     call BitMod2510
 ;    return
;****************************************


;TxCANMessage
;     ; Wait for pending message to be sent
;     bL2bV   0x08, b2510RegMask
;     movlw TXB0CTRL
;     call WaitANDeqZ

     ; Send CAN Standard frame

;     SPI_WriteL TXB0SIDH, 0x07  ; message ID
;     SPI_WriteL TXB0SIDL, 0xdf     ; Send message - lower bits 0
;     SPI_WriteL TXB0DLC, 0x08      ; n data bytes

    ; Send least significante byte first
;     SPI_WriteL TXB0D0, 0x01  ; SID $01
;     SPI_WriteL TXB0D1, 0x00  ; PID $00
;     SPI_Rts RTS0                  ; Transmit buffer 0

;      bsf T1CON,TMR1ON     ; Start TMR1
;     return

;*******************************************************

ParseCAN
     SPI_Read CANINTF
     movlw 2
     call LCDcomd
     movlw 3
     call LCDText
;      LCD_Text 2, "CAN OK"

      movfw iRecID_H
      movwf HI
      movfw iRecID_L
      movwf LO
      call LCDval16
      bcf tbRxMsgPend
   
     ; Clear interrupt flag
     bL2bV 0x03, b2510RegMask 
     bL2bV 0x00, b2510RegData
     movlw CANINTF
     call BitMod2510 
     return

;***************ID TABLE******************************

    ; Functional addr 11 bits 0x07DF
    ; Functional addr 29 bits 0x18DB33F1

;*********************************************
; Inicio de sessão
;   SID $01   Request current powertrain diagnostic data
;   PID $00   Suported PIDs

;   0x07 0x01 0x00 0x0 0x0 0x0 0x0 0x0

Init
     clrwdt
     BANK1
     clrf PIE1_P

     movlw b'00000110'
     movwf ADCON1        ; Set PORTA Digital I/O

  ; TRM0 config 
     movlw b'00000101'       ; configure Timer0
           ; 0-------       PORTB pull-up resistor enable 
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

;     bsf INTCON, INTE
     bsf INTCON,PEIE
     bsf INTCON,GIE   ; 11000000

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
     ;       0 in MCP2515 INT
     ;       1 in key left
     ;       2 in key enter
     ;       3 in key right
     ;       4 in key down
     ;       5 in key up
     ;       6 -
     ;       7 -
     movlw b'11111111'
     movwf TRISB

     ; PORTC
     ;      0
     ;      1
     ;      2   out CS
     ;      3   out SPI clock - master
     ;      4   in SPI data - SO
     ;      5   out SPI data out - SI
     ;      6
     ;      7
     movlw b'11010010'
     movwf TRISC
     BANK0
     return

;**************************************************

WaitMSec
     movwf bCnt
jWaitMSec0
     clrwdt
     Set1HClock bGenClk,4
jWaitMSec1
     jmp1HNotYet bGenClk,jWaitMSec1
     decfsz bCnt,F
     goto jWaitMSec0
     return

;****************************************************

    org 0x300

GetSRegAddr:
    addwf PCL,F
    retlw reg0-reg0
    retlw reg1-reg0
    retlw reg2-reg0
    retlw reg3-reg0
    retlw reg4-reg0
    retlw reg5-reg0
    retlw reg6-reg0
    retlw reg7-reg0
    retlw reg8-reg0
    retlw reg9-reg0
    retlw reg10-reg0
    retlw reg11-reg0
    retlw reg12-reg0
    retlw reg13-reg0
    retlw reg14-reg0
    retlw reg15-reg0
    retlw reg16-reg0
    retlw reg17-reg0
 ;  retlw reg18-reg0
 ;  retlw reg19-reg0

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
Str0: dt " O B D II",0x00
Str1: dt "Not compatible",0x00
Str2: dt "CAN    bits/    Kbps",0x00
Str3: dt "Msg ID ",0x00
Str4: dt " Normal mode",0x00
Str5: dt " Sleep mode",0x00
Str6: dt "LoopBack mode",0x00
Str7: dt " Listen mode",0x00
Str8: dt " Config mode",0x00
Str9: dt "  Diagnostics   ",0x00

   org 0x400

RegNames
    addwf PCL, F
reg0:  dt "CANSTAT    0x",0x00
reg1:  dt "CANCTRL    0x",0x00
;reg2:  dt "TEC        0x",0x00
;reg3:  dt "REC        0x",0x00
reg2:  dt "CNF3       0x",0x00
reg3:  dt "CNF2       0x",0x00
reg4:  dt "CNF1       0x",0x00
reg5:  dt "CANINTE    0x",0x00
reg6:  dt "CANINTF    0x",0x00
reg7:  dt "EFLG       0x",0x00
reg8: dt "TXB0CTRL   0x",0x00
reg9: dt "TXB0SIDH   0x",0x00
reg10: dt "TXB0SIDL   0x",0x00
reg11: dt "RXB0CTRL   0x",0x00
reg12: dt "RXB0SIDH   0x",0x00
reg13: dt "RXB0SIDL   0x",0x00
reg14: dt "RXB0EID8   0x",0x00
reg15: dt "RXB0EID0   0x",0x00
reg16: dt "TXB0DLC    0x",0x00
reg17: dt "TXB0DM     0x",0x00

;******************************************

jSendFrame
      ; Wait for pending message to be sent
     bL2bV   0x08, b2510RegMask
     movlw TXB0CTRL
     call WaitANDeqZ

    SPI_WriteL TXB0CTRL, 0x0B 
    SPI_Rts RTS0       ; Transmit buffer 0
    return

;********************************************

;     SyncSeg = 1TQ
;     PropSeg = 1->8TQ
;     PS1     = 1->8TQ
;     PS2     = 2->8TQ

Init1
   call Reset2510
  ; wait 128 cicles
    movlw 0xff
    call delay_ms

    btfss baudrate_flag
    goto jlow 
    ; 500Khz
    SPI_WriteL CNF1, 0x41
    SPI_WriteL CNF2, 0xBA
    SPI_WriteL CNF3, 0x06
    goto brdone
jlow
    ; 250Khz
    SPI_WriteL CNF1, 0x42
    SPI_WriteL CNF2, 0xBA
    SPI_WriteL CNF3, 0x06
brdone
    SPI_WriteL CANINTE,0x00
    SPI_WriteL CANINTF,0x00
    SPI_WriteL TXB0CTRL,0x00
    SPI_WriteL TXB1CTRL,0x00
    SPI_WriteL TXB2CTRL,0x00
    SPI_WriteL RXB0CTRL,0x60 ;  turn Mask/Filter OFF | Roll Over ON

    btfss frametype_flag
    call CfgSTDMask
    btfsc frametype_flag
    call CfgExMask

    bL2bV 0x1F, b2510RegMask
    bL2bV 0x00, b2510RegData
    movlw CANCTRL
    call BitMod2510

;    movlw 0x00      ; NORMAL_MODE
 ;   call Set2510Mode
    call SetLoopBackMode

jCANFrameData
     ; Set to DataFrame|SzFrame
     SPI_WriteL TXB0DLC, 0x02    ; data frame | n data bytes

    ; Send least significante byte first
     SPI_WriteL TXB0D0, 0x01  ; SID $01
     SPI_WriteL TXB0D1, 0x00  ; PID $00
    return

;**************************************************

CfgFunc
    ; SPI_WriteL CANCTRL,0x98  ;set config mode
    ; abort all pending transmitions
;     movlw 0x80
;     call Set2510Mode
     movlw LCDL1+3
     call LCDcomd

     SPI_Read CANSTAT
     movwf cdata
     swapf cdata
     bcf _C
     rrf  cdata,F
     movfw cdata
     addlw 4
     call LCDText
     call Refresh
     bsf config_mode
     return

;*********************************************************



Refresh
     movlw LCDL3
     call LCDcomd
     movfw cIdx
     call LCD_RegString
     movlw high getRegAddr
     movwf PCLATH
     movfw cIdx
     call getRegAddr
     call Rd2510Reg
     movwf cdata
     call LCD_Hex
     return

getRegAddr
     addwf PCL,F
     retlw 0x0E   ; CANSTAT
     retlw 0x0F   ; CANCTRL
;     retlw 0x1C   ; TEC
;     retlw 0x1D   ;REC
     retlw 0x28   ;CNF3
     retlw 0x29   ; CNF2
     retlw 0x2A   ; CNF1
     retlw 0x2B   ;CANINTE
     retlw 0x2C   ; CANINTF
     retlw 0x2D  ; EFLG
     retlw 0x30  ; TXB0CTRL
     retlw 0x31  ; TXB0SIDH
     retlw 0x32  ; TXB0SIDL
     retlw 0x60  ; RXB0CTRL
     retlw 0x61  ; RXB0SIDH
     retlw 0x62  ; RXB0SIDL
     retlw 0x63  ; RXB0EID8
     retlw 0x64  ; RXB0EID0
     retlw 0x35  ; TXD0DLC
     retlw 0x36  ; TXB0DM

;******************************************************************************

Key_Left
     btfss data_nibble
     goto AdjRew
     bcf data_nibble
     movlw LCDL3+13
     call LCDcomd
     return
AdjRew
     btfss menu_stage
     goto normal_func
     bcf menu_stage
     movlw 0x0c
     call LCDcomd
     return
normal_func
     movlw LCDCLR
     call LCDcomd
     movlw LCDL1+5
     call LCDcomd
    ; set normal mode
;     movlw 0x00
;      call Set2510Mode
     movlw 0
     call LCDText 
;     LCD_Text 1,"      O B D II"
     bcf config_mode
    ; set default
     bcf CanType, 0   ; default 11 bits
     bsf CanType,1    ; default 500Khz
     goto jSendFrame
     return

Key_Right
     btfsc menu_stage
     goto AdjFw
     bsf  menu_stage
     movlw LCDL3+13
     call LCDcomd
     movlw 0x0f
     call LCDcomd
     return
AdjFw
     btfsc data_nibble
     return
     bsf data_nibble
     movlw LCDL3+14
     call LCDcomd
     return

Key_Down
     btfsc menu_stage
     goto wrmode3
     movf cIdx,W
     btfsc _Z
     return
     decf cIdx
     goto Refresh
wrmode3
     movlw 0x01
     btfss data_nibble
     movlw 0x10
     subwf cdata
     movfw cdata
     btfss data_nibble
     swapf cdata,W
     andlw 0x0f
     call getCH
     call LCDdata
     movlw LCDL3+13
     btfsc data_nibble
     movlw LCDL3+14
     call LCDcomd
     return

Key_Up
     btfsc menu_stage
     goto wrmode4
     movfw  cIdx
     sublw 16
     btfss _C
     return
     incf cIdx
     goto Refresh
wrmode4
     movlw 0x01
     btfss data_nibble
     movlw 0x10
     addwf cdata 
     movfw cdata
     btfss data_nibble
     swapf cdata,W
     andlw 0x0f
     call getCH
     call LCDdata
     movlw LCDL3+13
     btfsc data_nibble
     movlw LCDL3+14
     call LCDcomd
     return

Key_Enter
     btfss menu_stage
     return
     movlw high getRegAddr
     movwf PCLATH
     movfw cdata
     movwf b2510RegData
     movfw cIdx
     call getRegAddr
     movwf b2510RegAdr
     movlw high Wrt2510Reg
     movwf PCLATH
     movfw b2510RegAdr
     call Wrt2510Reg
     movlw LCDCONT
     call LCDcomd
     bcf data_nibble
     bcf menu_stage
     return

;*******************************************************

CfgSTDMask

    SPI_WriteL RXM0SIDH, 0xFF
    SPI_WriteL RXM0SIDL, 0xE0

    SPI_WriteL RXF0SIDH, 0xFF
    SPI_WriteL RXF0SIDL, 0xFF

    SPI_WriteL TXB0SIDH, 0xFB  ; message ID
    SPI_WriteL TXB0SIDL, 0xE0
    return

;***********************************************

CfgExMask

    SPI_WriteL RXF0EID8, 0x1b
    SPI_WriteL RXF0EID0, 0x66

    SPI_WriteL RXF0SIDH, 0x7e
    SPI_WriteL RXF0SIDL, 0x2b

    SPI_WriteL RXM0EID8, 0xFF
    SPI_WriteL RXM0EID0, 0xFF

    SPI_WriteL RXM0SIDH, 0xFF
    SPI_WriteL RXM0SIDL, 0xFF

     SPI_WriteL TXB0SIDL, 0x2B  ; message ID
     SPI_WriteL TXB0SIDH, 0x7E  ; SID|ExFrame|ExID
     SPI_WriteL TXB0EID0, 0x66  ; message ExID
     SPI_WriteL TXB0EID8, 0x1B

    return

;***********************************************

;Init2510 

;   call Reset2510
    ; ---0----     abort all pending msg
    ; ----1---     one shot tx
    ; -----0--     disable ClkOut

  ; wait 128 cicles
;    movlw 0xff
;    call delay_ms

;    bL2bV 0x1C, b2510RegMask
;    bL2bV 0x08, b2510RegData
;    movlw CANCTRL
;    call BitMod2510

    ; set MCP2515 interrupts
    ;  0-------    Error on TX/RX message
    ;  -0------    CAN bus activity
    ;  --0-----    Interrupt on EFLAG reg
    ;  ---0----    TXB2 empty
    ;  ------1-    Messsage received on RXB1
    ;  -------1    Messsage received on RXB0

;   SPI_WriteL CANINTE,0x00

;    btfss CanType,0
;    call  CfgSTDMask
;    btfsc CanType,0
;    call  CfgExMask

    ; x00-----  receive all valid message using filter criteria
    ; x----000  accept filter 0
 ;;   SPI_Write RXB1CTRL, 0x00

;    btfss CanType,1
;    goto LowBRP

    ; Set physical layer configuration
    ;  16TQ
    ; Fosc       = 16Mhz : 500Kbits/s
    ; BRP        = 1  
    ; Sync Seg   = 1TQ
    ; Prop Seg   = 2TQ
    ; Phase Seg1 = 10TQ
    ; Phase Seg2 = 3TQ

    ; TQ = 2 * (1/Fosc) * (BRP+1)

    ; Hi_BRP 500Kb/s
    ;  BRP=1
    ; Sync Seg = 1TQ 0x00
;    SPI_WriteL CNF1, 0x01   ;   set BRP
    ; BTLMODE_CNF3   0x80
    ; SMPL_1X        0x40
    ; PHSEG1 = 10QT  0x10
    ; PropSeg = 2QT  0x0A
;    SPI_WriteL CNF2, 0xDA
    ; SOF = 0 - WAKFIL = 0 - PHSEG2 = 2QT
;    SPI_WriteL CNF3, 0x03

;    goto CfgBRPDone
;LowBRP
    ; Low_BRP 250Kb/s

    ;  16TQ
    ; Fosc       = 16Mhz : 500Kbits/s
    ; SJW=0 - BRP=2
    ; Sync Seg   = 1TQ
    ; Prop Seg   = 2TQ
    ; Phase Seg1 = 10TQ
    ; Phase Seg2 = 3TQ
;   SPI_WriteL CNF1, 0x01
;    SPI_WriteL CNF2, 0xF2
;    SPI_WriteL CNF3, 0x03

    ; Config RX registers

    ; x00x----  RXM   turn on mask/filter
    ; x--x-1--  BUKT  RXB0 msg roll over

;    SPI_WriteL RXB0CTRL,  0x64

;     bL2bV 0x0B, b2510RegMask
;     bL2bV 0x0B, b2510RegData
;     movlw TXB0CTRL
;     call BitMod2510

    ;    125Kb/s
    ; BaundRate=Fxtal/(2*(BPR+1)*(3+TSEG1+TSEG2))

;CfgBRPDone

;       call SetNormalMode
  ;    call SetLoopBackMode
   ;  call SetListenMode
;       movlw 0x80
;      call delay_ms

  ; InitializeOBD2 by detecting Baud rate

     ; Wait for pending message to be sent
;jSendFrame
;     bL2bV   0x08, b2510RegMask
;     movlw TXB0CTRL
;     call WaitANDeqZ

     ; Send CAN frame

;      btfsc CanType,0
;     goto jExCANFrame

     ; Send CAN Standard frame
     ; Functional addr   0x7DF
     ; Physical request ECU1 addr  0x7E0
     ; Physical respond ECU1 addr  0x7E8

;     SPI_WriteL TXB0SIDH, 0xFB  ; message ID
;     SPI_WriteL TXB0SIDL, 0xE0
;     goto jCANFrameData

;jExCANFrame

     ; Send CAN Extended frame 
     ; Functional addr 0x 18 DB 33 F1
     ; Physical request addr 0x 18 DA xx F1 ( ECU#xx )
     ; Physical respond addr 0x 18 DA F1 xx ( ECU#xx )

;     SPI_WriteL TXB0SIDL, 0x2B  ; message ID
;     SPI_WriteL TXB0SIDH, 0x7E  ; SID|ExFrame|ExID
;     SPI_WriteL TXB0EID0, 0x66  ; message ExID
;     SPI_WriteL TXB0EID8, 0x1B
;jCANFrameData
     ; Set to RemoteFrame|SzFrame
;     SPI_WriteL TXB0DLC, 0x42      ; n data bytes

    ; Send least significante byte first
;     SPI_WriteL TXB0D0, 0x01  ; SID $01
;     SPI_WriteL TXB0D1, 0x00  ; PID $00

;    SPI_Rts RTS0       ; Transmit buffer 0

 ;     Set1HClock bGenClk,200   ; 0xc8 Wait 56ms for response
 ;     bsf T1CON,TMR1ON 

;jWaitAnswer
;      SPI_Read TXB0CTRL
;      andlw 0x70
;      btfss _Z
;      goto tx_err
;      movfw TMR1L
;      subwf bGenClk,W
;  	  btfsc   _C
;	  goto   jWaitAnswer
;      goto pass2
;tx_err
;    bsf ertx_flag
;    bL2bV 0x08,b2510RegMask
;    bL2bV 0x00, b2510RegData
;    movlw TXB0CTRL
;    call BitMod2510
;pass2
;      clrf TMR1H
;      clrf TMR1L
;      bcf T1CON,TMR1ON

;     SPI_Read CANINTF
;     movwf SaveReg   ; save reg
;     andlw 0x80   ; see  MERR|ERRIF
     
;     SPI_WriteL CANINTF,0x00
   
;     btfsc ertx_flag
;     goto  jCfgBRP
     ; No errors; Check Msg
;     movfw SaveReg
;     andlw 0x03
;     btfss _Z
;     retlw 2   ; return CAN msg received
;     movfw CanType
;     andlw 0x01
;     btfss _Z
;     retlw 0x01      ; return no compatible
;     movlw 0x01
;     iorwf CanType
;     goto jSendFrame

;jCfgBRP
;    btfss CanType,1
;    retlw  0x01   ; não compativel

    ; Clear error flag on CANINTF
;    bL2bV 0xA0,b2510RegMask
;    bL2bV 0x00, b2510RegData
;    movlw CANINTF
;    call BitMod2510

;    call SetConfigMode
;    movlw 0x80
;    call delay_ms
;    goto LowBRP

;******************************************

LCD_RegString

    movwf STR_NUM
   movlw high GetSRegAddr
   movwf PCLATH
   movf STR_NUM,W
   call GetSRegAddr
   movwf INDEX
next
   movlw high RegNames
   movwf PCLATH

    movf INDEX,W
    call RegNames
    addlw d'0'
    btfsc STATUS,Z
    goto done
    call LCDdata
    incf INDEX, F
    goto next
done
    return

;*****************************************************

LCDText
   movwf STR_NUM
   movlw high GetStrAddr
   movwf PCLATH
   movf STR_NUM,W
   call GetStrAddr
   movwf INDEX
   movlw high Strings
   movwf PCLATH
next1
    movf INDEX,W
    call Strings
    addlw d'0'
    btfsc STATUS,Z
    goto done1

    call LCDdata
    incf INDEX, F

    goto next1
done1
    return

;*****************************************************

LCD_Hex
    movwf LO
    movwf LO_TEMP
    bcf BCflag
    movlw 0x10
    movwf TEMP2
    call _Hex08
    call LCDdata
    movlw 1
    movwf TEMP2
    bsf BCflag
    call _Hex08
    call LCDdata
    return

getCH
     addwf PCL,F
     retlw '0'
     retlw '1'
     retlw '2'
     retlw '3'
     retlw '4'
     retlw '5'
     retlw '6'
     retlw '7'
     retlw '8'
     retlw '9'
     retlw 'A'
     retlw 'B'
     retlw 'C'
     retlw 'D'
     retlw 'E'
     retlw 'F'

_Hex08
     clrf TEMP1
     movfw TEMP2
_V08_H
     subwf LO_TEMP,W
     skpc
     goto _VH8_LCD
     incf TEMP1,F
     movfw TEMP2
     subwf LO_TEMP,F
     bsf BCflag
     goto _V08_H
_VH8_LCD

     movlw high getCH
     movwf PCLATH
     movfw TEMP1
     call getCH
     return

;*****************************************************

LCDval08

	movwf	LO
	movwf	LO_TEMP		; LO -> LO_TEMP
	bcf	BCflag		; blank checker for preceeding zeros

	movlw	d'100'		; check amount of 100s
	movwf	TEMP2		; ==> Decimal Range 0 - 255 <=> 8 bit
	call	_VALcnv08	; call conversion sub-routine
	LCDw			; call LCD sub-routine with value stored in w

	movlw	d'10'		; check amount of 10s
	movwf	TEMP2
	call	_VALcnv08	; call conversion sub-routine
	LCDw			; call LCD sub-routine with value stored in w

	movlw	d'1'		; check amount of 1s
	movwf	TEMP2
	bsf	BCflag		; remove blank checker in case of zero
	call	_VALcnv08	; call conversion sub-routine
	LCDw			; call LCD sub-routine with value stored in w

	RETURN

;*****************************************************

LCDval16
 
	movfw	LO		; LO -> LO_TEMP
	movwf	LO_TEMP
	movfw	HI		; HI -> HI_TEMP
	movwf	HI_TEMP
	bcf	BCflag		; Blank checker for preceeding zeros

	movlw	b'00010000'	; check amount of 10000s
	movwf	TEMP2		; Sub-LO
	movlw	b'00100111'
	movwf	TEMP3		; Sub-HI
	call	_VALcnv16	; call conversion sub-routine
	LCDw			; call LCD sub-routine with value stored in w

	movlw	b'11101000'	; check amount of 1000s
	movwf	TEMP2		; Sub-LO
	movlw	b'00000011'
	movwf	TEMP3		; Sub-HI
	call	_VALcnv16	; call conversion sub-routine
	LCDw			; call LCD sub-routine with value stored in w

	movlw	b'01100100'	; check amount of 100s
	movwf	TEMP2		; Sub-LO
	clrf	TEMP3		; Sub-HI is zero
	call	_VALcnv16	; call conversion sub-routine
	LCDw			; call LCD sub-routine with value stored in w

	movlw	b'00001010'	; check amount of 10s
	movwf	TEMP2		; Sub-LO
	clrf	TEMP3		; Sub-HI is zero
	call	_VALcnv16	; call conversion sub-routine
	LCDw			; call LCD sub-routine with value stored in w

	movlw	b'00000001'	; check amount of 1s
	movwf	TEMP2		; Sub-LO
	clrf	TEMP3		; Sub-HI is zero
	bsf	BCflag		; remove blank checker in case of zero
	call	_VALcnv16	; call conversion sub-routine
	LCDw			; call LCD sub-routine with value stored in w

	RETURN

;*****************************************************

LCDval32

   bcf BCflag
   movlw 0x80
   movwf TEMP2
   movlw 0x96
   movwf TEMP3
   movlw 0x98
   movwf TEMP4
   clrf TEMP5
   call _Valcnv32
   call LCDdata
   bsf BCflag

   movlw 0x40
   movwf TEMP2
   movlw 0x42
   movwf TEMP3
   movlw 0x0f
   movwf TEMP4
   clrf TEMP5
   call _Valcnv32
   call LCDdata

   movlw ','
   call LCDdata

   movlw 0xa0
   movwf TEMP2
   movlw 0x86
   movwf TEMP3
   movlw 0x01
   movwf TEMP4
   clrf TEMP5
   call _Valcnv32
   call LCDdata

   movlw 0x10
   movwf TEMP2
   movlw 0x27
   movwf TEMP3
   clrf TEMP4
   clrf TEMP5
   call _Valcnv32
   call LCDdata

 ;  movlw 0xe8
 ;  movwf TEMP2
 ;  movlw 0x03
 ;  movwf TEMP3
 ;  clrf TEMP4
 ;  clrf TEMP5
 ;  call _Valcnv32
 ;  call LCDdata

 ;  movlw 0x64
 ;  movwf TEMP2
 ;  clrf TEMP3
 ;  clrf TEMP4
 ;  clrf TEMP5
 ;  call _Valcnv32
 ;  call LCDdata

 ;  movlw 0x0a
 ;  movwf TEMP2
 ;  clrf TEMP3
 ;  clrf TEMP4
 ;  clrf TEMP5
 ;  call _Valcnv32
 ;  call LCDdata

 ;  movlw 0x01
 ;  movwf TEMP2
 ;  clrf TEMP3
 ;  clrf TEMP4
 ;  clrf TEMP5
 ;  call _Valcnv32
 ;  call LCDdata

   return

;************************************************************************

_VALcnv08
	clrf	TEMP1		; counter
	movfw	TEMP2		; decrement-value
_V08_1	subwf	LO_TEMP,W	; TEST: LO_TEMP-TEMP2 >= 0 ?
	skpc    	; btfss STATUS,C skip, if true
	goto	_V08_LCD	; result negativ, exit
	incf	TEMP1,F		; count
	movfw	TEMP2		; decrement-value
	subwf	LO_TEMP,F	; STORE: LO_TEMP = LO_TEMP - TEMP2
	bsf	BCflag		; invalidate flag
	goto	_V08_1		; repeat
_V08_LCD
	movlw	'0'		; writes Number to LCD
	addwf	TEMP1,W		; '0' is ascii offset, add counter
	btfss	BCflag		; check flag
	movlw ' '		; clear preceeding zeros
	; return with data in w
	RETURN

;*****************************************************

_VALcnv16
	clrf	TEMP1		; clear counter
_V16_1	movfw	TEMP3
	subwf	HI_TEMP,w	; TEST: HI_TEMP-TEMP3 >= 0 ?
	skpc			; skip, if true
	goto	_V16_LCD	; result negativ, exit
	bnz	_V16_2		; test zero, jump if result > 0
	movfw	TEMP2		; Precondition: HI-TEST is zero
	subwf	LO_TEMP,w	; TEST: LO_TEMP-TEMP2 >= 0 ?
	skpc			; skip, if true
	goto	_V16_LCD	; result negativ, exit
_V16_2
	movfw	TEMP3
	subwf	HI_TEMP,f	; STORE: HI_TEMP = HI_TEMP - TEMP3
	movfw	TEMP2
	subwf	LO_TEMP,f	; STORE: LO_TEMP = LO_TEMP - TEMP2
	skpc			; skip, if true
	decf	HI_TEMP,f	; decrement HI
	incf	TEMP1,f		; increment counter
	bsf	BCflag		; invalidate flag
	goto	_V16_1
_V16_LCD
	movlw	'0'		; writes number to LCD
	addwf	TEMP1,w		; '0' is ascii offset, add counter
	btfss	BCflag		; check flag
	movlw	' '		; clear preceeding zeros
	; return with data in w
	RETURN

;********************************************************************

_Valcnv32
   clrf TEMP1
_V32_1
   movfw TEMP5
   subwf HI_TEMP,w
   skpc
   goto _V32_LCD
   bnz _V32_2

   movfw TEMP4
   subwf LO_TEMP,w
   skpc
   goto _V32_LCD
   bnz _V32_2

   movfw TEMP3
   subwf HI,w
   skpc
   goto _V32_LCD
   bnz _V32_2

   movfw TEMP2
   subwf LO,w
   skpc
   goto _V32_LCD
_V32_2
   movfw TEMP5
   subwf HI_TEMP,f
   movfw TEMP4
   subwf LO_TEMP,f
   skpc
   decf HI_TEMP,f
   movfw TEMP3
   subwf HI,f
   skpc
   decf LO_TEMP,f
   movfw TEMP2
   subwf LO,f
   skpc     ; skeep if carry
   decf HI,f
   incf TEMP1,f
   bsf BCflag
   goto _V32_1
_V32_LCD
   movlw '0'
   addwf TEMP1,w
   btfss BCflag
   movlw ' '
   return

;***** SUBROUTINES *****

	; transmit only lower nibble of w

LCDxmit	movwf	LCDbuf		; store command/data nibble
	; first, clear LCD data lines
	clrLCDport
	; second, move data out to LCD data lines
	movf	LCDbuf,w	; get data
	andlw	b'00001111'	; extract only valid part
	iorwf	LCDport,f	; put to LCD data lines
	RETURN

;*****************************************************

	; clocks LCD data/command
LCDclk	;WAIT	LCDWAIT
    movlw LCDWAIT
    call delay_ms
    bsf     LCD_EN		; set LCD enable
	; insert LCDSPEED x nops to comply with manufacturer
	; specifications for clock rates above 9 MHz
	VARIABLE CNT_V		; declare intermediate variable
CNT_V = LCDSPEED	; assign pre-defined constant
	WHILE (CNT_V > 0x0)	; perform while loop to insert 'nops'
	  nop			; insert 'nop'
CNT_V -= 1		; decrement
	ENDW
	bcf     LCD_EN
	WAIT	LCDWAIT		; clocks LCD data/command
	RETURN

;*****************************************************

	; transmit command to LCD
LCDcomd
    bcf	LCD_RS		; select command registers
	goto	_LCD_wr

	; transmit data to LCD
LCDdata	bsf	LCD_RS		; select data registers
_LCD_wr ;	bcf	LCD_RW		; set write direction
	movwf	LCDtemp		; store command/data to send
	; send hi-nibble
;	movfw	LCDtemp		; get data
	swapf	LCDtemp,w	; swap hi- and lo-nibble, store in w
	call	LCDxmit		; transmit nibble
	call	LCDclk
	; send lo-nibble
	movfw	LCDtemp		; get data
	call	LCDxmit		; transmit nibble
	call	LCDclk
	; reset LCD controls
	clrLCDport		; reset LCD data lines
	bcf	LCD_RS		; reset command/data register
	RETURN

;*****************************************************

LCDinit
	bcf	LCD_EN		; clear LCD clock line
	bcf	LCD_RS		; clear command/data line
	clrLCDport		; reset LCD data lines
	WAIT	4*LCDWAIT	; >= 40 ms 

	; LCD INITIALIZATION STARTS HERE
	; start in 8 bit mode
	movlw	LCDEM8		; send b'0011' on [DB7:DB4]
	call	LCDxmit		; start in 8 bit mode 
	call	LCDclk		; That's while: Wait 39us
	WAIT	LCDWAIT 	; On POWER UP, the LCD will initialize itself,
				; but after a RESET of the microcontroller without
				; POWER OFF, the 8 bit function mode will reboot
				; the LCD to 4 bit mode safely.

	movlw	LCDDZ		; set DDRAM to zero
	call	LCDxmit
	call	LCDclk
	WAIT	LCDWAIT		; ~1 ms @ 4 MHz

	movlw	LCDEM4		; send b'0010' on [DB7:DB4]
	call	LCDxmit		; change to 4 bit mode
	call	LCDclk
	WAIT	LCDWAIT		; ~1 ms @ 4 MHz

	; now in 4 bit mode, sending two nibbles
	IF LCDTYPE == 0x00
	  LCDcmd LCD2L		; function set: 4 bit mode, 2 lines, 5x7 dots
	  LCDcmd LCDCONT	; display control: display on, cursor off, blinking off
	  LCDcmd LCDCLR		; clear display, address counter to zero
	  WAIT LCDCLRWAIT	; wait after LCDCLR until LCD is ready again
	ELSE
	  IF LCDTYPE == 0x01
	    ; for LCD EA DIP204-4 (white chars, blue backlight)
	    LCDcmd LCD2L_A	; switch on extended function set
	     LCDcmd LCDEXT	; 4 lines
	     LCDcmd LCD2L_B	; switch off extended function set
	    LCDcmd LCDCONT	; display control: display on, cursor off, blinking off
	    LCDcmd LCDCLR	; clear display, address counter to zero
	    WAIT LCDCLRWAIT	; wait after LCDCLR until LCD is ready again
	  ELSE
	    ERROR "Unsupported parameter"
	  ENDIF
	ENDIF
    return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

delay_ms            ; W*1ms  at 4MHz
    movwf delay_mult
del_20m             ; 20ms

    movlw  d'200'
    movwf delay_k200
;   clrwdt
del_2u
    nop
    nop
    nop
    decfsz delay_k200,f
    goto del_2u

    decfsz delay_mult,f
    goto del_20m
    return

;**********************************************************************************************

      cblock 0x30

bIntSaveSt
bIntSaveFSR
bIntSavePCLATH

delay_mult
delay_k50
delay_k200

bGenFlags1       ; general control flags 1
bCnt            ; work byte conter

; Timer1
iTimer1:2
bGenClk
bXmitClk

iA2DValue:2
bRegAddr
bRegData
 

iRecID_L         ; ID of receiver message( 3 bits left justified)
iRecID_H        ; ID of receiver message( 8 bits left justified)
iRecEX_H
iRecEX_L
bRecCount        ; number of bytes received
pRecDataBase:8

   ;; Low level SPI interface
b2510RegAdr     ; 0x50
b2510RegData
b2510RegMask

   ;; used in interrupt
bSPICnt        ; # bytes remaining to receive
pSPIBuf         ; Pointer into buffer
pSPIBufBase:16        ; 0x55 Base of SPI receive/xmit buffer

CanType        ;  0x65
IntCAN_Err
SaveReg
cdata
cbit
cIdx

 endc

bIntSaveW0 equ 0x7F
bIntSaveW1 equ 0xFF 

     END
