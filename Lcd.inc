 






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

delay_mult	set	BASE+d'0'	; wait cycle counter

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


    #define delay_k50 TEMP4
    #define delay_k200 TEMP5 

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



 LCD_Text macro LineN, TXMessage
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









;*****************************************************


LCD_String
 
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


;*********************************************************************************************



;--------------------------------------------------------------------------

 

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

;***************************************************


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
