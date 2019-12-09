#include "macros16.inc" 

 

;*************************************************




ISO_Write
    movwf SData
    bsf PORTC,0
    movfw baudRate
    call delay_ms
    
local i=0
while i<8
        btfsc SData,i
        bcf PORTC,0
        btfss SData,i
        bsf PORTC,0
        call delay_ms
        i=i+1
endw
    bcf PORTC,0
    call delay_ms
    call delay_ms
    return




;***********************************************




ISO_Read

	movlw	0x08		; Init shift counter
	movwf	count

sbit
	btfsc	PORTB, 6	; Look for start bit
	goto 	sbit		; For Mark
	rrf     baudRate,W		; 
	subwf tmr0,F		; Load and start timer/counter   
	bcf	INTCON, TMR0IF	; Clear tmr0 overflow flag
time1	btfss INTCON, TMR0IF	; Has the timer (bit 2) overflowed?  Skip next line if 1
	goto	time1		; No
	btfsc	PORTB,6	; Start bit still low?
	goto 	sbit		; False start, go back
	movlw	0x30		; real, define N for timer
	movwf	tmr0		; start timer/counter - bit time
	bcf	 INTCON, TMR0IF	; Clear tmr0 overflow flag
time2	btfss INTCON, TMR0IF	; Timer overflow?
	goto	time2		; No
	movlw	0x30		; Yes, define N for timer
	movwf	tmr0		; Start timer/counter
	bcf	INTCON, 2;	; Clear tmr0 overflow flah
	movf	PORTB, w	; Read port A
	movwf	temp		; Store
	rrf	temp, f		; Rotate bit 0 into carry flag
	rrf	SData, f	; Rotate carry into rcvReg bit 7
	decfsz	count, f	; Shifted 8?
	goto	time2		; No
time3	btfss	INTCON, 2	; Timer overflow?
	goto	time3		; No
    

    return


 