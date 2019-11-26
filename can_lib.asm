
#define RXF0SIDH 0x00
#define RXF0SIDL 0x01
#define RXF0EID8 0x02
#define RXF0EID0 0x03
#define RXF1SIDH 0x04
#define RXF1SIDL 0x05
#define RXF1EID8 0x06
#define RXF1EID0 0x07
#define RXF2SIDH 0x08
#define RXF2SIDL 0x09
#define RXF2EID8 0x0A
#define RXF2EID0 0x0B
#define BFPCTRL  0x0C
#define TXRTSCTRL 0x0D
#define CANSTAT  0x0E
#define CANCTRL  0x0F

#define RXF3SIDH 0x10
#define RXF3SIDL 0x11
#define RXF3EID8 0x12
#define RXF3EID0 0x13
#define RXF4SIDH 0x14
#define RXF4SIDL 0x15
#define RXF4EID8 0x16
#define RXF4EID0 0x17
#define RXF5SIDH 0x18
#define RXF5SIDL 0x19
#define RXF5EID8 0x1A
#define RXF5EID0 0x1B
#define TEC      0x1C
#define REC      0x1D
#define CANSTAT1 0x1E
#define CANCTRL1 0x1F

#define RXM0SIDH 0x20
#define RXM0SIDL 0x21
#define RXM0EID8 0x22
#define RXM0EID0 0x23
#define RXM1SIDH 0x24
#define RXM1SIDL 0x25
#define RXM1EID8 0x26
#define RXM1EID0 0x27
#define CNF3     0x28
#define CNF2     0x29
#define CNF1     0x2A
#define CANINTE  0x2B
#define CANINTF  0x2C
#define EFLG     0x2D
#define CANSTAT2 0x2E
#define CANCTRL2 0x2F

#define TXB0CTRL 0x30
#define TXB0SIDH 0x31
#define TXB0SIDL 0x32
#define TXB0EID8 0x33
#define TXB0EID0 0x34
#define TXB0DLC 0x35
#define TXB0D0 0x36
#define TXB0D1 0x37
#define TXB0D2 0x38
#define TXB0D3 0x39
#define TXB0D4 0x3A
#define TXB0D5 0x3B
#define TXB0D6 0x3C
#define TXB0D7 0x3D
#define CANSTAT3 0x3E
#define CANCTRL3 0x3F

#define TXB1CTRL 0x40
#define TXB1SIDH 0x41
#define TXB1SIDL 0x42
#define TXB1EID8 0x43
#define TXB1EID0 0x44
#define TXB1DLC 0x45
#define TXB1D0 0x46
#define TXB1D1 0x47
#define TXB1D2 0x48
#define TXB1D3 0x49
#define TXB1D4 0x4A
#define TXB1D5 0x4B
#define TXB1D6 0x4C
#define TXB1D7 0x4D
#define CANSTAT4 0x4E
#define CANCTRL4 0x4F

#define TXB2CTRL 0x50
#define TXB2SIDH 0x51
#define TXB2SIDL 0x52
#define TXB2EID8 0x53
#define TXB2EID0 0x54
#define TXB2DLC 0x55
#define TXB2D0 0x56
#define TXB2D1 0x57
#define TXB2D2 0x58
#define TXB2D3 0x59
#define TXB2D4 0x5A
#define TXB2D5 0x5B
#define TXB2D6 0x5C
#define TXB2D7 0x5D
#define CANSTAT5 0x5E
#define CANCTRL5 0x5F

#define RXB0CTRL 0x60
#define RXB0SIDH 0x61
#define RXB0SIDL 0x62
#define RXB0EID8 0x63
#define RXB0EID0 0x64
#define RXB0DLC 0x65
#define RXB0D0 0x66
#define RXB0D1 0x67
#define RXB0D2 0x68
#define RXB0D3 0x69
#define RXB0D4 0x6A
#define RXB0D5 0x6B
#define RXB0D6 0x6C
#define RXB0D7 0x6D
#define CANSTAT6 0x6E
#define CANCTRL6 0x6F

#define RXB1CTRL 0x70
#define RXB1SIDH 0x71
#define RXB1SIDL 0x72
#define RXB1EID8 0x73
#define RXB1EID0 0x74
#define RXB1DLC 0x75
#define RXB1D0 0x76
#define RXB1D1 0x77
#define RXB1D2 0x78
#define RXB1D3 0x79
#define RXB1D4 0x7A
#define RXB1D5 0x7B
#define RXB1D6 0x7C
#define RXB1D7 0x7D
#define CANSTAT7 0x7E
#define CANCTRL7 0x7F

#define RTS0 0x01
#define RTS1 0x02
#define RTS2 0x04

;********************************************************** 
;********************************************************** 
;  General control flags definitions


#define tbWork       bGenFlags1,0   ; working bit
#define tbReset      bGenFlags1,1   ; must reset
#define tbNewSPI     bGenFlags1,2   ; new SPI data available
#define tbRxMsgPend  bGenFlags1,3   ; new CAN message received
#define tbTxMsg      bGenFlags1,4   ; xmit next CAN message
#define tbRC2NowHigh bGenFlags1,5   ; robot PWM signal high




;********************************************************** 
;********************************************************** 
 
 
; MCP2510 Instructions 
#define d2510Rd      0x03      ; MCP2510 read instruction 
#define d2510Wrt     0x02      ; MCP2510 write instruction 
#define d2510Reset   0xC0      ; MCP2510 reset instruction 
#define d2510RTS     0x80      ; MCP2510 RTS instruction 
#define d2510Status  0xA0      ; MCP2510 Status instruction 
#define d2510BitMod  0x05      ; MCP2510 bit modify instruction 
 
 
 
#define _SSPEN  SSPCON,SSPEN
 



;********************************************************** 
;*************** SPECIAL CAN MACROS *********************** 
;********************************************************** 
 


 

; Read 2510 register Reg and return data in W. 
SPI_Read macro Reg 
          movlw     Reg 
          call      Rd2510Reg 
          endm 
 
; Write literal byte to 2510 register Reg. 
SPI_WriteL macro Reg,LitData 
          movlw     LitData 
          movwf     b2510RegData 
          movlw     Reg 
          call      Wrt2510Reg 
          endm 
 
; Write Data byte to 2510 register Reg. 
SPI_WriteV macro Reg,RegData 
          movfw     RegData 
          movwf     b2510RegData 
          movlw     Reg 
          call      Wrt2510Reg 
          endm 
 
; Write W byte to 2510 register Reg. 
SPI_WriteW macro Reg 
          movwf     b2510RegData 
          movlw     Reg 
          call      Wrt2510Reg 
          endm 
 
 
; Write bits determined by Mask & Data to 2510 register Reg. 
SPI_BitMod macro Reg,Mask,_Data 
          movlw     Mask 
          movwf     b2510RegMask 
          movlw     _Data 
          movwf     b2510RegData 
          movlw     Reg 
          call      BitMod2510 
          endm 
 
; Arm xmit buffers for xmission 
SPI_Rts macro _Data 
          movlw     _Data 
          call      Rts2510 
          endm 
      




        
 
;********************************************************** 
;********************************************************** 
;Support routines for communicating with 2510 chip 
;********************************************************** 
;********************************************************** 
 
;****************************************************** 
;CheckCANMsg 
; 
; Checks for message in Receive Buf 0.  If no message pending return 
; with Z flag set. 
;          
; If message pending: 
;     Load iRecID_L,iRecID_H with ID. 
;     Load bRecCount with number of bytes of data received. 
;     Load buffer at pRecDataBase with data 
;     Clear 2510 Receive Buffer 1 interrupt flag 
;     Set tbRxMsgPend flag and clear Z flag. 
; 
; NOTE: If message already pending doesn't check for new message. 
; 
;******************************************************/ 
CheckCANMsg 
 
          bcf       _Z                  ; for return 
          skipClr   tbRxMsgPend         ; new CAN message received 
          retlw 0                        ; Message already pending 
 
     ;; Test for Message pending in Receive Buffer 0   
           SPI_Read  CANINTF 
          andlw     0x01       
 
          skipNZ 
          retlw 0              ; Nothing in Rec Buf 0 
 
          bsf       tbRxMsgPend         ; new CAN message received 
 
     ;; Get ID of message source  
           SPI_Read  RXB0SIDH 
          movwf     iRecID_H 
           SPI_Read  RXB0SIDL         
          movwf     iRecID_L    
     ;; teste from a extended frame
          btfss iRecID_L,3                              
          goto jStdFrame
     ;; this is a extended frame
          SPI_Read RXB0EID8
          movwf iRecEX_H
          SPI_Read RXB0EID0
          movwf iRecEX_L
jStdFrame
          movlw 0xE0
          andwf iRecID_L,F 
     ;; Get number of bytes of data 
           SPI_Read  RXB0DLC 
          andlw     0x0F 
          movwf     bRecCount 
 
     ;; Get data from buffer. Up to 8 bytes based on  
          clrf      bCnt 
 
jRxChk11  jmpFeqF   bCnt,bRecCount,jRxChk90    ; no data left 
 
     ;; Calculate correct 2510 receive buffer location 
          movlw     RXB0D0 
          addwf     bCnt,W 
 
     ;; Get data byte 
          call      Rd2510Reg 
          movwf     b2510RegData     ; temporary save 
 
     ;; Calculate destination buffer location 
          movlw     pRecDataBase 
          addwf     bCnt,W 
          movwf     FSR 
 
     ;; Store data in buffer 
          movfw     b2510RegData     ; temporary save 
          movwf     INDF 
          incf      bCnt,F 
          goto      jRxChk11 
 
jRxChk90 
          SPI_BitMod CANINTF,0x01,0     ; Clear receive buffer 0 interrupt 
          bcf       _Z                  ; signal data pending 
          retlw 1 
 
 
;********************************************************** 
;SetConfigMode 
; 
;// Function Name: Set_Config_Mode() 
;********************************************************** 
SetConfigMode 
;  SPI_BitMod(CANCTRL, 0xE0, 0x80);    //Config. mode/ 
          bL2bV     0xE0,b2510RegMask 
          bL2bV     0x80,b2510RegData 
          movlw     CANCTRL 
          call      BitMod2510 
 
jSetConfigM1 
          movlw     CANSTAT 
          call      Rd2510Reg 
          andlw     0xE0 
          xorlw     0x80 
          jmpNZ     jSetConfigM1 
 
          return 
 
 
;********************************************************** 
;SetNormalMode   0x00
;SetSleepMode    0x20
;SetLoopBackMode 0x40
;SetListenMode   0x60
;SetConfigMode   0x80
; 
;// Function Name: Set_Normal_Mode() 
;********************************************************** 
SetNormalMode 
 
          bL2bV     0xE0,b2510RegMask 
          bL2bV     0x00,b2510RegData 
          movlw     CANCTRL 
          call      BitMod2510 
 
jSetNormalM1 
          movlw     CANSTAT 
          call      Rd2510Reg 
          andlw     0xE0 
          jmpNZ     jSetNormalM1 
 
          return 




;********************************************************** 
;SetLoopBackMode  
; 
;// Function Name: Set_LoopBack_Mode() 
;********************************************************** 
SetLoopBackMode 
          bL2bV     0x40,b2510RegData 
          bL2bV     0xE0,b2510RegMask 
          movlw     CANCTRL 
          call      BitMod2510 
 
jSetLoopBackM1 
          movlw     CANSTAT 
          call      Rd2510Reg 
          andlw     0xE0 
          xorlw     0x40  
          jmpNZ     jSetLoopBackM1 
 
          return 

;**************************************************

Set2510Mode 
          movwf     b2510RegData
          bL2bV     0xE0,b2510RegMask 
          movlw     CANCTRL 
          call      BitMod2510 
 
jSet2510M1 
          movlw     CANSTAT 
          call      Rd2510Reg 
          andlw     0xE0 
          xorlw     b2510RegData  
          jmpNZ     jSet2510M1 
 
          return 

 

;********************************************************** 
;SetListenMode  
; 
;// Function Name: Set_Listen_Mode() 
;********************************************************** 
SetListenMode 
 
          bL2bV     0xE0,b2510RegMask 
          bL2bV     0x60,b2510RegData 
          movlw     CANCTRL 
          call      BitMod2510 
 
jSetListenM1 
          movlw     CANSTAT 
          call      Rd2510Reg 
          andlw     0xE0 
          xorlw     0x60 
          jmpNZ     jSetListenM1 
 
          return 



;********************************************************** 
;WaitANDeqZ 
;         Wait for byte from address in W to AND with mask in 
;         b2510RegMask to be zero. Uses b2510RegAdr to hold address. 
;          
;********************************************************** 
WaitANDeqZ 
          movwf     b2510RegAdr         ; save 
 
jWaitANDeqZ 
          movfw     b2510RegAdr         ; save 
          call      Rd2510Reg 
          andwf     b2510RegMask,W 
          jmpNZ     jWaitANDeqZ 
          return 
 
 
;********************************************************** 
;********************************************************** 
 
 
;********************************************************** 
;**************** BASIC COMMUNICATION ********************* 
;********************************************************** 
 
 
;********************************************************** 
;Get2510Status 
;         Get Status byte from 2510. 
;// Function Name: SPI_ReadStatus() 
;********************************************************** 
Get2510Status 
          call      InitSPIBuf 
          movlw     d2510Status          ; MCP2510 Status instruction 
          call      LoadSPIByte 
          movlw     1                   ; expect 1 byte answer 
          call      LoadSPIZeros 
          call      ExchangeSPI 
          call      WaitSPIExchange 
          return 
 
;********************************************************** 
;Rd2510Reg 
;         Read 2510 register at address in W. Return results 
;         in W. Uses b2510RegAdr to hold address. 
;// Function Name: SPI_Read(uint address) 
;********************************************************** 
Rd2510Reg 
          movwf     b2510RegAdr         ; save 
          call      InitSPIBuf 
          movlw     d2510Rd              ; MCP2510 read instruction 
          call      LoadSPIByte 
          movfw     b2510RegAdr         ; get address 
          call      LoadSPIByte 
          movlw     1                   ; expect 1 byte answer 
          call      LoadSPIZeros 
          call      ExchangeSPI 
          call      WaitSPIExchange 
          movfw     pSPIBufBase+2 
          return 
 
;********************************************************** 
;Wrt2510Reg 
;         Write byte in b2510RegData to 2510 register at location in W.  
;         Uses b2510RegAdr to hold address. 
;// Function Name: SPI_Write(uint address) 
;********************************************************** 
Wrt2510Reg 
          movwf     b2510RegAdr         ; save 
          call      InitSPIBuf 
          movlw     d2510Wrt             ; MCP2510 write instruction 
          call      LoadSPIByte 
          movfw     b2510RegAdr         ; get address 
          call      LoadSPIByte 
          movfw     b2510RegData        ; get data 
          call      LoadSPIByte 
          call      ExchangeSPI 
          call      WaitSPIExchange 
          return 
 
 
;********************************************************** 
;BitMod2510 
;// Function Name: SPI_BitMod() 
;         Write data in b2510RegData using mask in b2510RegMask to  
;         address in W. Uses b2510RegAdr to hold address. 
;********************************************************** 
BitMod2510 
          movwf     b2510RegAdr         ; save 
          call      InitSPIBuf 
 
          movlw     d2510BitMod         ; MCP2510 bit modify instruction 
          call      LoadSPIByte 
 
          movfw     b2510RegAdr         ; address 
          call      LoadSPIByte 
 
          movfw     b2510RegMask        ; mask 
          call      LoadSPIByte 
 
          movfw     b2510RegData        ; data 
          call      LoadSPIByte 
 
          call      ExchangeSPI 
          call      WaitSPIExchange 
          return 
 
 
;********************************************************** 
;Rts2510 
;         Request to send to MCP2510. 
;         Send the request to send instruction to the CANbus Controller ORed 
;         with value in W.  Uses b2510RegData. 
;// Function Name: SPI_Reset() 
;********************************************************** 
Rts2510 
          movwf     b2510RegData 
          call      InitSPIBuf 
 
          movlw     d2510RTS            ; MCP2510 RTS instruction 
          iorwf     b2510RegData,W      ; get data and OR it with RTS 
          call      LoadSPIByte 
 
          call      ExchangeSPI 
          call      WaitSPIExchange 
          return 
 
 
;********************************************************** 
;Reset2510 
;         Reset MCP2510. 
;// Function Name: SPI_Reset() 
;********************************************************** 
Reset2510 
          call      InitSPIBuf 
          movlw     d2510Reset           ; MCP2510 reset instruction 
          call      LoadSPIByte 
          call      ExchangeSPI 
          call      WaitSPIExchange 
          return 
 
 
 
;********************************************************** 
;***************** LOCAL - DON'T CALL DIRECTLY ************ 
;********************************************************** 
 
;********************************************************** 
;InitSPIPort 
;         Intialize SPI port 
;********************************************************** 
InitSPIPort
	BANK0     
          movlw     0x11           ; SPI Master, Idle high, Fosc/16 
          movwf     SSPCON               
          BANK1 
          clrf      SSPSTAT        ; data sample at the middle, idle to active clock state
          bsf       PIE1, SSPIE      ; SSP int enable (BANK 1) 
          BANK0 
          bsf       SSPCON, SSPEN         ; enable SPI 
          return 
  

;*********************************************



Init2510aa
    call Reset2510 
    
    ; Set CLKOUT prescaler to div by 4
;    bL2bV 0x03, b2510RegMask
;    bL2bV 0x02, b2510RegData
    bL2bV 0x07, b2510RegMask ; disable clkout
    bL2bV 0x02, b2510RegData
    movlw CANCTRL
    call BitMod2510

    ; Set physical layer configuration
    ;
    ; Fosc       = 16Mhz
    ; BRP        = 2  ( divide by 8)
    ; Sync Seg   = 1TQ
    ; Prop Seg   = 2TQ
    ; Phase Seg1 = 10TQ
    ; Phase Seg2 = 3TQ
    ;
    ; TQ = 2 * (1/Fosc) * (BRP+1)
    ;

    SPI_WriteL CNF1, 0x02   ; 0x07 set BRP

    ; BTLMODE_CNF3   0x80
    ; SMPL_1X        0x00
    ; PHSEG1_3QT     0x10
    ; PHSEG_1QT      0x00
    SPI_WriteL CNF2, 0x90   ;

    ; PHSEG2_3QT     0x03 
    SPI_WriteL CNF3, 0x03   ;



    ; Configure receive buffer 0 Mask and filters
    ; Receive buffer o will not be used
     SPI_WriteL RXM0SIDH, 0x00
     SPI_WriteL RXM0SIDL, 0x00

     SPI_WriteL RXF0SIDH, 0xdf
     SPI_WriteL RXF0SIDL, 0x07

 ;   SPI_WriteL RXF1SIDH, 0xFF
 ;   SPI_WriteL RXF1SIDL, 0xFF

    ; Configure receive Buffer 1 Mask and Filyers
 ;   SPI_WriteL RXM1SIDH, 0xFF
 ;   SPI_WriteL RXM1SIDL, 0xE0

    ; Initialize Filter 2 to match x0 bBasedRecID from DIP switch
 ;   SPI_WriteL RXF2SIDH, bBaseRecID
 ;   SPI_WriteL RXF2SIDL, 0x00

     ; Initialize Filter 3 to match x1 bBasedRecID from DIP switch
 ;   incf    bBaseRecID,F
 ;   SPI_WriteL RXF3SIDH, bBaseRecID
 ;   SPI_WriteL RXF3SIDL, 0x00

     ; Initialize Filter 4 to match x1 bBasedRecID from DIP switch
 ;   incf    bBaseRecID,F
 ;   SPI_WriteL RXF4SIDH, bBaseRecID
 ;   SPI_WriteL RXF4SIDL, 0x00

     ; Initialize Filter 5 to match x1 bBasedRecID from DIP switch
;    incf    bBaseRecID,F
;    SPI_WriteL RXF5SIDH, bBaseRecID
;    SPI_WriteL RXF5SIDL, 0x00

 ;    movlw b'11110000'
 ;    andwf bBaseRecID,F
  
    ; clear MCP2515 interrupts
    ;  1-------    Error on TX/RX message  
    ;  -0------    CAN bus activity
    ;  --0-----    Interrupt on EFLAG reg
    ;  ---0----    TXB2 empty
    ;  ------10    Messsage received on RXB1
    ;  -------1    Messsage received on RXB0

    bL2bV   0x83, b2510RegData
    movlw CANINTE
    call Wrt2510Reg

  ;    call SetNormalMode
     movlw 0x40
     call Set2510Mode
  ;   call SetListenMode

    return




;****************************************************
;********************************************************** 
;InitSPIBuf 
;         Initializes SPI buffer for transaction.  Sets up 
;         FSR as buffer pointer. 
;********************************************************** 
InitSPIBuf 
          clrf      bSPICnt 
          movlw     pSPIBufBase 
          movwf     pSPIBuf 
          movwf     FSR 
          return 
 



ProccessSPI
     skipSet bSPICnt,2
     ; buffer not full yet
     return
  
     ; disable SPI interrupt
     BANK1
     bcf   PIE1, SSPIE
     BANK0

     ; enable SPI  
     BANK1
      bsf  PIE1, SSPIE      

     BANK0
    
     return

;*****************************************************

;********************************************************** 
;LoadSPIByte 
;         Load byte in W to SPI buffer.  Assumes FSR is pointer. 
;********************************************************** 
LoadSPIByte 
          movwf     INDF 
          incf      FSR,F 
          return 
 
;********************************************************** 
;LoadSPIZeros 
;         Load number of zeros in W to SPI buffer.   
;         Assumes FSR is pointer. 
;********************************************************** 
LoadSPIZeros 
          andlw     0xFF 
          skipNZ 
          return                        ; finished 
          clrf      INDF 
          incf      FSR,F 
          addlw     0xFF                ; Subtract 1 from W 
          jmpNZ     LoadSPIZeros 
          return 
 
;********************************************************** 
;ExchangeSPI 
;         Initiate SPI transaction.   
;********************************************************** 
ExchangeSPI 
     ;; Get number of bytes to exchange 
          bV2bV     FSR,bSPICnt 
          movlw     pSPIBufBase 
          subwf     bSPICnt,F 
 
          skipNZ 
          return                        ; nothing to exchange 
 
          movlw     pSPIBufBase 
          movwf     pSPIBuf 
 
     ;; Load 1st byte to begin exchange 
          bcf       tp2510_CS_           ; CS_ for 2510 chip 
          movfw     pSPIBufBase         ; get 1st byte in buffer 
          movwf     SSPBUF              ; send it 
          return 
 
 
;********************************************************** 
;WaitSPIExchange 
;         Wait for SPI transaction to be completed. 
;********************************************************** 
WaitSPIExchange 
  ;      bsf INTCON,GIE
  ;      nop
  ;      nop
  ;      bcf INTCON,GIE
        movf    bSPICnt,F 
        btfss   _Z 
        goto    WaitSPIExchange 
        return 



       