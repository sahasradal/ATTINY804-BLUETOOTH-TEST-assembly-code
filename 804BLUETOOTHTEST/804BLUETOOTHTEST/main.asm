;
; 804BLUETOOTHTEST.asm
;
; Created: 05/05/2022 06:37:42
; Author : Manama
;


.equ fclk = 10000000

.DEF SLAVE_REG = R17
.DEF TEMP = R16
.def data = r19
.def counter = r25
.equ BAUD = 9600
.equ fBAUD = ((64 * fclk) /(16 * BAUD)+0.5)
















.macro millis
ldi YL,low(@0)
ldi YH,high(@0)
rcall delayYx1mS
.endm

.macro micros
ldi XL,low(@0-1)  ;1
ldi XH,high(@0-1) ;1
rcall us		  ;2
.endm

.macro BLETX
ldi ZL,low(2*@0)
ldi ZH,high(2*@0)
rcall TXstring1
.endm


.cseg


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;PROTECTED WRITE (processor speed set to 10MHZ)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;ldi r16,0b01111110  ; osclock =0 bit7  and bit 1:0 = 0x2 for 20mhz ,all reserved bits to ber witten as 1.
;out OSCCFG,r16

	ldi r16,0Xd8
	out CPU_CCP,r16
	ldi r16,0x01
	STS CLKCTRL_MCLKCTRLB,R16
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	rcall UART_SETUP          ; call subroutine to setup UART engine
test:
	rcall BLE_SETUP
	rcall TXUSART	
	rjmp test
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


UART_SETUP:
	lds r16,PORTB_DIR	
	ori r16,(1<<2)
	sts PORTB_DIR,r16			;set portB PB2 direction as output
	lds r16,PORTB_OUT
	ori r16,(1<<2)
	sts PORTB_OUT,r16			;set portB PB2 as 1 or +ve
	ldi r16,low(fBAUD)			;load low value of fBAUD as calculated in the formula provided above
	ldi r17,high(fBAUD)			;load high value of fBAUD as calculated in the formula provided above
	sts USART0_BAUD,r16			;store low fBAUD in BAUD register
	sts USART0_BAUD + 1,r17		;store low fBAUD in BAUD register
	ldi r16,USART_TXEN_bm	    ;0b01000000 ,(1<<6) ,bitmask value to enable USART transmit 
	sts USART0_CTRLB,r16		;store TXEN in USART_CTRLB register
;	rjmp TXUSART				;jump to TXUSART address (main routine)
	ret

sendbyte:
	lds r16,USART0_STATUS		;copy USART status register to r16
	andi r16,USART_DREIF_bm     ;(0b00100000) AND with DATA REGISTER EMPTY FLAG bitmask (position5) to check flag status 0= not empty 1= empty 
	sbrs r16,5					;skip next instruction if bit 5 is 1 (means flag set for data transmit buffer ready to receive new data )
	rjmp sendbyte				;if DREIF = 0 ,bit 5 in r16 is 0 then loop back to sendbyte until DREIF = 1
	mov r16,r24					;copy data to be transmitted from r20 to r16
	sts USART0_TXDATAL,r16		;store r16 in TXDATAL transmit data low register 
	ret

/*
send_temp:
	ldi YL,low(minussign)		; set pointer Y to SRAM location minus which is the starting point of the converted ASCII values holding temprature
	ldi YH,high(minussign)
	ldi r25,9					; load counter value 9 , 9 bytes holding 1 sign ,3 main digits , decimal point, 4 fraction digits 
loop:
	cpi r25,5
	breq insert
	ld r24, Y+					; load r24 with value pointed by Y pointer and increase pointer to next address
insert1:
	rcall sendbyte				; call routine to send byte to UART engine
	dec r25						; decrease counter
	breq exit					;if null/0x00 the end of string has reached
	rjmp loop					;jump to loop back through the string until a null is encountered
	rcall ms10					; wait time of 10ms
exit:ret
insert:
	ldi r24,'.'
	rjmp insert1

	*/

TXstring:
	ldi ZL,low(2*string)
	ldi ZH,high(2*string)	
loop2:
	lpm r24,Z+
	cpi r24,0x00
	breq exit2					;if null/0x00 the end of string has reached
	rcall sendbyte
	rjmp loop2					;jump to loop back through the string until a null is encountered
exit2:ret


TXstring1:
	lpm r24,Z+
	cpi r24,0x00
	breq exit21					;if null/0x00 the end of string has reached
	rcall sendbyte
	rjmp TXstring1					;jump to loop back through the string until a null is encountered
exit21:ret


TXUSART:
	rcall TXstring				;call routine to transmit a null terminated string
;	rcall send_temp				; call routine send temp which reads values in SRAM and sends to UART engine
	ldi ZL,low(2*string2)		; string 2 is nothing but carriage feed and line feed for temprature values
	ldi ZH,high(2*string2) 
	rcall loop2
	ret

string: .db "TEMPERATURE", '\n', '\r' , 0
string2:.db " ",'\n', '\r' , 0
AT: .db "AT", '\n', '\r' , 0
role: .db "AT+ROLE0", '\r', '\n' , 0
service: .db"AT+UUIDFFE0", '\r', '\n' , 0
char: .db "AT+CHARFFE1", '\r', '\n' , 0
name: .db "AT+NAMESAJEEV", '\r', '\n' , 0

BLE_SETUP:
	BLETX AT
	rcall ms500
	BLETX role
	rcall ms500
	BLETX service
	rcall ms500
	BLETX char
	rcall ms500
	BLETX name
	rcall ms500
	ret





























	; ============================== Time Delay Subroutines =====================
; Name:     delayYx1mS
; Purpose:  provide a delay of (YH:YL) x 1 mS
; Entry:    (YH:YL) = delay data
; Exit:     no parameters
; Notes:    the 16-bit register provides for a delay of up to 65.535 Seconds
;           requires delay1mS

delayYx1mS:
    rcall    delay1mS                        ; delay for 1 mS
    sbiw    YH:YL, 1                        ; update the the delay counter
    brne    delayYx1mS                      ; counter is not zero

; arrive here when delay counter is zero (total delay period is finished)
    ret
; ---------------------------------------------------------------------------
; ---------------------------------------------------------------------------
; Name:     delay1mS
; Purpose:  provide a delay of 1 mS
; Entry:    no parameters
; Exit:     no parameters
; Notes:    chews up fclk/1000 clock cycles (including the 'call')

delay1mS:
    push    YL                              ; [2] preserve registers
    push    YH                              ; [2]
    ldi     YL, low(((fclk/1000)-18)/4)     ; [1] delay counter              (((fclk/1000)-18)/4)
    ldi     YH, high(((fclk/1000)-18)/4)    ; [1]                            (((fclk/1000)-18)/4)

delay1mS_01:
    sbiw    YH:YL, 1                        ; [2] update the the delay counter
    brne    delay1mS_01                     ; [2] delay counter is not zero

; arrive here when delay counter is zero
    pop     YH                              ; [2] restore registers
    pop     YL                              ; [2]
    ret                                     ; [4]

; ---------------------------------------------------------------------------

; ---------------------------------------------------------------------------
; Name:     delay10uS
; Purpose:  provide a delay of 1 uS with a 16 MHz clock frequency ;MODIFIED TO PROVIDE 10us with 1200000cs chip by Sajeev
; Entry:    no parameters
; Exit:     no parameters
; Notes:    add another push/pop for 20 MHz clock frequency

delay10uS:
    ;push    temp                            ; [2] these instructions do nothing except consume clock cycles
    ;pop     temp                            ; [2]
    ;push    temp                            ; [2]
    ;pop     temp                            ; [2]
    ;ret                                     ; [4]
     nop
     nop
     nop
     ret

; ---------------------------------------------------------------------------

us1:
	nop  ; 1 cyc (0.0000001 sec = 100 nano ; 1us = 1000ns)
	nop	 ; 1
	nop  ; 1
	nop  ; 1
	ret  ; call 2 cyc, ret 4 cyc

us:
	nop	 ;1
	nop  ;1
	nop  ;1
	nop  ;1
	nop  ;1
	nop  ;1
	sbiw XH:XL,1	;2
	brne us		;2  (1 for last iteration)
	ret		    ;4 + 2  for ret and call , (takes 13 cycles(1.3us) total + loop), delayvalue-2 is to makeup for the extra 3cycles.If 10us is needed delay value of 9 is loaded


ms1:
		millis 1
		ret
ms10:
		millis 10
		ret
ms50:
		millis 50
		ret
ms100:
		millis 100
		ret
ms500:
		millis 500
		ret
ms750:
		millis 750
		ret
ms1000:
		millis 1000
		ret

; ============================== End of Time Delay Subroutines ==============---------------------------------------------------------------------------------