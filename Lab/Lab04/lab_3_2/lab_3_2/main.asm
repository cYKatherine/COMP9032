;
; lab_3_2.asm
;
; Created: 6/10/2022 1:52:54 PM
; Author : Katherine Chen
;


; Replace with your application code
.include "m2560def.inc"
.def sign =r16				;define sign to be resgiter r16
.def character =r17			;define character to be resgiter r17

setup:
	ldi sign, '-'			;load value '-' into sign
	ldi character, '0'		;load value '6' into character, assumption is there we will only load 0-9 here
start:
	cpi sign, '-'			;comapre sign with '-'
	breq negative			;if sign == '-', goto negative
	mov r18, character		;else r18 <- character
	subi r18, 0x30			;r18 <- r18 - 0x30, because we need to -48 to get the actual value from the ascii table. r18 is where the result is stored
	rjmp halt				;jump to halt
negative:
	ldi r19, -1				;r19 <- -1
	mov r18, character		;r18 <- character
	subi r18, 0x30			;r18 <- r18 - 0x30
	sub r19, r18			;r19 <- r19 - r18
	mov r18, r19			;r18 <- r19
	inc r18					;r18 <- r18 + 1, r18 is where the result is stored
halt:
	rjmp halt				;halt the processor execution
