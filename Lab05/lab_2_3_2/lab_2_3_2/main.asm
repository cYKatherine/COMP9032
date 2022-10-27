;
; lab_2_3_2.asm
;
; Created: 13/10/2022 11:02:16 AM
; Author : Katherine Chen
;

.include "m2560def.inc"
.cseg
	rjmp main
.def a =r24				;define a to be resgiter r24
.def b =r25				;define b to be resgiter r25
.def c =r26				;define c to be resgiter r26
.def d =r27				;define d to be resgiter r27

; macro parameters:
; @0-a
; @1-b
; Finding the remainder of a and b, and store the value in a
.macro remainder
start:
	cp @0, @1				;compare a and b
	brsh minus				;if a >= b, go to minus
	rjmp end				;else go to the end
minus:
	sub @0, @1				;a = a - b
	rjmp start
end:
	nop
.endmacro

;main function to calculate gcd of 4 integers
main:
SPI_INIT:					;initialise stack pointer
	ldi r31,low(RAMEND)
	out SPL,r31
	ldi r31,high(RAMEND)
	out SPH,r31

	ldi a, 77				;assign value to a
	ldi b, 231				;assign value to b
	ldi c, 21				;assign value to c
	ldi d, 168				;assign value to d
	rcall gcd				;gcd(a,b), result stored in a
	mov b, c				;move c to r25
	rcall gcd				;gcd(a,c), result stored in a
	mov b, d				;move d to r25
	rcall gcd				;gcd(a, d), result stored in a

end:
	rjmp end

; gcd(a, b)
gcd:
	; Prologue
	push r16				;save r16 on the stack
	push YL					;save r29:r28 in the stack
	push YH
	in YL, SPL				;initialize the stack frame pointer value
	in YH, SPH
	sbiw Y, 2				;reserve space for local variables and parameters, let Y point to the top of the stack frame
	out SPH, YH				;update SP so that it points to the new stack top
	out SPL, YL
	std Y+1, r24			;get the actual parameter a
	std Y+2, r25			;get the actual parameter b

	; Function body
	cpi r25, 0				;compare b and 0
	brne L2					;go to L2 if b != 0
	ldd r24, Y+1			;if b==0, return a
	rjmp L1					;jump to epilogue

L2:
	ldd r24, Y+2			;load Y+2 (b) to r24
	ldd r25, Y+1			;load Y+1 (a) to r25
	remainder r25, r24		;get remainder of r25 % r24 and store it in r25
	rcall gcd

L1:
	; Epilogue
	adiw Y, 2				;de-allocate the stack frame for gcd()
	out SPH, YH				;restore SP
	out SPL, YL
	pop YH					;restore Y
	pop YL
	pop r16					;restore r16
	ret


