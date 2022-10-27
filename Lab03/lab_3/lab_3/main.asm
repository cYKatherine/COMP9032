;
; lab_3.asm
;
; Created: 29/09/2022 4:09:56 PM
; Author : Katherine Chen
;


; To calculate square root of n:
; As it's one byte, it's 8 bit, the range is 0 - 255
; 
; i = 0;
; while (i*i <= n) i++;
; return i-1;
;


.include "m2560def.inc"
.def i =r16							; define i to be register r16

main:
		ldi i, 0					; load value 0 into i
loop:
		cpi i, 16					; i-16
		breq end					; if i == 16, go to the end (because it will overload)
		mul i, i					; r1:r0 <- i*i
		cp r5, r0					; r5 - i*i
		brlo end					; if i*i <= n, go to end
		inc i						; increment i by 1
		rjmp loop					; go back to the beginning of the loop
end:
		dec i						; decrement i by 1
		mov r6, i
halt:
		rjmp halt					; halt the processor execution