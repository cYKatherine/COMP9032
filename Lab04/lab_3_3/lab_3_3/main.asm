;
; lab_3_3.asm
;
; Created: 6/10/2022 1:52:54 PM
; Author : Katherine Chen
;


; Replace with your application code
.include "m2560def.inc"
.def ah =r16				;define a to be resgiter r16:r17
.def al =r17
.def bh =r18				;define b to be resgiter r18:r19
.def bl =r19
.def ch =r20				;define c to be resgiter r20:r21
.def cl =r21
.def dh =r22				;define d to be resgiter r22:r23
.def dl =r23

; macro parameters:
; @0-ah, high bits of a
; @1-al, low bits of a
; @2-bh, high bits of b
; @3-bl, low bits of b
; Finding the gcd of (r16:r17) and (r18:r19)
.macro gcd
loop:
	cp @0, @2				;compare ah and bh
	brne insideLoop			;go to insideLoop if ah != bh
	cp @1, @3				;compare al and bl
	breq end1				;goto end1 if al == bl
insideLoop:
	cp @2, @0				;compare ah and bh
	brlo aBigger			;go to aBigger if bh < ah
	breq highBitsEqual		;go to highBitsEqual if bh == ah
highBitsEqual:
	cp @3, @1				;compare al and bl
	brlo aBigger			;goto aBigger if bl < al
bBigger:
	sub bl, al				;bl <- bl - al
	sbc bh, ah				;bh <- bh - ah
	rjmp loop				;go back to loop
aBigger:
	sub al, bl				;al <- al - bl
	sbc ah, bh				;ah <- ah - bh
	rjmp loop				;go back to loop
end1:
	nop
.endmacro


ldi ah, high(234)
ldi al, low(234)
ldi bh, high(156)
ldi bl, low(156)
ldi ch, high(89)
ldi cl, low(89)
ldi dh, high(100)
ldi dl, low(100)
gcd ah, al, bh, bl			;call macro with ah, al, bh, bl
gcd ah, al, ch, cl			;call macro with ah, al, ch, cl
gcd ah, al, dh, dl			;call macro with ah, al, dh, dl
gcd bh, bl, ch, cl			;call macro with bh, bl, ch, cl
gcd bh, bl, dh, dl			;call macro with bh, bl, dh, dl
gcd ch, cl, dh, dl			;call macro with ch, cl, dh, dl
end:
	rjmp end