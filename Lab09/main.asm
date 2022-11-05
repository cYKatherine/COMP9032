;
; lab_4.asm
;
; Created: 28/10/2022 5:40:11 AM
; Author : wills
;



.include "m2560def.inc"

.equ LCD_RS = 7
.equ LCD_E = 6
.equ LCD_RW = 5
.equ LCD_BE = 4

.def digit_1 = r3
.def digit_2 = r4
.def digit_3 = r5

.def temp1 = r16
.def temp2 = r17
.def counter = r18
.def lcd_register = r20
.dseg TempCounter: 
	.byte 2
.dseg RevCounter:
	.byte 2

.cseg
.org $0000
jmp RESET
.org INT0addr
	jmp EXT_INT0
.org OVF0addr
	jmp Timer0OVF


.macro clear
	ldi ZL, low(@0)
	ldi ZH, high(@0)
	clr temp1
	st Z+, temp1
	st Z, temp1
.endmacro

/* LCD macros and functions */
.macro do_lcd_command					; perform lcd_command
	ldi lcd_register, @0
	rcall lcd_command
	rcall lcd_wait
.endmacro

.macro do_lcd_data						; set lcd data
	mov lcd_register, @0
	rcall lcd_data
	rcall lcd_wait
.endmacro

.macro lcd_set
	sbi PORTA, @0
.endmacro

.macro lcd_clr
	cbi PORTA, @0
.endmacro

lcd_command:
	out PORTF, lcd_register
	nop
	lcd_set LCD_E
	nop
	nop
	nop
	lcd_clr LCD_E
	nop
	nop
	nop
	ret

lcd_data:
	out PORTF, lcd_register
	lcd_set LCD_RS
	nop
	nop
	nop
	lcd_set LCD_E
	nop
	nop
	nop
	lcd_clr LCD_E
	nop
	nop
	nop
	lcd_clr LCD_RS
	ret

lcd_wait:
	push lcd_register
	clr lcd_register
	out DDRF, lcd_register
	out PORTF, lcd_register
	lcd_set LCD_RW
lcd_wait_loop:
	nop
	lcd_set LCD_E
	nop
	nop
    nop
	in lcd_register, PINF
	lcd_clr LCD_E
	sbrc lcd_register, 7
	rjmp lcd_wait_loop
	lcd_clr LCD_RW
	ser lcd_register
	out DDRF, lcd_register
	pop lcd_register
	ret

.equ F_CPU = 16000000
.equ DELAY_1MS = F_CPU / 4 / 1000 - 4
; 4 cycles per iteration - setup/call-return overhead

sleep_1ms:
	push r24
	push r25
	ldi r25, high(DELAY_1MS)
	ldi r24, low(DELAY_1MS)
delayloop_1ms:
	sbiw r25:r24, 1
	brne delayloop_1ms
	pop r25
	pop r24
	ret

sleep_5ms:
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	ret

/* END LCD MACROS */

; decimal to digits
.macro to_decimal
	clr digit_1
	clr digit_2
	clr digit_3
check_hundred:
	cpi @0, 100
	brsh over_hundred
check_ten:
	cpi @0, 10
	brsh over_ten
; The number is a single digit at this point
	mov digit_3, @0
	rjmp end_to_decimal
over_hundred:
	subi @0, 100 
	inc digit_1
	rjmp check_hundred
over_ten:
	subi @0, 10
	inc digit_2
	rjmp check_ten
end_to_decimal:
	ldi temp1, -'0'
	sub digit_1, temp1
	sub digit_2, temp1
	sub digit_3, temp1
	nop	
.endmacro

/* Interrupts*/
RESET:
	;cbi DDRD, 7							; set Port D for input
	;sbi PORTD, 7						; Pull up resistor

	; LCD port setup
	ser temp1
	out DDRF, temp1						; Port F and A set for output
	out DDRA, temp1
	clr temp1
	out PORTF, temp1
	out PORTA, temp1

	; LCD setup
	do_lcd_command 0b00111000			; 2x5x7
	rcall sleep_5ms
	do_lcd_command 0b00111000			; 2x5x7
	rcall sleep_1ms
	do_lcd_command 0b00111000			; 2x5x7
	do_lcd_command 0b00111000			; 2x5x7
	do_lcd_command 0b00001000			; display off
	do_lcd_command 0b00000001			; clear display
	do_lcd_command 0b00000110			; increment, no display shift
	do_lcd_command 0b00001111			; Cursor on, bar, blink

	; Set up clock
	; clear clock counter
	clr temp1
	out TCCR0A, temp1
	ldi temp1, 0b00000101 ; prescaler
	out TCCR0B, temp1
	ldi temp1, 1<<TOIE0
	sts TIMSK0, temp1

	clear TempCounter
	clear RevCounter
	; EIMSK enable interrupt from INT0 (SCL)

	ser temp1
	out DDRC, temp1

	; Want OpO to be triggered on falling edge, ISC01
	ldi temp1, (1 << ISC01)
	sts EICRA, temp1
	ldi temp1, (1 << INT0)
	out EIMSK, temp1
	sei									; set interrupt flag

	jmp main

EXT_INT0:									; detected OpO low
	in temp1, SREG
	push temp1
	push YL
	push YH
	push r24
	push r25
	ldi YL, low(RevCounter)
	ldi YH, high(RevCounter)

	ld r24, Y+
	ld r25, Y


	adiw r25:r24, 1

	st Y, r25
	st -Y, r24

	pop r25
	pop r24
	pop YH
	pop YL
	pop temp1
	out SREG, temp1
	reti



Timer0OVF: ; Count to 61 interrupts
	in temp1, SREG
	push temp1
	push XL
	push XH
	push YL
	push YH

	push r22
	push r23

	push r24
	push r25

	ldi YL, low(TempCounter)
	ldi YH, high(TempCounter)

	ld r24, Y+
	ld r25, Y
	adiw r25:r24, 1

	cpi r24, 61
	brne jump_not_second	
	jmp display_digits

jump_not_second:
	jmp not_second
display_digits:
	
	ldi XL, low(RevCounter)
	ldi XH, high(RevCounter)
	ld r22, X+						; low byte
	ld r23, X						; high byte



	// Shift divide by 4
	lsr r22
	lsr r22


	bst r23, 0
	bld r22, 6

	bst r23, 1
	bld r22, 7


	do_lcd_command 0b10000000	; Move cursor to front of row

	to_decimal r22
	do_lcd_data digit_1
	do_lcd_data digit_2
	do_lcd_data digit_3
	ldi temp1, 'r'
	do_lcd_data temp1
	ldi temp1, 'p'
	do_lcd_data temp1
	ldi temp1, 's'
	do_lcd_data temp1


	clear RevCounter
	clear TempCounter
	rjmp end_timer

not_second:
	st Y, r25
	st -Y, r24
	rjmp end_timer
end_timer:
	pop r25
	pop r24
	pop r23
	pop r22
	pop YH
	pop YL
	pop XH
	pop XL
	pop temp1
	out SREG, temp1
	reti
	
main:
	nop
	rjmp main