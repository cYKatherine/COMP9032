;
; lab_4.asm
;
; Created: 28/10/2022 5:40:11 AM
; Author : Group C
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
.def temp_counter = r18
.def counter = r19
.def lcd_register = r20
.def four_counter = r22


.cseg
.org 0x0000
	jmp RESET
; Just to test different ports
.org INT0addr
	jmp EXT_INT0
.org INT1addr
	jmp EXT_INT0
.org INT2addr
	jmp EXT_INT0
.org INT3addr
	jmp EXT_INT0
.org INT4addr
	jmp EXT_INT0
.org INT5addr
	jmp EXT_INT0
.org INT6addr
	jmp EXT_INT0
.org INT7addr
	jmp EXT_INT0
.org OVF0addr							; time counter overflow
	jmp Timer0OVF


/* LCD macros and functions */
.macro do_lcd_command					; perform lcd_command, transfer command to LC
	ldi lcd_register, @0				; load data @0 to lcd_register
	rcall lcd_command					; rcall lcd_command
	rcall lcd_wait						; rcall lcd_wait
.endmacro

.macro do_lcd_data						; set lcd data, transfer data to LCD
	mov lcd_register, @0				; move data @0 to lcd_register
	rcall lcd_data						; rcall lcd_data
	rcall lcd_wait						; rcall lcd_wait
.endmacro

.macro lcd_set
	sbi PORTA, @0						; set pin @0 of port A to 1
.endmacro

.macro lcd_clr
	cbi PORTA, @0						; clear pin @0 of port A to 0
.endmacro

lcd_command:							; send a command to LCD IR
	out PORTF, lcd_register
	nop
	lcd_set LCD_E						; use macro lcd_set to set pin 7 of port A to 1
	nop
	nop
	nop
	lcd_clr LCD_E						; use macro lcd_clr to clear pin 7 of port A to 0
	nop
	nop
	nop
	ret

lcd_data:								; send a data to LCD DR
	out PORTF, lcd_register				; output lcd_register to port F
	lcd_set LCD_RS						; use macro lcd_set to set pin 7 of port A to 1
	nop
	nop
	nop
	lcd_set LCD_E						; use macro lcd_set to set pin 6 of port A to 1
	nop
	nop
	nop
	lcd_clr LCD_E						; use macro lcd_clr to clear pin 6 of port A to 0
	nop
	nop
	nop
	lcd_clr LCD_RS						; use macro lcd_clr to clear pin 7 of port A to 0
	ret

lcd_wait:								; LCD busy wait
	push lcd_register					; push lcd_register into stack
	clr lcd_register					; clear lcd_register
	out DDRF, lcd_register				; set port F to output mode
	out PORTF, lcd_register				; output 0x00 in portF
	lcd_set LCD_RW
lcd_wait_loop:
	nop
	lcd_set LCD_E						; use macro lcd_set to set pin 6 of port A to 1
	nop
	nop
    nop
	in lcd_register, PINF				; read data from port F to lcd_register
	lcd_clr LCD_E						; use macro lcd_clr to clear pin 6 of port A to 0
	sbrc lcd_register, 7				; skip if bit 7 in lcd_register is cleared
	rjmp lcd_wait_loop
	lcd_clr LCD_RW						; use macro lcd_clr to clear pin 7 of port A to 0
	ser lcd_register					; set lcd_register to 0xFF
	out DDRF, lcd_register				; set port F to input mode
	pop lcd_register					; pop r16 from stack
	ret

.equ F_CPU = 16000000
.equ DELAY_1MS = F_CPU / 4 / 1000 - 4
; 4 cycles per iteration - setup/call-return overhead

sleep_1ms:
	push r24							; push r24 to stack
	push r25							; push r25 to stack
	ldi r25, high(DELAY_1MS)			; load high 8 bits of DELAY_1MS to r25
	ldi r24, low(DELAY_1MS)				; load low 8 bits of DELAY_1MS to r25
delayloop_1ms:
	sbiw r25:r24, 1						; r25:r24 = r25:r24 - 1
	brne delayloop_1ms					; branch to delayloop_1ms
	pop r25								; pop r25 from stack
	pop r24								; pop r24 from stack
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
	ldi temp1, '0'
	add digit_1, temp1
	add digit_2, temp1
	add digit_3, temp1
	nop	
.endmacro

/* Interrupts*/
RESET:
	clr four_counter
	clr counter
	clr temp_counter

	; LCD port setup
	ser temp1							; set temp1 to 0xFF
	out DDRF, temp1						; set PORT F to input mode
	out DDRA, temp1						; set PORT A to input mode
	clr temp1							; clear temp1
	out PORTF, temp1					; out 0x00 to PORT F
	out PORTA, temp1					; out 0x00 to PORT A

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

	; Want OpO to be triggered on falling edge, ISC01
	;ldi temp1, (2 << ISC00)					; set INT0 as falling edge triggered interrupt
	;sts EICRA, temp1
	;in temp1, EIMSK							; enable INT0
	;ori temp1, (1 << INT0)
	;out EIMSK, temp1

	; Want OpO to be triggered on falling edge, ISC01
	ldi temp1, (1<<ISC01) | (1 << ISC11) | (1 << ISC21) | (1 << ISC31)
	sts EICRA, temp1

	ldi temp1, (1 << ISC41) | (1 << ISC51) | (1 << ISC61) | (1<<ISC71)
	sts EICRB, temp1
	ser temp1
	out EIMSK, temp1
	sei	

	; Set up clock
	; clear clock counter
	clr temp1
	out TCCR0A, temp1
	ldi temp1, 0b00000101				; prescaler = 1024
	out TCCR0B, temp1
	ldi temp1, 1<<TOIE0					; =16384 microseconds
	sts TIMSK0, temp1					; T/C0 interrupt enable 

	; EIMSK enable interrupt from INT0 (SCL)

	;ser temp1
	;out DDRC, temp1

	sei										; set interrupt flag
	jmp main

EXT_INT0:									; detected OpO low
	in temp1, SREG
	push temp1
	
	inc four_counter						; increase four_counter
	cpi four_counter, 4
	breq four_detected
	rjmp continue
four_detected:
	inc counter								; increase counter
	clr four_counter
continue:
	pop temp1
	out SREG, temp1
	reti									; return from interrupt

Timer0OVF:									; Count to 61 interrupts
	in temp1, SREG
	push temp1								; prologue starts

	inc temp_counter						; increase the temporary counter by one
	; timer counts from 0 to 255, everytime we reaches 255, an "interrupt" was triggerd.
	; 256*1024/16MHz = 16384 us => every 16384us, there will be one interrupt
	; 1m/16384=61, every one sec there will be 61 interrupts
	cpi temp_counter, 61					; check if temp_counter == 61
	brne jump_not_second					; if temp_counter != 61, jump to not_second 
	jmp display_digits						; otherwise, jump to display_digits

jump_not_second:
	jmp not_second

display_digits:
	do_lcd_command 0b00000001				; clear and move cursor to front of row

	to_decimal counter
	do_lcd_data digit_1
	do_lcd_data digit_2
	do_lcd_data digit_3
	ldi temp1, 'r'
	do_lcd_data temp1
	ldi temp1, 'p'
	do_lcd_data temp1
	ldi temp1, 's'
	do_lcd_data temp1

	clr counter
	clr temp_counter
	rjmp end_timer

not_second:
	rjmp end_timer

end_timer:
	pop temp1
	out SREG, temp1
	reti

main:
	nop
	rjmp main