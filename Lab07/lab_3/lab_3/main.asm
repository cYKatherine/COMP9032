;
; lab_3.asm
;
; Created: 22/10/2022 12:41:18 AM
; Author : Group C
;
; In this program, we use: Port C for LED, Port D for Keypad, Port A4-7 for LCD control, and Port F for LCD data
; Pin connections: PF0-7 with D0-7
; LED PortC
; Keypad: C3 to PL0, R3 to PL4
; LCD data: BE to PA4

.include "m2560def.inc"

.def lcd_register = r25
.def row = r16							; current row number
.def col = r17							; current column number
.def rmask = r18						; mask for current row
.def cmask = r19						; mask for current column
.def temp1 = r20						; define r20 as temp1 
.def temp2 = r21						; define r21 as temp2 
.def number_temp = r24					; register to keep track of temporary numbers as they are input
.def running_result = r22				; register to keep track of running total
.def temp_running_result = r8
.def delay_register = r23

.def digit_1 = r5						; place to store digits of number
.def digit_2 = r6
.def digit_3 = r7
.def hex_1 = r3							; place to store hex digits
.def hex_2 = r4

.def unit = r2							; mark whether decimal or hex showing. 0 = decimal. 1 = hex.

.equ KEYPAD_PORTDIR = 0xF0				; use PortL for input/output from keypad: PF7-4, output, PF3-0, input
.equ INITCOLMASK = 0xEF					; scan from the leftmost column, the value to mask output
.equ INITROWMASK = 0x01					; scan from the bottom row
.equ ROWMASK = 0x0F						; low four bits are output from the keypad. This value mask the high 4 bits.

.equ LCD_RS = 7
.equ LCD_E = 6
.equ LCD_RW = 5
.equ LCD_BE = 4

jmp reset

; decimal to digits
.macro decimal_to_digits
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
	rjmp end_decimal_to_digits
over_hundred:
	subi @0, 100
	inc digit_1
	rjmp check_hundred
over_ten:
	subi @0, 10
	inc digit_2
	rjmp check_ten
end_decimal_to_digits:
	nop	
.endmacro

; Turns decimal to ascii.
.macro decimal_to_ascii
	ldi temp1, -'0'
	sub @0, temp1
.endmacro

.macro hex_to_ascii
	ldi temp1, 10
	cp @0, temp1						; compare @0 with 10
	brsh convert_to_hex_letter			; if @0 >= 10, convert to letter
	ldi temp1, -'0'						; else, convert to number ascii
	sub @0, temp1
	rjmp end_hex_to_ascii
convert_to_hex_letter:
	ldi temp1, 10
	sub @0, temp1
	ldi temp1, -'A'
	sub @0, temp1
end_hex_to_ascii:
	nop
.endmacro

; macro parameters:
; @0-temp_running_result
; Finding the remainder of temp_running_result and 16, and store the value in temp_running_result
.macro divide_by_hex
	clr temp1							; temp1 = 0, temp1 is used to find the division result
start:
	cpi @0, 16							; compare temp_running_result and 16
	brsh minus							; if temp_running_result >= 16, go to minus
	rjmp end							; else go to the end
minus:
	subi @0, 16							; temp_running_result -= 16
	inc temp1							; temp1 += 1
	rjmp start
end:
	nop
.endmacro

; decimal to hex
; Take in the number and keep dividing it by 16 until the value is 0
.macro decimal_to_hex
	divide_by_hex @0
	mov hex_2, @0
	mov @0, temp1
	divide_by_hex @0
	mov hex_1, @0
.endmacro


/* Task macros */
.macro check_overflow
	brcs overflow
	;brvs overflow						; if overflow bit set, jump to overflow
	tst @0								; If @0 is not zero, jump to overflow.
	brne overflow

	rjmp exit_overflow

overflow:
	ldi temp2, 0xFE						; blink pattern
	out PORTC, temp2
	wait
	wait							
	ldi temp2, 0x00						; clear blink
	out PORTC, temp2
exit_overflow:
	nop
.endmacro

.macro input_number						; Take the number in number_temp and multiply it by 10.
	ldi temp2, 10
	mul number_temp, temp2

	mov number_temp, r0					; Store multiplied value to number_temp
	adc number_temp, @0					; Add the parameter

	check_overflow r1					; flash if r1 != 0 or overflow is detected

	rjmp end_input_number

end_input_number:
	nop
.endmacro


; Wait macro
.macro wait
	ser delay_register
wait_loop:
	dec delay_register
	tst delay_register
	breq wait_end
	rcall sleep_1ms
	rjmp wait_loop
wait_end:
	nop
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

reset:
	ldi temp1, KEYPAD_PORTDIR			; Port L columns are outputs, rows are inputs
	sts	DDRL, temp1
	ser temp1							; PORTC is outputs for LED
	out DDRC, temp1				
	clr temp1							; LED off
	out PORTC, temp1	

	; LCD port setup
	ser temp1
	out DDRF, temp1						; Port F and A set for output
	out DDRA, temp1
	clr temp1
	out PORTF, temp1
	out PORTA, temp1

	clr number_temp
	clr running_result
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



main:
	ldi cmask, INITCOLMASK				; initial column mask 1110 1111
	clr	col								; initial column = 0
colloop:								; Loop through columns C0 (P5) C1 (P6) C2 (P7) C3 (P8), setting voltage to zero
	cpi col, 4							; compare col with 4
	breq main							; When reaching C4, start from 0 again
	sts	PORTL, cmask			
	ldi temp1, 0xFF						; load 0xff to temp1
delay:
	dec temp1							; decrease temp1, temp1 -= 1
	brne delay							; if temp1 is not 0x00, branch to delay, else continue

	lds	temp1, PINL						; read PORTL, we care about low bits R0 (P1), R1 (P2), R3 (P3), R4 (P4) so mask off high bits
	andi temp1, ROWMASK
	cpi temp1, 0xF						; check if any rows are on, i.e., any button is pressed
	breq nextcol	
										; if yes, find which row is on
	ldi rmask, INITROWMASK				; initialise row check 0000 0001
	clr	row								; initial row = 0
rowloop:
	cpi row, 4							; Loop through rows R0, R1, R2, R3. When reaching R4, row scan is over. No press detected.
	breq nextcol						; if row = 4, branch to nextcol
	mov temp2, temp1					; Save reading from Port D input in temporary location.
	and temp2, rmask					; check masked bit. If masked bit is high, press not detected. Otherwise, press detected.
	breq convert 						; if bit is clear, convert the bitcode
	inc row								; else move to the next row
	lsl rmask							; shift the mask to the next bit
	jmp rowloop

nextcol:
	lsl cmask							; else get new mask by shifting left and 
	inc col								; increment column value
	jmp colloop							; and check the next column

convert:
	wait
	cpi col, 3							; if column is 3 we have a letter
	breq letters						; branch to letters
	cpi row, 3							; if row is 3 we have a symbol or 0
	breq symbols_jmp1					; branch to symbols

	; We have a number in 1-9, start filling the "buffer"
	mov temp1, row						; otherwise we have a number in 1-9
	; To get the actual value, temp1 = row * 3 + col + 1
	lsl temp1							; temp1 left shift, temp1 = 2 * row
	add temp1, row						; temp1 += row
	add temp1, col						; add the column address to get the value
	inc temp1							; temp1 += 1

	jmp number_detected

symbols_jmp1:
	jmp symbols

letter_d_jmp1:
	jmp letter_d

letters:
	cpi row, 2							; If row is 2 we have C
	breq letter_c

	cpi row, 3							; if row is 3 we have D
	breq letter_d_jmp1

	jmp end

letter_c:
	ser temp1							; Set temp1 to be 0xFF
	eor unit, temp1						; unit XOR 0xFF, toggle convert
	tst unit
	breq show_hex						; if unit=1, we are showing hex, otherwise show decimal

show_decimal:
	do_lcd_command 0b11000000			; Move cursor to front of second row
	; Convert decimal number to string and load digits for displaying
	mov temp2, running_result	
	decimal_to_digits temp2	
	decimal_to_ascii digit_1
	decimal_to_ascii digit_2
	decimal_to_ascii digit_3
	ldi temp1, '0'						; display an empty space to clear hex display
	do_lcd_data temp1
	do_lcd_data digit_1
	do_lcd_data digit_2
	do_lcd_data digit_3

	rjmp main

show_hex:
	do_lcd_command 0b11000000			; Move cursor to front of second row
	; Convert decimal number to hex and load digits for displaying
	mov temp2, running_result
	decimal_to_hex temp2
	hex_to_ascii hex_1
	hex_to_ascii hex_2
	ldi temp1, '0'						; display 0x
	do_lcd_data temp1
	ldi temp1, 'x'
	do_lcd_data temp1
	do_lcd_data hex_1
	do_lcd_data hex_2
	jmp end

letter_d:
	ldi temp1, '='						; display = symbol
	do_lcd_data temp1

	add running_result, number_temp		; Add running_result and number_temp
	clr r1								; clear r1 (so no overflow is flagged from that in the macro, just check overflow bit)
	check_overflow r1

	do_lcd_command 0b11000000			; move to second row

	rjmp show_decimal
symbols:
	cpi col, 0							; If col is 0 we have star
	breq star
	clr temp1							; Clear temp1

	cpi col, 1							; If col is 1 we have zero
	breq number_detected					

; Otherwise, we have a hash(+)
hash:
	mul running_result, number_temp		; Multiply first two numbers
	mov running_result, r0				; Store the result to running_result
	check_overflow r1					; Check overflow and flash
	clr number_temp						; Clear number_temp as a symble detected and we need to reset this value
	ldi temp1, '+'						; Load + symble to temp1
	do_lcd_data temp1					; Display +

	jmp end

star:
	mov running_result, number_temp		; Move current number to running_result
	clr number_temp						; Clear number_temp waiting for the next number
	ldi temp1, '*'						; store symbol for * and display
	do_lcd_data temp1

	jmp end

number_detected:
	; Currently, temp1 holds the number we have discovered.
	input_number temp1					; run macro to update nuber
	subi temp1, -'0'
	do_lcd_data temp1					; print digit

	jmp end

end:
	jmp main							; restart main loop