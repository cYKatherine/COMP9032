; task_3.asm
;
; Created: 6/10/2022 12:57:37 AM
; Author : H18A Group C
; Members:
; Version: 1.0
; This program will display one of 3 patterns on the LED board.
; There are four states
;    1. Pattern 1
;    2. Pattern 2
;    3. Pattern 3
;    4. Alternating between patterns 1, 2, 3, 4
; The second button resets.
; In the "state" register, 0x01 corresponds to pattern 1, 0x02 corresponds to pattern 2
;    0x04 corresponds to pattern 3, and 0x10 corresponds to pattern 4.
; Patterns are displayed based on which bit is set.
; To show all three patterns in an alternating fashion, we set "state" to 0x1F, then loop
; through the patterns.
; Each time the button is pressed, we shift the bit to the left. Once the 5th bit is set
;    i.e. "state" holds more than 0x1F, it is reset back to 0x01.
; We take the delay macro directly from the lecture slides.

.include "m2560def.inc"


.def pattern_1 = r23
.def pattern_2 = r24
.def pattern_3 = r25
.def state = r18
.def bounce_counter = r19
.def button_input = r20
.def delay_counter = r21



.equ F_CPU = 16000000
.equ DELAY_1MS = F_CPU / 4 / 1000 - 4
; 4 cycles per iteration - setup/call-return overhead
ldi pattern_1,0x01
ldi pattern_2,0x49
ldi pattern_3,0x53
ldi r18,0

rjmp setup

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

; The macro is used to check if a button is pressed.
; If the button is pressed, the value of button_input will be set to 0.
.macro button_pressed						; Check if button 1 is pressed
	ser button_input 						; Set button_input to all 1
    ldi bounce_counter, 10					; Initialise bounce counter to 10
loop_button:
    rcall sleep_1ms
	sbic PIND, @0							; If the button is pressed, skip next line
    inc bounce_counter						; If the button is NOT pressed, increase counter
	sbis PIND, @0							; If the button is not pressed, skip the next line
    dec bounce_counter						; If the button is pressed, decrease counter

    tst bounce_counter					; If bounce_counter reaches zero, move to waiting_button_release
    breq waiting_button_release

	cpi bounce_counter, 20					; Button pressed if the user pressed the button for (20-10)ms
    breq return_button_not_pressed

    rjmp loop_button
waiting_button_release:						; Button pressed, clear all bits in button_input register
	rcall sleep_1ms
	sbic PIND, @0							; If the button is not pressed, skip the next line
    inc bounce_counter						; If the button is pressed, decrease counter

	cpi bounce_counter, 20
	breq return_button_pressed

	rjmp waiting_button_release
return_button_not_pressed:					; Button not pressed, set all bits in button_input register
    ser button_input
	rjmp button_pressed_end
return_button_pressed:
	clr button_input
	rjmp button_pressed_end
button_pressed_end:
	nop
.endmacro

; The macro is used to set delay.
.macro delay
    ldi delay_counter, 255					; Set delay counter to 300
loop_delay:
    rcall sleep_1ms
	rcall sleep_1ms
    dec delay_counter
    TST delay_counter
    breq end_delay
	jmp loop_delay
end_delay:
    nop
.endmacro

.macro stop_button_pressed
	button_pressed 0
	sbrs button_input, 1
	jmp main
.endmacro

; Main function starts here.
setup:
    ;; LED
    ; Set PORT C for output
    ser r17
    out DDRC, r17

    ;; BUTTON 1 and 2
    ; Set PORT D for input
    clr r17
    out DDRD, r17
    ser r17
    out PORTD, r17

main:
    clr r16								; clear r16 and set lights to OFF
    out PORTC, r16
	ldi state, 1						; reset state to 0 (everything to 0)

    rjmp listener						; Jump to listener

; Assumptions
; 1. If we press the first button more than 4 times (without second button involved), the board won't do anything, it will stay at the last LED pattern
; 2. While the pattern is displaying, we won't interupt the displaying
listener:
    button_pressed 1					; If the first button is pressed
    tst button_input
    breq increment_state				; Go to increment_state

	button_pressed 0					; If the second button is pressed
    tst button_input
    breq listJumpToMain					; go to main (which is to reset everything)
	
	rjmp listener  

listJumpToMain:
	jmp main

; 00000001 off, initial state	
; 00000010 pattern 1
; 00000100 pattern 2
; 00001000 pattern 3
; 00010000 pattern 4
increment_state:
    lsl state                           ; bitshift left state

	sbrc state, 4						; if bit 4 is set
	rjmp continues_pattern				; go to continues_pattern

    rjmp set_pattern					; go to set_pattern

; When first button pressed less than 4 times, we go to this section
set_pattern:
	stop_button_pressed
    sbrc state, 1                       ; If bit 1 is set, display pattern 1
    out PORTC, pattern_1

	stop_button_pressed
    sbrc state, 2                        ; If bit 2 is set, display pattern 2
    out PORTC, pattern_2

	stop_button_pressed
    sbrc state, 3                        ; If bit 3 is set, display pattern 3
    out PORTC, pattern_3

	jmp listener


jmp_to_main:
	jmp setup

continues_pattern:
	stop_button_pressed

    out PORTC, pattern_2
	delay

	stop_button_pressed

    out PORTC, pattern_2
	delay

	stop_button_pressed
	
    out PORTC, pattern_3
	delay

	jmp continues_pattern

end:
	rjmp end