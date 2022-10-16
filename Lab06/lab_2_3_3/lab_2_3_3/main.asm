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


;LED - port C
;push button1 - port D RDX3



.include "m2560def.inc"


.def pattern_1 = r2
.def pattern_2 = r3
.def pattern_3 = r4
.def state = r18
.def bounce_counter = r19
.def button_input = r20
.def delay_counter = r21



.equ F_CPU = 16000000
.equ DELAY_1MS = F_CPU / 4 / 1000 - 4
; 4 cycles per iteration - setup/call-return overhead


C:\Program Files (x86)\Arduino\hardware\tools\avr\bin\avrdude.exe
-C "C:\Program Files (x86)\Arduino\hardware\tools\avr\etc\avrdude.conf" -c wiring -p m2560 -P COM9 -b 115200 -U flash:w:"$(ProjectDir)Debug\$(TargetName).hex":i -D 

; sbis PIND, 0         ; if PIND pin0 = 1, skip 
; sbic PIND, 0         ; if PIND pin0 = 0, skip 



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


.macro button_pressed						; Check if button 1 is pressed
    ldi bounce_counter, 10					; Initialise bounce counter to 10
loop_button:
    rcall sleep_1ms
	sbic PORTD, $0
    ;in button_input, $0						; TODO: Read button 1
    ;sbrc button_input, 0					; TODO: If high voltage (button not pressed), increment counter
    inc bounce_counter

	sbis PORTD, $0
    ;sbrs button_input, 0					; TODO: If low voltage (button pressed), decrement counter
    dec bounce_counter


    cpi bounce_counter, 0					; If bounce_counter reaches zero, move to button_pressed
    breq return_button_pressed


    cpi bounce_counter, 20					; Button pressed if the user pressed the button for (20-10)ms
    breq return_button_not_pressed


    rjmp loop_button


return_button_pressed:						; Button pressed, clear all bits in button_input register
    clr button_input


return_button_not_pressed:					; Button not pressed, set all bits in button_input register
    ser button_input


.endmacro


.macro delay
    ldi delay_counter, 300					; Set delay counter to 300

loop_delay:
    rcall sleep_1ms
    dec delay_counter
    cp delay_counter, 0
    breq end_delay

end_delay:
    nop
.endmacro


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
	ldi state, 1						; reset state to 1 (everything to 0)

    rjmp listener						; Jump to listener

; Assumptions
; 1. If we press the first button more than 4 times (without second button involved), the board won't do anything, it will stay at the last LED pattern
; 2. While the pattern is displaying, we won't interupt the displaying
; 3. TODO: figure out how long we need / can press the button for
listener:
    ldi bounce_counter, 10				; Initialise bounce counter to 10

    button_pressed 0					; TODO: if the first button is pressed
    cpi button_input, 0					; If the first button is bressed
    breq increment_state				; Go to increment_state

	button_pressed 1					; TODO
    cpi button_input, 0					; If the second button is pressed
    breq main							; go to main (which is to reset everything)
	
	rjmp listener


; 00000001 off, initial state	
; 00000010 pattern 1
; 00000100 pattern 2
; 00001000 pattern 3
; 00010000 pattern 4
; 00100000 bit 5 is set, loop back to off
increment_state:
	sbrc state, 4						; if bit 4 is set
	rjmp continues_pattern				; go to continues_pattern
    lsl state                           ; bitshift left state
    rjmp set_pattern					; go to set_pattern

; When first button pressed, we go to this section
; 
set_pattern:
    ;sbrc state, 0                        ; if bit 0 is set, clear the LED
    ;clr r16
	;sbrc state, 0                        ; if bit 0 is set, clear the LED, maybe TODO: figure out a better way to do that
    ;out PORTC, r16


    sbrc state, 1                        ; If bit 1 is set, display pattern 1
    out PORTC, pattern_1

    sbrc state, 2                        ; If bit 2 is set, display pattern 2
    out PORTC, pattern_2

    sbrc state, 3                        ; If bit 3 is set, display pattern 3
    out PORTC, pattern_3

    rjmp listener


continues_pattern:
	button_pressed PORTD				; TODO
    cpi button_input, 0					; If the second button is pressed
    breq main							; go to main (which is to reset everything)

    out PORTC, pattern_1
	delay

	; repeating this to handle the corner case of pressing the button while the 2nd pattern is showing
	button_pressed PORTD				; TODO
    cpi button_input, 0					; If the second button is pressed
    breq main							; go to main (which is to reset everything)

    out PORTC, pattern_2
	delay

	; repeating this to handle the corner case of pressing the button while the 3rd pattern is showing
	button_pressed PORTD				; TODO
    cpi button_input, 0					; If the second button is pressed
    breq main							; go to main (which is to reset everything)

    out PORTC, pattern_3
	delay

	rjmp continues_pattern

