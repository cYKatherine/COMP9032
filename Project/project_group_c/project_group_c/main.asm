/* PORT CONFIGURATION
 *  LCD: Data PortF D0:PF0 --> D7:PF7
 *  LCD: Control BE:PA4 --> ZBL:PA7 
 *  Keypad: C3:PL0, R3:PL4
 *  PB0 (East): SDA (actually RDX3)
 *  PB1 (West): SCL (Actually RDX4)
 *  Internal interrupt: RDX1 (Actually TDX2)
 *  LED: Port C
 END PORT CONFIGURATION */

/* CURRENT STATE OF AFFAIRS
The internal interrupt flag isn't working, when activated it will keep triggering
During an emergency, the lights flash on and off too fast. I expected it to come on
then turn off quickly, for the majority of the second. (lights disabled for now)

One can press PB1 and a number 1-6 to show a placeholder W10 on the screen
One can press PB0 and a number 1-6 to show a placeholder E15 on the screen

One can press * to show * in the corner
*/

/* REGISTERS: (please feel free to change these if they don't make sense)
 *  r7 (FOR NOW) will contain number of cars on road (should be calculated elsewhere later)
 *  r15 will contain some info about # cars on road, speed of car to be added:
 *		3-0: speed of car to be added
 will contain the speed from 0-5 of the car to be added
 *	r16, r17, r18 are temp1 and temp22, don't leave important things here
 *	will need to check that interrupts and temp registers don't interfere
 *	r8, r9, r10, r11: row, col, rmask, cmask
 *  r19 will be road status:
 *		7: Set if traffic light from W to E on
 *		6: Set if traffic light from E to W on
 *		5: Set if emergency state activated
 *		4: Set iff PB1 (West) detected
 *		3: Set iff PB0 (East) detected
 * Open to any ideas about storing number of cars on the road
 END REGISTERS */


/* DATA STRUCTURES (TODO)
 *	Expect: two arrays in memory, one for EAST and one for WEST queue
 *	Two words in each array will be reserved for EAST_HEAD and EAST_TAIL pointers
 *	One byte will be reserved for queue length
END DATA STRUCTURES */

/* ADD_TO_QUEUE (TODO)
 *	Called when: car and speed detected
 *	TAIL: is the empty space for next car
 *	HEAD is location of current car, or if no cars, equal to TAIL
 *
 *	Insert speed into tail location
 *	Increment tail to point to next location
 *	Increment queue length
 *	Check new tail location of queue
 *	If tail location >= 100 do (check wraparound logic)
 *    set tail location = 0
 *	endif
END ADD_TO_QUEUE */

/* POP_FROM_QUEUE (TODO)
 *	Called when: HEAD != TAIL detected
 *
 *	SEND CAR ON ROAD
 *	Increment head to point to next location
 *	Decrement queue length
 *	Check new head location of queue
 *	If head location >= 100 do
 *	set tail location = 0
 *	endif
 END POP_FROM_QUEUE */

 /* CONSTANTS */
.set QUEUE_SIZE = 102        ; head + tail + 99 + 1 byte of padding

/* DATA MEMORY */
.dseg
.org 0x200
roadQ:												; 10 cars will be availeble for this Q. each car will have 2 bytes, 1 for speed, 1 for position.
	.byte 22
TimerOvFCounter: 
	.byte 1
eastQ:
	.byte QUEUE_SIZE
westQ:
	.byte QUEUE_SIZE									

/* PROGRAM MEMORY */
.cseg
.include "m2560def.inc"

.equ CAR_SPEED_MASK = 0x0F
.def car_speed = r15								; register for car speed
.def temp1=r16										;
.def temp2=r17
.def temp3=r18
.def temp4=r20
.def temp5=r21
.def num_cars = r7									; number of cars on the road

.def row = r8
.def col = r9
.def rmask = r10
.def cmask = r11

; road status, 8 bits
;		5: Set if emergency state activated
;		4: Set iff PB1 (West) detected
;		3: Set iff PB0 (East) detected
.def road_status = r19
  
.equ LCD_RS = 7
.equ LCD_E = 6
.equ LCD_RW = 5
.equ LCD_BE = 4

.equ INTERRUPT_BIT = 7								; output for internal interrupt, input for else
.equ KEYPAD_PORTDIR = 0xF0				
.equ INITCOLMASK = 0xEF					
.equ INITROWMASK = 0x01					
.equ ROWMASK = 0x0F	

; Patterns for LED
.equ WEST_LIGHT = 0xC0
.equ EAST_LIGHT = 0x03
.equ EMERGENCY_LIGHT = 0x18

; bitmask
.equ WEST_LIGHT_ON = 7
.equ EAST_LIGHT_ON = 6
.equ EMERGENCY_STATE = 5
.equ WEST_CAR_DETECTED = 4
.equ EAST_CAR_DETECTED = 3

.equ MAX_CARS_ON_ROAD = 7
/* END CONSTANTS */

/* PROGRAM MEMORY/INTERRUPT LOCATIONS */
jmp RESET
.org INT0addr									; PB1, once pressed, go into interrupt
	jmp EXT_INT0
.org INT1addr									; PB0
	jmp EXT_INT1
.org INT7addr									; internal interrupt. (if we press * or some car triggers the emengency state)
	jmp EXT_INT7
.org OVF0addr
	jmp Timer0OVF
/* END PROGRAM MEMORY LOCATIONS */


/* INITIALISE THE QUEUES */
.macro initialise_queues
	in temp4, SREG
	push temp4
	push yh
	push yl
	push xh
	push xl
	push zh
	push zl

	ldi yh, high(eastQ)
    ldi yl, low(eastQ)

    ldi xh, high(westQ)
    ldi xl, low(westQ)

    ldi zh, high(roadQ)
    ldi zl, low(roadQ)
	ldi temp4, 9
	st z+, temp4

    clr temp4

    st y+, temp4
    st x+, temp4
    st y+, temp4
    st x+, temp4
    st z+, temp4

	pop zl
	pop zh
	pop xl
	pop xh
	pop yl
	pop yh
	pop temp4
	out SREG, temp4
.endmacro
/* END INITIALISE THE QUEUES */

/* @0: car speed, put it in road_queue */
.macro add_car_to_road
	in temp3, SREG
	push temp3
	push zh
	push zl

	ldi zh, high(roadQ)					; the address of the tail (number)	
    ldi zl, low(roadQ)					; the address of the head (number)

	ld temp5, z+						; temp5 has the value of tail		
	ld temp4, z+						; temp4 the value of head, z points to the first car speed in the queue

	inc temp5							; set tail offset to be one after the current last
	cpi temp5, 10						; comapre temp5 with 10
	brlo update_car_speed				; if lower, safe, update, car speed
	subi temp5, 10						; otherwise, wrap around
update_car_speed:
	add zl, temp5						; go to the new last position to insert car speed
	ldi temp4, @0						; load provided speed to temp4

	st z, temp4							; store the car speed
	clr temp4							; store the car position as 0
	std z+10, temp4

update_new_tail:
	ldi zh, high(roadQ)					; the address of the tail (number)	
    ldi zl, low(roadQ)					; the address of the head (number)
	st z, temp5							; store temp5 as the new tail value

	inc num_cars						; increment number of cars on the road
end_add_car_to_road:
	pop zl
	pop zh
	pop temp3
	out SREG, temp3
    nop
.endmacro
/* END CAR GOES ON ROAD */

/* REMOVE CARS FROM THE QUEUE
; @0 is the west/east queue we want to modify
.macro remove_from_direction_queue
	in temp1, SREG
	push temp1
	push zh
	push zl

    ldi zh, high(@0)					; the address of the tail (number)	
    ldi zl, low(@0)						; the address of the head (number)

    ld temp4, z+						; temp4 is the tail (where to insert thing)
    ld temp5, z+						; temp5 is the head
    cp temp4, temp5						; temp4 and temp5 both points to the same one (head and tail)
    breq no_cars

    add zl, temp5						; point to the head
    ld temp4, z

    ldi zh, high(@0)
    ldi zl, low(@0)

    ld temp5, z+
    ld temp5, z							; get the appropriate value
    inc temp5
    cpi temp5, 100						; compare the number of cars with 100
    brsh set_to_zero
    st z, temp5
    rjmp fin
    
    no_cars:
        clr temp4
        rjmp fin

    set_to_zero:						; if there are more than 100
        ldi temp5, 0
        st z, temp5
        rjmp fin

    fin:
		pop zl
		pop zh
		pop temp1
		out SREG, temp1
        nop
.endmacro
**/


; every one second, update the location of cars on the road
.macro update_car_positions
	in temp1, SREG
	push temp1
	push zh
	push zl

	ldi zh, high(roadQ)					; the address of the tail (number)	
    ldi zl, low(roadQ)					; the address of the head (number)

	ld temp3, z+						; temp3 has the value of tail		
	ld temp3, z+						; temp3 has the value of head, z points to the first car speed in the queue
	dec temp3							; decrease due to increment in the loop
	add zl, temp3						; go to the car speed we want to iterate

	mov temp5, num_cars					; temp5 stores the value of original num_cars
	
	clr temp3							; clear temp3 to be used as a counter
update_car_positions:
	cp temp3, temp5						; comapre counter and orginal num_cars
	brsh update_new_head_value			; if counter >= original num_cars, go to update_new_head_value
	adiw zl, 1							; go to the car speed we want to iterate

	inc temp3							; increase temp3

	ld temp2, z							; load car speed to temp2
	ldd temp4, z+10						; load car position to z+10

	add temp4, temp2					; temp4 += temp2
	cpi temp4, 180						; compare the car position with 50m/(5/18)
	brsh car_out						; if it's larger than 180, it's drove off
	std z + 10, temp4					; otherwise, store the updated position
	rjmp update_car_positions

car_out:
	dec num_cars						; decrement num_cars
	rjmp update_car_positions

update_new_head_value:
	sub temp5, num_cars					; temp5 now stores how many cars have drove off
	
	ldi zh, high(roadQ)					; the address of the tail (number)	
    ldi zl, low(roadQ)					; the address of the head (number)
	; update new head value
	adiw z, 1							; increment z
	ld temp4, z							; get the head offset we need to store the car to
	add temp4, temp5					; increase the current head value with the # of cars that have drove off
	cpi temp4, 10						; check if it's over 10
	brsh wrap_around_offset				; if it is, wrap around
	rjmp store_head						; otherwise, store head
wrap_around_offset:
	subi temp4, 10
	rjmp store_head
store_head:
	st z, temp4							; store the new head value to the queue

end_update_car_positions:
	pop zl
	pop zh
	pop temp1
	out SREG, temp1
    nop
.endmacro


.macro display_road
	clr temp4
	cp, temp4, num_cars
	brsh end_display_road
	inc temp4

display_road:
	nop
.endmacro

/* CLEAR MEMORY MACRO */
.macro clear
	ldi ZL, low(@0)
	ldi ZH, high(@0)
	clr temp1
	st Z+, temp1
	st Z, temp1
.endmacro
/* LCD MACROS AND FUNCTIONS */
.macro do_lcd_command
	ldi temp1, @0
	rcall lcd_command
	rcall lcd_wait
.endmacro
.macro do_lcd_data
	mov temp1, @0
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
	out PORTF, temp1
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
	out PORTF, temp1
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
	push temp1
	clr temp1
	out DDRF, temp1
	out PORTF, temp1
	lcd_set LCD_RW
lcd_wait_loop:
	nop
	lcd_set LCD_E
	nop
	nop
    nop
	in temp1, PINF
	lcd_clr LCD_E
	sbrc temp1, 7
	rjmp lcd_wait_loop
	lcd_clr LCD_RW
	ser temp1
	out DDRF, temp1
	pop temp1
	ret

.equ F_CPU = 16000000
.equ DELAY_1MS = F_CPU / 4 / 1000 - 4

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

sleep_20ms:
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	ret

sleep_100ms:
	rcall sleep_20ms
	rcall sleep_20ms
	rcall sleep_20ms
	rcall sleep_20ms
	rcall sleep_20ms
	ret 
/* END LCD MACRO AND FUNCTIONS */

/* WAIT */
wait:
	push temp1
	in temp1, sreg
	push temp1

	ldi temp1, 15
wait_loop:
	dec temp1
	tst temp1
	breq wait_end
	rcall sleep_1ms

	rjmp wait_loop
wait_end:
	pop temp1
	out sreg, temp1
	pop temp1
	ret

/* LED MACRO */
.macro refresh_lights
	; If west to east, show lights
	sbrc road_status, EMERGENCY_STATE
	rjmp emergency_led

	sbrc road_status, WEST_LIGHT_ON
	ldi temp1, WEST_LIGHT

	sbrc road_status, EAST_LIGHT_ON
	ldi temp1, EAST_LIGHT

	;ldi temp1, $00
	out portc, temp1
	jmp refresh_lights_end
emergency_led:
	ldi temp1, EMERGENCY_LIGHT
	out portc, temp1
	rcall sleep_100ms
	rcall sleep_100ms
	clr temp1
	out portc, temp1


refresh_lights_end:
	nop
.endmacro

.macro display_car_symbols
print_cars:
	mov temp2, @0

equals_loop:
	tst temp2		; if num cars on road != 0, put one on road
	breq display_car_end

	ldi temp1, '='
	do_lcd_data temp1

	dec temp2
	rjmp equals_loop
display_car_end:
	nop
.endmacro

.macro display_car_padding
pad_spaces:
	; pad cars on road
	ldi temp2, MAX_CARS_ON_ROAD
	mov temp1, @0
	sub temp2, temp1

spaces_loop:
	tst temp2
	breq display_padding_end

	ldi temp1, ' '
	do_lcd_data temp1
	dec temp2

	rjmp spaces_loop
display_padding_end:
	nop
.endmacro

/* DISPLAY MACRO */
.macro refresh_display_top
	do_lcd_command 0b10000000			; move to first row
	; print number of cars in west queue TODO
	ldi temp1, '0'
	do_lcd_data temp1
	do_lcd_data temp1

	; Print space
	ldi temp1, ' '
	do_lcd_data temp1


	; print cars and direction
	sbrc road_status, WEST_LIGHT_ON
	rjmp west_car_pattern
	
	; we have east casrs:
	ldi temp1, '<'
	do_lcd_data temp1
	do_lcd_data temp1
	ldi temp1, ' '
	do_lcd_data temp1
	display_car_symbols num_cars	
	display_car_padding num_cars
	rjmp no_more_cars

west_car_pattern:

	; print number of cars

	display_car_padding num_cars
	display_car_symbols num_cars	
	; print space
	ldi temp1, ' '
	do_lcd_data temp1
	ldi temp1, '>'

	do_lcd_data temp1
	do_lcd_data temp1



no_more_cars:
	; print space
	ldi temp1, ' '
	do_lcd_data temp1


	; print queue size east TODO
	ldi temp1, '0'
	do_lcd_data temp1
	do_lcd_data temp1


.endmacro


/* Refresh second row for input */
.macro refresh_bottom
	do_lcd_command 0b11000000						; move cursor to second row
	sbrs road_status, WEST_CAR_DETECTED
	rjmp east_car
	ldi temp1, 'W'
	do_lcd_data temp1
	; we have a west car
	ldi temp1, '1'
	do_lcd_data temp1
	mov temp1, car_speed
	subi temp1, -'0'
	do_lcd_data temp1

	do_lcd_command 0b11001101						; move to char 14 on line?
	ldi temp1, ' '
	do_lcd_data temp1
	do_lcd_data temp1
	do_lcd_data temp1

	rjmp end_refresh_bottom
east_car:
	ldi temp1, ' '
	do_lcd_data temp1
	do_lcd_data temp1
	do_lcd_data temp1

	do_lcd_command 0b11001101 ; move to char 14 on line?
	ldi temp1, '1'
	do_lcd_data temp1
	mov temp1, car_speed
	subi temp1, -'0'
	do_lcd_data temp1


end_refresh_bottom:
	nop
.endmacro

/* MAIN FUNCTIONALITY */
RESET:
	; KAT TEST road
	;initialise_queues
	;add_car_to_road 10
	;add_car_to_road 11
	;add_car_to_road 10

	;update_car_positions
	; KAT TEST

	ldi temp1, low(RAMEND)
	out SPL, temp1
	ldi temp1, high(RAMEND)
	out SPH, temp1

	;clr temp1							; set portd for input
	;out ddrd, temp1
	;ser temp1
	;out portd, temp1

	ser temp1
	out DDRF, temp1
	out DDRA, temp1
	clr temp1
	out PORTF, temp1
	out PORTA, temp1

	ser temp1							; PORTC is outputs for LED
	out DDRC, temp1	

	ldi temp1, KEYPAD_PORTDIR			; Port D columns are outputs, rows are inputs
	sts	DDRL, temp1			

	; Set direction to west
	;sbr road_status, TRAFFIC_DIR_WEST_TO_EAST

	ldi road_status, (1<<EAST_LIGHT_ON) 
	;ldi temp1, 5			; 3 cars on road
	clr temp1
	mov num_cars, temp1

	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_5ms
	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_1ms
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00001000 ; display off
	do_lcd_command 0b00000001 ; clear display
	do_lcd_command 0b00000110 ; increment, no display shift
	do_lcd_command 0b00001110 ; Cursor on, bar, no blink


	; Set up clock
	; clear clock counter
	clr temp1
	out TCCR0A, temp1
	ldi temp1, 0b00000101 ; prescaler
	out TCCR0B, temp1
	ldi temp1, 1<<TOIE0
	sts TIMSK0, temp1

	; Internal interrupt
	sbi DDRE, INTERRUPT_BIT							; set to output
	sbi porte, INTERRUPT_BIT						; output 1 to byte. at the beginning, set interrupt_bit to be 1, so that when we clear the bit, it tiggers a falling edge.


	; Want PB0, PB1 to be triggered on falling edge, ISC01, ISC11
	ldi temp1, (1 << ISC01)|(1<<ISC11)
	sts EICRA, temp1


	ldi temp1, (1<<ISC71)
	sts EICRB, temp1

	ldi temp1, (1 << INT0) | (1 << INT1) | (1 << INT7)
	out EIMSK, temp1

	initialise_queues
	add_car_to_road 10
	add_car_to_road 11
	add_car_to_road 10

	sei									; set interrupt flag
	jmp check_keypad

	; Set traffic light from West to East on


/* PB1 (WEST) DETECTED EXT_INT0 */
; Set bit 6 in register road_status
EXT_INT0:											
	; testing
	do_lcd_command 0b11000000							; set the location of the cursor to be at the beginning of the second line
	ldi temp1, 'W'										; for testing purpose
	do_lcd_data temp1
	do_lcd_command 0b11001111							; remove east
	ldi temp1, ' '
	do_lcd_data temp1

	; which queue to go to depends on the road_status
	ori road_status, (1<<WEST_CAR_DETECTED)				
	ldi temp1, (1<<EAST_CAR_DETECTED)
	com temp1
	and road_status, temp1
	reti

/* PB0 (EAST) DETECTED EXT_INT1 */
; Set bit 5 in register road_status
EXT_INT1:
	; testing
	do_lcd_command 0b11000000 ; remove W if necessary
	ldi temp1, ' '
	do_lcd_data temp1
	do_lcd_command 0b11001111
	ldi temp1, 'E'
	do_lcd_data temp1

	ori road_status, (1<<EAST_CAR_DETECTED)
	
	; Clear anyh west car detected
	ldi temp1, (1<<WEST_CAR_DETECTED)
	com temp1
	and road_status, temp1
	reti

/* INTERNAL INTERRUPT TRIGGERED */
; This can be triggered under two condition:
; 1. We press the * button
; 2. Two cars have collided with each other
EXT_INT7:
	; If emergency state is on, go out of emergency state 
	sbrc road_status, EMERGENCY_STATE
	jmp RESET

	; Otherwise, set road status to be emergency state
	ori road_status, (1<<EMERGENCY_STATE)

	; Reset interrupt bit 1
	sbi porte, INTERRUPT_BIT
	reti

/* CHECK_KEYPAD */
; This is the background behaviour
check_keypad:
	ldi temp2, INITCOLMASK
	mov cmask, temp2
	clr	col						
colloop:						
	mov temp2, col
	cpi temp2, 4
	breq check_keypad_jmp_1
	sts	PORTL, cmask			
	ldi temp2, 0xFF
	rjmp delay
check_keypad_jmp_1:
	jmp check_keypad
delay:
	dec temp2
	brne delay

	lds	temp2, PINL				
	andi temp2, ROWMASK
	cpi temp2, 0xF			
	breq nextcol
								
	ldi temp2, INITROWMASK		
	mov rmask, temp2
	clr	row						
rowloop:
	mov temp2, row
	cpi temp2, 4					
	breq nextcol
	lds	temp2, PINL	
	andi temp2, ROWMASK
	mov temp3, temp2			
	and temp3, rmask			
	breq convert 				
	inc row						
	lsl rmask					
	jmp rowloop

nextcol:
	lsl cmask				
	inc col						
	jmp colloop					

check_keypad_jmp:
	jmp check_keypad
symbols_jmp:

	jmp symbols
convert:
	rcall sleep_20ms
	mov temp2, col
	cpi temp2, 3					; if column is 3 we have a letter, ignore
	breq check_keypad_jmp
	mov temp2, row							
	cpi temp2, 3					; if row is 3 we have a symbol or 0
	breq symbols_jmp


	; We have a number in 1-9, start filling the "buffer"
	mov temp2, row					; otherwise we have a number in 1-9
	lsl temp2						; temp 1 = 2 * row + row = row * 3
	add temp2, row				
	add temp2, col					; add the column address to get the value
	;inc temp2	
	
	;temp2 contains 0-5 (corresponding to pressing 1-6)

	
	cpi temp2, 6					; if greater than or equal to 6, and repeat
	brsh check_keypad_jmp

	mov car_speed, temp2			; assign the keypad value to car_speed

	sbrc road_status, WEST_CAR_DETECTED			; If bit 6 (WEST_CAR_DETECTED) set, detected car from West
	jmp west_car_keypad

	sbrc road_status, EAST_CAR_DETECTED			; If bit 5 (EAST_CAR_DETECTED) set, detected car from East
	jmp east_car_keypad

	jmp check_keypad				; Otherwise, loop again
west_car_keypad:
	; testing
	;do_lcd_command 0b11000000
	;ldi temp1, 'w'
	;do_lcd_data temp1

	refresh_bottom
	; clear off bit
	ldi temp1, (1<<WEST_CAR_DETECTED)
	com temp1
	and road_status, temp1
	

	jmp check_keypad
east_car_keypad:
	; for debugging
	;do_lcd_command 0b11000000
	;ldi temp1, 'e'
	;do_lcd_data temp1
	refresh_bottom
	ldi temp1, (1<<EAST_CAR_DETECTED)
	com temp1
	and road_status, temp1
	jmp check_keypad

symbols:
	rcall sleep_20ms
	mov temp1, col
	cpi temp1, 0							; If col is 0 we have star
	brne check_keypad_jmp_2

	cbi porte, INTERRUPT_BIT				; * detected, clear interrupt bit and generate a falling edge
	
	; for debugging
	do_lcd_command 0b11000000
	ldi temp1, '*'
	do_lcd_data temp1
	jmp check_keypad

check_keypad_jmp_2:
	jmp check_keypad

/* TIMER OVERFLOW FOR BEHAVIOUR EVERY SECOND */
Timer0OVF: 
	in temp1, SREG
	push temp1
	push YL
	push YH

	push r24

	ldi YL, low(TimerOvFCounter)
	ldi YH, high(TimerOvFCounter)

	ld r24, Y
	inc r24

	cpi r24, 61
	brne not_second_jmp
	jmp refresh	
not_second_jmp:
	jmp not_second
refresh:
	clear TimerOvFCounter

	; check if it is emergency
	update_car_positions
	refresh_display_top
	refresh_lights
	jmp end_timer

not_second:
	st Y, r24
	rjmp end_timer
end_timer:
	pop r24
	pop YH
	pop YL
	pop temp1
	out SREG, temp1
	reti

; << ooo== 1
; << oo==o 2
; << o==oo 3
; << ==ooo 4
; << =oooo 5