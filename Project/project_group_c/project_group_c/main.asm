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



 /* CONSTANTS */
.set QUEUE_SIZE = 102        ; head + tail + 99 + 1 byte of padding

/* DATA MEMORY */
.dseg
.org 0x200
eastQ:
	.byte QUEUE_SIZE
westQ:
	.byte QUEUE_SIZE
roadQ:												; 10 cars will be availeble for this Q. each car will have 2 bytes, 1 for speed, 1 for position.
	.byte 22
TimerOvFCounter: 
	.byte 1	
global_timer:
	.byte 2								

/* PROGRAM MEMORY */
.cseg
.include "m2560def.inc"

.equ CAR_SPEED_MASK = 0x0F
.def digit_1 = r2
.def digit_2 = r3
.def num_cars = r7									; number of cars on the road
.def row = r8
.def col = r9
.def rmask = r10
.def cmask = r11
.def car_speed = r15								; register for car speed
.def temp1=r16										;
.def temp2=r17
.def temp3=r18
; road status, 8 bits
;		7: West traffic light is on
;		6: East traffic light is on
;		5: TRAFFIC_DIRECTION_WEST_TO_EAST
;		4: Set iff PB1 (West) detected
;		3: Set iff PB0 (East) detected
.def road_status = r19
.def temp4=r20
.def temp5=r21
.def temp6=r22
.def three_seconds_count = r23
.def temp8=r24
.def temp9=r25

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
.equ WEST_LIGHT_ON_MASK = 0b01111111
.equ EAST_LIGHT_ON = 6
.equ EAST_LIGHT_ON_MASK = 0b10111111
.equ TRAFFIC_DIRECTION_WEST_TO_EAST = 5
.equ TRAFFIC_DIRECTION_WEST_TO_EAST_MASK = 0b11011111
.equ EMERGENCY_STATE = 4
.equ EMERGENCY_STATE_MASK = 0b11101111
.equ WEST_CAR_DETECTED = 3
.equ WEST_CAR_DETECTED_MASK = 0b11110111
.equ EAST_CAR_DETECTED = 2
.equ EAST_CAR_DETECTED_MASK = 0b11111011

.equ MAX_CARS_ON_ROAD = 7

.equ ONE_CAR_SYMBOL = 0b00101101
.equ TWO_CAR_SYMBOL = 0b00111101
.equ SPACE = ' '
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

/* TO_DECIMAL */
; decimal to digits
.macro to_decimal
push temp1
	clr digit_1
	clr digit_2

check_ten:
	cpi @0, 10
	brsh over_ten
; The number is a single digit at this point
	mov digit_2, @0
	rjmp end_to_decimal
over_ten:
	subi @0, 10
	inc digit_1
	rjmp check_ten
end_to_decimal:
	ldi temp1, '0'
	add digit_1, temp1
	add digit_2, temp1
	pop temp1
.endmacro

; @0: the register store the pointer
; @1: the number we want to wrap around
.macro wrap_around_pointer
	cpi @0, @1
	brlo end_wrap_around_pointer
	subi @0, @1
end_wrap_around_pointer:
	nop
.endmacro

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

	clr temp4

	st x+, temp4
	st x+, temp4
	st y+, temp4
	st y+, temp4
	st z+, temp4
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

/* ADD CARS TO THE DIRECTION QUEUE */
; @0 is the west/east queue we want to modify
; @1 is the car speed we want to add
.macro add_cars_to_direction_queue
	in temp3, SREG
	push temp3
	push temp4
	push zh
	push zl

    ldi zh, high(@0)
    ldi zl, low(@0)

    ld temp4, z+						; temp4 is the no_of_items in the queue
    ld temp5, z+						; temp5 is the head offset

	add temp4, temp5					; point temp4 to the position to add items
	wrap_around_pointer temp4, 99

    add zl, temp4						; point to the position to add items
    mov temp5, @1						; load car speed to temp 5
	st z, temp5							; store the car speed to one spot after the tail

end_add_cars_to_direction_queue:
; store new no_of_items in the queue
	ldi zh, high(@0)					; increase num_of_items and store it in the queue
    ldi zl, low(@0)
    ld temp4, z
	inc temp4
	st z, temp4
;clean up
	pop zl
	pop zh
	pop temp4
	pop temp3
	out SREG, temp3
    nop
.endmacro
/* END ADD CARS TO THE DIRECTION QUEUE */

/* POP CARS FROM THE QUEUE */
; @0 is the west/east queue we want to pop
; @1 is the register to store the car speed
; return value: the first car speed, it will be stored in temp4
.macro pop_cars_from_direction_queue
	push temp3
	in temp3, SREG
	push temp3
	push temp4
	push temp5
	push zh
	push zl

    ldi zh, high(@0)					; the address of the no_of_items in the queue
    ldi zl, low(@0)

    ld temp3, z+						; temp3 is the no_of_items in the queue
    cpi temp3, 0						; check if there are cars in the queue
    breq end_pop_cars_from_direction_queue
    ld temp5, z+						; temp5 is the head offset

    add zl, temp5						; point to the head
    ld @1, z							; r12 now stores the value of the car we want to pop

	; store new no_of items and head 
	ldi zh, high(@0)
    ldi zl, low(@0)
	ld temp5, z							; temp5 now have the original no_of_items in the queue
	dec temp5							; decrease this value
	st z, temp5							; store the decreased no_of_items in the queue

	adiw z, 1							; increase z address by 1 to point to the head address
	ld temp5, z
	inc temp5
    st z, temp5							; store the new head value there
end_pop_cars_from_direction_queue:
	pop zl
	pop zh
	pop temp5
	pop temp4
	pop temp3
	out SREG, temp3
	pop temp3
    nop
.endmacro
/* ENDPOP CARS FROM THE QUEUE */

/* @0: car speed, put it in road_queue */
.macro add_car_to_road
	push temp3
	in temp3, SREG
	push temp3
	push temp4
	push temp5
	push zh
	push zl

	mov temp4, @0						; get car speed in temp4
	cpi temp4, 10						; check if the speed is lower than 10, if so end
	brlo end_add_car_to_road

	ldi zh, high(roadQ)					; the address of the number_of_items
    ldi zl, low(roadQ)

	ld temp5, z+						; temp5 has the value of no_of_items in this queue		
	ld temp4, z+						; temp4 the value of head, z now points to the first car speed in the queue

	add temp5, temp4					; temp5 = no_of_items + head offset
	wrap_around_pointer temp5, 10		; wrap around

update_car_speed:
	add zl, temp5						; go to the new last position to insert car speed
	mov r14, @0							; load provided speed to r14

	/*
	mov temp6, r14
	to_decimal temp6
	do_lcd_command 0b1100_0100
	do_lcd_data digit_1
	do_lcd_data digit_2*/

	st z, r14							; store the car speed
	clr r14								; store the car position as 0
	std z+10, r14

; update new number_of_items in the queue
	ldi zh, high(roadQ)					; the address of the tail (number)	
    ldi zl, low(roadQ)					; the address of the head (number)
	ld temp5, z							; get the current value
	inc temp5
	st z, temp5							; store temp5 as the new number_of_items value

	inc num_cars						; increment number of cars on the road
end_add_car_to_road:
	pop zl
	pop zh
	pop temp5
	pop temp4
	pop temp3
	out SREG, temp3
	pop temp3
    nop
.endmacro
/* END CAR GOES ON ROAD */


; every one second, update the location of cars on the road
.macro update_car_positions
	push temp3
	in temp3, SREG
	push temp3
	push temp4
	push temp5
	push temp6
	push zh
	push zl

	ldi zh, high(roadQ)					; the address of the tail (number)	
    ldi zl, low(roadQ)					; the address of the head (number)

	ld temp3, z+						; temp3 has the value of number_of_items	
	tst temp3
	breq rjmp_end_update_car_positions
	ld temp3, z+						; temp3 has the value of head, z points to the first car speed in the queue
	;; WS: can just do ld temp3, z right?
	dec temp3							; decrease due to increment in the loop
	add zl, temp3						; go to the car speed we want to iterate

	mov temp5, num_cars					; temp5 stores the value of original num_cars
	
	clr temp3							; clear temp3 to be used as a counter
	ldi temp8, 190						; set temp8 to 190, used to keep track of prevous_car_position
update_car_positions:
	cp temp3, temp5						; comapre counter and orginal num_cars
	brsh rjmp_end_update_car_positions		; if counter >= original num_cars, go to end_update_car_positions
	rjmp continue
rjmp_end_update_car_positions:
	rjmp end_update_car_positions
continue:
	adiw zl, 1							; go to the car speed we want to iterate

	inc temp3							; increase temp3

	ld temp4, z							; load car speed to temp4
	ldd temp9, z+10						; load car position to temp9

	add temp9, temp4					; temp9 += temp4
	cpi temp9, 180						; compare the car position with 50m/(5/18)
	brsh car_out						; if it's larger than 180, it's drove off
	cp temp9, temp8						; compare the current car position with the previous one
	brsh car_crash						; breach to car_crash if current_position >= previous_position
	std z + 10, temp9					; otherwise, store the updated position
	mov temp8, temp9					; update the previous_car position with the current one
	rjmp update_car_positions

; we enter into a car_crash state, we want to set the speed of every car in front of it to be 10, and everything behind it to be 0
car_crash:
	ldi temp4, 0
	st z, temp4							; set the two crashed car speed and position to 0
	std z + 10, temp4
	dec zl
	st z, temp4
	std z + 10, temp4
	ori road_status, (1<<EMERGENCY_STATE)	; set road status to be emergency state
	update_car_speed_before_toll
	rjmp end_update_car_positions

car_out:
	dec num_cars						; decrement num_cars
	clr temp6							; clear the value of temp6, as the next car won't crush with this one
	rjmp update_car_positions

end_update_car_positions:
; update new num_of_items in the queue
	ldi zh, high(roadQ)
	ldi zl, low(roadQ)
	st z, num_cars

	; check if it's emergency state, if it is, go to update_car_speed_after_toll
	;sbrc road_status, EMERGENCY_STATE	; skip next line if emergency state is cleared/off
	;check_car_and_toll

	pop zl
	pop zh
	pop temp6
	pop temp5
	pop temp4
	pop temp3
	out SREG, temp3
	pop temp3
    nop
.endmacro

/* UPDATE CAR SPEED BEFORE TOLL */
.macro update_car_speed_before_toll
	in temp3, SREG
	push temp3
	push zh
	push zl

	ldi zh, high(roadQ)							; the address of the no_of_items (number)	
    ldi zl, low(roadQ)

; Firstly, set every car speed before the first crashed car to be 0
	ld temp8, z+								; temp8 has the value of number_of_items		
	ld temp5, z+								; temp5 has the value of head, z points to the first car speed in the queue

	add zl, temp5								; go to the first car speed

	clr temp2									; clear temp2, use it to record how many cars in front of the crashed cars
	clr temp5									; clear temp5, use it as a counter
	; clear temp6, use it to indicate if we want to set the speed to 10 or 0.
	; 1 means we want to set it to 10, 0 means we want to set it to 0.
	ser temp6
update_car_speed_before_toll_loop:
	cp temp5, temp8								; compare counter and number_of_items
	brsh end_update_car_speed_before_toll		; breach if larger or equal

	tst temp6									; go to set_speed_to_zero if temp6 is 0
	breq set_speed_to_zero
	; otherwise, check if this is the crashed car
	ld temp3, z									; get the current car speed to temp3
	tst temp3									; check if the current car speed is 0
	brne set_speed_to_ten						; if it is not, go to set_speed_to_ten
	clr temp6									; otherwise, clear temp6
	rjmp end_update_car_speed_before_toll_loop
set_speed_to_ten:
	inc temp2									; increment temp2
	ldi temp3, 10								; otherwise, set the current speed to 10
	st z, temp3
	rjmp end_update_car_speed_before_toll_loop
set_speed_to_zero:
	clr temp3									; otherwise, set the current speed to 10
	st z, temp3
	rjmp end_update_car_speed_before_toll_loop
end_update_car_speed_before_toll_loop:
	inc zl										; go to the next car speed
	inc temp5									; increment the counter
	rjmp update_car_speed_before_toll_loop		; jump back to update_car_speed_before_toll_loop

end_update_car_speed_before_toll:
	pop zl
	pop zh
	pop temp3
	out SREG, temp3
.endmacro
/* END UPDATE CAR SPEED BEFORE TOLL */

/* CHECK IF CAR NEED TO  BE TOLLED AND TOLL CARS */
.macro check_car_and_toll
	push temp4
	in temp4, SREG
	push temp2
	push temp3
	push temp4
	push temp5
	push temp6
	push zh
	push zl

	ldi zh, high(roadQ)							; the address of the no_of_items (number)	
    ldi zl, low(roadQ)

	ld temp4, z+								; temp4 has the value of number_of_items		
	ld temp5, z+								; temp5 has the value of head, z points to the first car speed in the queue

	add zl, temp5								; go to the first car speed in the queue
	ld temp4, z									; load this value to temp4

	tst temp4									; check if car speed is 0
	brne end_check_car_and_toll					; if not, go to end_check_car_and_toll
	; TODO: do some animation to show the cars are being tolled
	ldi zh, high(roadQ)							; the address of the no_of_items (number)	
    ldi zl, low(roadQ)
	; toll the first car
	dec num_cars								; decrease the num_cars
	st z, num_cars								; store this value to the queue
	inc temp5									; increase head
	wrap_around_pointer temp5, 10				; wrap around
	std z+1, temp5								; store this value in the queue

	; toll the second car
	dec num_cars								; decrease the num_cars
	st z, num_cars								; store this value to the queue
	inc temp5									; increase head
	wrap_around_pointer temp5, 10				; wrap around
	std z+1, temp5								; store this value in the queue

	update_car_speed_after_toll
end_check_car_and_toll:
	pop zl
	pop zh
	pop temp6
	pop temp5
	pop temp4
	pop temp3
	pop temp2
	out SREG, temp4
	pop temp4
.endmacro
/* END CHECK IF CAR NEED TO  BE TOLLED AND TOLL CARS */

/* UPDATE CAR SPEED AFTER TOLL */
.macro update_car_speed_after_toll
	in temp4, SREG
	push temp4
	push zh
	push zl

	ldi zh, high(roadQ)							; the address of the no_of_items (number)	
    ldi zl, low(roadQ)

; Now the crashed cars have been tolled, set every car speed to be 10
	ld temp8, z+								; temp8 has the value of number_of_items		
	ld temp5, z+								; temp5 has the value of head, z points to the first car speed in the queue

	add zl, temp5								; go to the first car speed

	clr temp5									; clear temp5, use it as a counter
update_car_speed_after_toll_loop:
	cp temp5, temp8								; compare counter and number_of_items
	brsh end_update_car_speed_after_toll		; breach if larger or equal

	ldi temp3, 10								; set the current speed to 10
	st z, temp3

	inc zl										; go to the next car speed
	inc temp5									; increment the counter
	rjmp update_car_speed_after_toll_loop		; jump back to update_car_speed_after_toll_loop

end_update_car_speed_after_toll:
	pop zl
	pop zh
	pop temp4
	out SREG, temp4
.endmacro
/* END UPDATE CAR SPEED AFTER TOLL */

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
	push temp1
	ldi temp1, @0
	call lcd_command
	call lcd_wait
	pop temp1
.endmacro
.macro do_lcd_data
	push temp1
	mov temp1, @0
	call lcd_data
	call lcd_wait
	pop temp1
.endmacro

.macro lcd_set
	sbi PORTA, @0
.endmacro

.macro lcd_clr
	cbi PORTA, @0
.endmacro

lcd_command:
	push temp1
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
	pop temp1
	ret
lcd_data:
	push temp1
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
	pop temp1
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

/* SHOW CAR POSITION ON ROAD */
show_car_position: 
	push temp1
	push temp2
	push temp3
	push temp4
	push temp5
	push temp6
	push three_seconds_count
	; Rationale: 10 positions in increments of 18 from 0 to 180
	; Display number of cars in each bin (0-18, 19-36etc)
	in temp3, SREG
	push temp3

	push zh
	push zl

	ldi zh, high(roadQ)
	ldi zl, low(roadQ)

	mov temp5, zl				; temp5 contains last location in memory (for wrapping around)
	subi temp5, -22					; NEED TO DOUBLE CHECK THIS ISN'T ONE OFF, should be 23?

	ldd temp3, z+1				; temp3 contains head location
	subi temp3, -12				; Offset to get position
	; ZL points to start of queue, add 2 + 10 + head
	add zl, temp3				; zh:zl now points to location of first car location

	ldi temp1, 0				; let temp1 be lower bound
	ldi temp2, 17				; let temp2 be the upper bound

	clr three_seconds_count					; let three_seconds_count be the count of cars in bin
	do_lcd_command 0b11000011		; move cursor to second row position
	ldi temp6, SPACE
	do_lcd_data temp6
	do_lcd_data temp6
	do_lcd_data temp6
	do_lcd_data temp6
	do_lcd_data temp6
	do_lcd_data temp6
	do_lcd_data temp6
	do_lcd_data temp6
	do_lcd_data temp6
	do_lcd_data temp6
	do_lcd_command 0b11000011
	; From now on, temp3 contains location of current car
loop_cars_in_road:
	cpi temp2, 180		; If car past 180, no more cars to print
	brsh no_more_cars_position

	; Move ZH:ZL so it points to first car
	mov zl, temp3
	mov temp4, num_cars
attempt_to_collect_car_position:
	tst temp4
	breq no_more_cars
	cp zl, temp5
	brlo get_car
	subi zl, -10
get_car:
	ld temp6, z+					; load car position into z
	dec temp4
	; Check if car is in bin: if car below temp1 or car position above temp2, skip
	cp temp6, temp1
	brlo not_in_bin
	cp temp2, temp6
	brlo not_in_bin

	; Now, we are in the bin, increment
	inc three_seconds_count

	; For each [0-17], [18-35], ... [,-179] chunk, count number of cars
not_in_bin:
	rjmp attempt_to_collect_car_position

no_more_cars: ; no more cars, increment
;; print
	; Now, car is in next bin
	; Print out current bin
	cpi three_seconds_count, 0
	breq zero_cars
	cpi three_seconds_count, 1
	breq one_car
	cpi three_seconds_count, 2
	breq two_cars
	rjmp more_cars

zero_cars:
	ldi temp6, SPACE
	rjmp display_car

one_car:
	ldi temp6, ONE_CAR_SYMBOL
	rjmp display_car
two_cars:
	ldi temp6, TWO_CAR_SYMBOL
	rjmp display_car
more_cars:
	ldi temp6, 0b11010000
	rjmp display_car

display_car:
	do_lcd_data temp6

	clr three_seconds_count

	;; Move to the next bin
	 subi temp1, -18
	subi temp2, -18
	rjmp loop_cars_in_road

no_more_cars_position:
	do_lcd_command 0b11111111						; get rid of cursor
	pop zl
	pop zh
	pop temp3
	out SREG, temp3
	pop three_seconds_count
	pop temp6
	pop temp5
	pop temp4
	pop temp3
	pop temp2
	pop temp1
	nop
ret

/* END SHOW CAR POSITION ON ROAD*/
/* LED MACRO */
.macro refresh_lights
	push temp1
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
	call sleep_100ms
	call sleep_100ms
	clr temp1
	out portc, temp1


refresh_lights_end:
	pop temp1
.endmacro

.macro display_car_symbols
push temp1
push temp2
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
	pop temp2
	pop temp1
.endmacro

/* DISPLAY MACRO */
.macro refresh_display_top
	push temp1
	in temp1, SREG
	push temp1	
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

	pop temp1
	out SREG, temp1
	pop temp1
.endmacro

/* Refresh second row for input */
.macro refresh_bottom
	push temp1
	in temp1, SREG
	push temp1
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
	pop temp1
	out SREG, temp1
	pop temp1
.endmacro

/* MAIN FUNCTIONALITY */
RESET:
/*
	; KAT TEST road
	initialise_queues
	ldi temp2, 30
	add_cars_to_direction_queue westQ, temp2
	pop_cars_from_direction_queue westQ
	add_car_to_road r12
	ldi temp2, 20
	add_car_to_road temp2
	ldi temp2, 35
	add_car_to_road temp2
	ldi temp2, 15
	add_car_to_road temp2
	update_car_positions
	update_car_positions
	update_car_positions
	update_car_positions
	update_car_positions
	update_car_positions
	update_car_positions
	update_car_positions
	update_car_positions
	*/
		

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

	clear global_timer
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



	;add_car_to_road 15
	;update_car_positions
	;rcall show_car_position


	sei									; set interrupt flag
	initialise_queues



	jmp check_keypad

	; Set traffic light from West to East on


/* PB1 (WEST) DETECTED EXT_INT0 */
; Set bit 6 in register road_status
EXT_INT0:
	push temp1
	in temp1, SREG
	push temp1									
	; testing
	do_lcd_command 0b11000000							; set the location of the cursor to be at the beginning of the second line
	ldi temp1, 'W'										; for testing purpose
	do_lcd_data temp1
	do_lcd_command 0b11001111							; remove east
	ldi temp1, ' '
	do_lcd_data temp1

	; which queue to go to depends on the road_status
	ori road_status, (1<<WEST_CAR_DETECTED)				
	andi road_status, EAST_CAR_DETECTED_MASK
	pop temp1
	out SREG, temp1
	pop temp1
	reti

/* PB0 (EAST) DETECTED EXT_INT1 */
; Set bit 5 in register road_status
EXT_INT1:
	push temp1
	in temp1, SREG
	push temp1	
	; testing
	do_lcd_command 0b11000000 ; remove W if necessary
	ldi temp1, ' '
	do_lcd_data temp1
	do_lcd_command 0b11001111
	ldi temp1, 'E'
	do_lcd_data temp1

	ori road_status, (1<<EAST_CAR_DETECTED)
	
	; Clear anyh west car detected
	andi road_status, WEST_CAR_DETECTED_MASK
	pop temp1
	out SREG, temp1
	pop temp1
	reti

/* INTERNAL INTERRUPT TRIGGERED */
; This can be triggered under two condition:
; 1. We press the * button
; 2. Two cars have collided with each other
EXT_INT7:
	push temp1
	in temp1, SREG
	push temp1
	sbi porte, INTERRUPT_BIT
	rcall sleep_100ms
	rcall sleep_100ms
	; If emergency state is on, go out of emergency state 
	sbrc road_status, EMERGENCY_STATE
	rjmp exit_emergency
	do_lcd_command 0b11000000
	ldi temp1, 'E'
	do_lcd_data temp1
	ldi temp1, 'S'
	do_lcd_data temp1

	; Otherwise, set road status to be emergency state
	ori road_status, (1<<EMERGENCY_STATE)

	rjmp end_emergency
exit_emergency:
	do_lcd_command 0b11000000
	ldi temp1, 'E'
	do_lcd_data temp1
	ldi temp1, 'E'
	do_lcd_data temp1
	andi road_status, EMERGENCY_STATE_MASK

	rjmp end_emergency
end_emergency:
	; Reset interrupt bit 1
	pop temp1
	out SREG, temp1
	pop temp1
	nop
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
	call sleep_20ms
	call sleep_20ms
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
	ldi temp1, 10
	add temp1, car_speed
	add_cars_to_direction_queue westQ, temp1
	andi road_status, WEST_CAR_DETECTED_MASK
	
	jmp check_keypad
east_car_keypad:
	; for debugging
	;do_lcd_command 0b11000000
	;ldi temp1, 'e'
	;do_lcd_data temp1
	refresh_bottom
	ldi temp1, 10
	add temp1, car_speed
	add_cars_to_direction_queue eastQ, temp1
	andi road_status, EAST_CAR_DETECTED_MASK
	jmp check_keypad

symbols:
	mov temp1, col
	cpi temp1, 0							; If col is 0 we have star
	brne check_keypad_jmp_2
	cbi porte, INTERRUPT_BIT				; * detected, clear interrupt bit and generate a falling edge
	; for debugging

	jmp check_keypad

check_keypad_jmp_2:
	jmp check_keypad

/* TIMER OVERFLOW FOR BEHAVIOUR EVERY SECOND */
Timer0OVF: 
	push temp1
	in temp1, SREG
	push temp1
	push YL
	push YH
	push zl
	push zh
	push r22
	push r24
	push r25

	ldi YL, low(TimerOvFCounter)
	ldi YH, high(TimerOvFCounter)

	ld r22, Y
	inc r22

	cpi r22, 61
	brne not_second_jmp
	jmp refresh	
not_second_jmp:
	jmp not_second
refresh:													; one second has passed
	ldi zl, low(global_timer)
	ldi zh, high(global_timer)
	ld r24, Z+
	ld r25, Z
	adiw r25:r24, 1
	st Z, r25
	st -Z, r24

	clear TimerOvFCounter

	update_car_positions
	refresh_display_top
	refresh_lights
	;rcall show_car_position

	; check if 3 seconds has passed
	cpi three_seconds_count, 3								; check three_seconds_count
	brsh send_cars_to_road
	inc three_seconds_count
	jmp end_timer

	; 1. Set TRAFFIC_DIRECTION_WEST_TO_EAST (and related) to true at proper time
	; 2. Set WEST_LIGHT_ON to true/false at proper time
	; if TRAFFIC_DIRECTION_WEST_TO_EAST enabled
	; if WEST_LIGHT_ON enabled
	; then we send west cars to road
send_cars_to_road:
	clr three_seconds_count									; if 3 seconds have passed, clear three_seconds_count
	clr r12													; set r12 to be 0
	pop_cars_from_direction_queue westQ, r12

	tst r12													; if r12 is not 0, add car to road
	brne r12_not_zero
	jmp end_timer
r12_not_zero:
	add_car_to_road r12										; othersie, add the car to the road
not_second:
	st Y, r22
	rjmp end_timer
end_timer:
	pop r25
	pop r24
	pop r22
	pop zh
	pop zl
	pop YH
	pop YL
	pop temp1
	out SREG, temp1
	pop temp1
	reti


;1. turn on/off traffic light
;2. sending cars to the road

; << ooo== 1
; << oo==o 2
; << o==oo 3
; << ==ooo 4
; << =oooo 5