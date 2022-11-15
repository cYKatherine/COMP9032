/* PORT CONFIGURATION
 *  LCD: Data PortF D0:PF0 --> D7:PF7
 *  LCD: Control BE:PA4 --> ZBL:PA7 
 *  Keypad: C3:PL0, R3:PL4
 *  PB0 (East): SDA (actually RDX3)
 *  PB1 (West): SCL (Actually RDX4)
 *  Internal interrupt: RDX1 (Actually TDX2)
 *  LED: Port C
 END PORT CONFIGURATION */



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
.def temp6=r22
.def temp7=r23
.def temp8=r24
.def temp9=r25
.def new_speed=r13
.def timer_overflow_count = r4
.def three_seconds_count = r5
.def three_minutes_count = r6
.def num_cars = r7									; number of cars on the road

.def digit_1 = r2
.def digit_2 = r3
.def digit_3=r12
.def row = r8
.def col = r9
.def rmask = r10
.def cmask = r11

; road status, 8 bits
;		7: West traffic light is on
;		6: East traffic light is on
;		5: TRAFFIC_DIRECTION_WEST_TO_EAST
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
.equ LIGHTS_OFF = 0
.equ EAST_LIGHT = 0x03
.equ EMERGENCY_LIGHT = 0x18

; bitmask
; decimal to digits
.macro to_decimal_three
	push temp5
	mov temp5, @0
	clr digit_1
	clr digit_2
	clr digit_3
check_hundred:
	cpi temp5, 100
	brsh over_hundred
check_ten:
	cpi temp5, 10
	brsh over_ten
; The number is a single digit at this point
	mov digit_3, temp5
	rjmp end_to_decimal
over_hundred:
	subi temp5, 100 
	inc digit_1
	rjmp check_hundred
over_ten:
	subi temp5, 10
	inc digit_2
	rjmp check_ten
end_to_decimal:
	ldi temp1, '0'
	add digit_1, temp1
	add digit_2, temp1
	add digit_3, temp1
	pop temp5
	nop	
.endmacro

; 0: WEST TO EAST, GREEN
; 1: WEST TO EAST, RED
; 2: EAST TO WEST, GREEN
; 3: EAST TO WEST, RED
.equ TRAFFIC_STATE_BIT_MASK = 0x0F
.equ EMERGENCY_STATE = 7
.equ EMERGENCY_STATE_MASK = 0b0111_1111
.equ CAR_DETECTED = 6
.equ CAR_DETECTED_MASK = 0b1011_1111
;.equ EAST_CAR_DETECTED = 5
;.equ EAST_CAR_DETECTED_MASK = 0b1101_1111
.equ EAST_TO_WEST_DIRECTION = 1
.equ EAST_TO_WEST_DIRECTION_MASK = 0b1111_1101
.equ RED_LIGHT = 0
.equ RED_LIGHT_MASK = 0b1111_1110

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

; @0: the register store the pointer
; @1: the number we want to wrap around
.macro wrap_around_pointer
	cpi @0, @1
	brlo end_wrap_around_pointer
	subi @0, @1
end_wrap_around_pointer:
	nop
.endmacro


/* TO_DECIMAL */
; decimal to digits
; DON"T SAVE IN TEMP%
.macro to_decimal
push temp5
mov temp5,  @0

	clr digit_1
	clr digit_2

check_ten:
	cpi temp5, 10
	brsh over_ten
; The number is a single digit at this point
	mov digit_2, temp5
	rjmp end_to_decimal
over_ten:
	subi temp5, 10
	inc digit_1
	rjmp check_ten
end_to_decimal:
	ldi temp5, '0'
	add digit_1, temp5
	add digit_2, temp5
	pop temp5
.endmacro



/* SEND CAR TO ROAD */
.macro try_send_car
	push zl
	push zh
	push temp3
	push temp4
	push temp9

	; if emergency state ignore
	sbrc road_status, EMERGENCY_STATE    ; skip next line if emergency state is cleared/off
    jmp end_try_send_car

	; if red light ignore
	sbrc road_status, RED_LIGHT
	rjmp end_try_send_car

	; if < 3 seconds ignore
	mov temp3, three_seconds_count
	cpi temp3, 3
	brlo end_try_send_car_jmp
	rjmp check_direction
end_try_send_car_jmp:
	jmp end_try_send_car

check_direction:
	; check direction
	sbrc road_status, EAST_TO_WEST_DIRECTION
	rjmp send_car_from_east
	rjmp send_car_from_west

send_car_from_east:
	ldi zh, high(eastQ)
	ldi zl, low(eastQ)
	ld temp3, z
	; no cars, end
	tst temp3
	breq end_try_send_car_jmp

	pop_cars_from_direction_queue eastQ, new_speed


	rjmp send_car_reset_timer
	; pop from relevant queue
	; reset counter
send_car_from_west:
	ldi zh, high(westQ)
	ldi zl, low(westQ)
	ld temp3, z
	; no cars, end
	tst temp3
	breq jmp_2
	rjmp continue_send
jmp_2:
	jmp end_try_send_car
continue_send:

	pop_cars_from_direction_queue westQ, new_speed


	rjmp send_car_reset_timer

send_car_reset_timer:
	add_car_to_road new_speed

	mov temp9, new_speed
		do_lcd_command 0b1000_1000
	ldi temp3, 'S'
	do_lcd_data temp3
	to_decimal temp9
	do_lcd_data digit_1
	do_lcd_data digit_2

	clr three_seconds_count

	end_try_send_car:
	pop temp9
	pop temp4
		pop temp3
		pop zh
		pop zl
.endmacro

/* INCREMENT TIMER TO LIMIT */
.macro increment_timer_to
	push temp5
	mov temp5, @0
	cpi temp5, @1
	brlo increment
	rjmp end_increment
increment:
	inc @0
end_increment:
	pop temp5

.endmacro


/* INITIALISE THE QUEUES */
.macro initialise_queues
	push temp4
	in temp4, SREG
	push temp4
	push yh
	push yl

	push temp5

	ldi yh, high(eastQ)
    ldi yl, low(eastQ)

	mov temp5, yl
	subi temp5, -226
	clr temp4
loop:
	cp yl, temp5
	brsh end_initialize
	st y+, temp4
	rjmp loop

end_initialize:
	pop temp5

	pop yl
	pop yh
	pop temp4
	out SREG, temp4
	pop temp4
.endmacro
/* END INITIALISE THE QUEUES */

/* ADD CARS TO THE DIRECTION QUEUE */
; @0 is the west/east queue we want to modify
; @1 is the car speed we want to add
.macro add_cars_to_direction_queue
	in temp4, SREG
	push temp4
	push temp5
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
	pop temp5
	pop temp4
	out SREG, temp4
    nop
.endmacro
/* END ADD CARS TO THE DIRECTION QUEUE */

/* SLOW DOWN ADDING CARS */
.macro slow_down_add_cars
	push temp5
	push zh
	push zl

	ldi zh, high(@0)
	ldi zl, low(@0)

	ld temp5, z
	; if no cars, reset timer
	cpi temp5, 2
	brsh end_slow_down_add_cars

	clr three_seconds_count


end_slow_down_add_cars:
pop zl
pop zh
	pop temp5
.endmacro



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

	/*
	do_lcd_command 0b1000_1000
	to_decimal @1
	do_lcd_data digit_1
	do_lcd_data digit_2
	*/
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

	sbrc road_status, EMERGENCY_STATE	; skip next line if emergency state is cleared/off
	rjmp end_add_car_to_road

	ldi zh, high(roadQ)					; the address of the number_of_items
    ldi zl, low(roadQ)

	ld temp5, z+						; temp5 has the value of no_of_items in this queue		
	ld temp4, z+						; temp4 the value of head, z now points to the first car speed in the queue

	add temp5, temp4					; temp5 = no_of_items + head offset
	wrap_around_pointer temp5, 10		; wrap around

update_car_speed:
	add zl, temp5						; go to the new last position to insert car speed
	mov temp3, @0						; load provided speed to temp3

	/*
	mov temp6, temp3
	to_decimal temp6
	do_lcd_command 0b1100_0100
	do_lcd_data digit_1
	do_lcd_data digit_2*/

	st z, temp3							; store the car speed
	clr temp3							; store the car position as 0
	std z+10, temp3

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
	st z, temp4							; set the two crashed car speed to 0
	dec zl
	st z, temp4
	sbrs road_status, EMERGENCY_STATE	; skip next line if emergency state is set/on
	ori road_status, (1<<EMERGENCY_STATE)	; set road status to be emergency state
	;sbrc road_status, EMERGENCY_STATE
    ;cbi porte, INTERRUPT_BIT
	update_car_speed_before_tow
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
	sub temp5, num_cars
	ldd temp4, z+1
	add temp4, temp5
	std z+1, temp4

	pop zl
	pop zh
	pop temp6
	pop temp5
	pop temp4
	pop temp3
	out SREG, temp3
	pop temp3

	; check if it's emergency state, if it is, go to update_car_speed_after_tow
	sbrc road_status, EMERGENCY_STATE	; skip next line if emergency state is cleared/off
	check_car_and_tow
    nop
.endmacro

/* UPDATE CAR SPEED BEFORE tow */
.macro update_car_speed_before_tow
	in temp3, SREG
	push temp3
	push temp5
	push temp6
	push temp8
	push zh
	push zl

	ldi zh, high(roadQ)							; the address of the no_of_items (number)	
    ldi zl, low(roadQ)

; Firstly, set every car speed before the first crashed car to be 0
	ld temp8, z+								; temp8 has the value of number_of_items		
	ld temp5, z+								; temp5 has the value of head, z points to the first car speed in the queue

	add zl, temp5								; go to the first car speed

	clr temp5									; clear temp5, use it as a counter
	; clear temp6, use it to indicate if we want to set the speed to 10 or 0.
	; 1 means we want to set it to 10, 0 means we want to set it to 0.
	ser temp6
update_car_speed_before_tow_loop:
	cp temp5, temp8								; compare counter and number_of_items
	brsh end_update_car_speed_before_tow		; breach if larger or equal

	tst temp6									; go to set_speed_to_zero if temp6 is 0
	breq set_speed_to_zero
	; otherwise, check if this is the crashed car
	ld temp3, z									; get the current car speed to temp3
	tst temp3									; check if the current car speed is 0
	brne set_speed_to_ten						; if it is not, go to set_speed_to_ten
	clr temp6									; otherwise, clear temp6
	rjmp end_update_car_speed_before_tow_loop
set_speed_to_ten:
	ldi temp3, 10								; otherwise, set the current speed to 10
	st z, temp3
	rjmp end_update_car_speed_before_tow_loop
set_speed_to_zero:
	clr temp3									; otherwise, set the current speed to 0
	st z, temp3
	rjmp end_update_car_speed_before_tow_loop
end_update_car_speed_before_tow_loop:
	inc zl										; go to the next car speed
	inc temp5									; increment the counter
	rjmp update_car_speed_before_tow_loop		; jump back to update_car_speed_before_tow_loop

end_update_car_speed_before_tow:
	pop zl
	pop zh
	pop temp8
	pop temp6
	pop temp5
	pop temp3
	out SREG, temp3
.endmacro
/* END UPDATE CAR SPEED BEFORE tow */


/* CHECK IF CAR NEED TO  BE towED AND tow CARS */
.macro check_car_and_tow
	in temp4, SREG
	push temp3
	push temp4
	push temp5
	push zh
	push zl

	sbrs road_status, EMERGENCY_STATE			; skip next line if emergency state is set/on
	rjmp end_check_car_and_tow

	ldi zh, high(roadQ)							; the address of the no_of_items (number)	
    ldi zl, low(roadQ)

	ld temp4, z+								; temp4 has the value of number_of_items		
	ld temp5, z+								; temp5 has the value of head, z points to the first car speed in the queue

	add zl, temp5								; go to the first car speed in the queue
	ld temp4, z									; load this value to temp4

	tst temp4									; check if car speed is 0
	brne end_check_car_and_tow					; if not, go to end_check_car_and_tow

	; Once confirm the cars in front have drove off, wait for three seconds
	ldi temp3, 3
	cp r14, temp3								; if three_seconds_count is < 3
	brlo increase_tow_delay_counter				; increase r14 and end
	clr r14										; otherwise, clear r14 and tow cars away

	; Otherwise, tow car away
	ldi zh, high(roadQ)							; the address of the no_of_items (number)	
    ldi zl, low(roadQ)

	; tow two cars away
	ldi temp4, 2
	sub num_cars, temp4							; decrease the num_cars by 2
	st z, num_cars								; store this value to roadQ
	ldd temp5, z+1								; load current head offset value
	ldi temp4, 2
	add temp5, temp4							; increase this value by 2
	wrap_around_pointer temp5, 10				; wrap around
	std z+1, temp5								; store the new head value

	update_car_speed_after_tow
increase_tow_delay_counter:
	inc r14
end_check_car_and_tow:
	pop zl
	pop zh
	pop temp5
	pop temp4
	pop temp3
	out SREG, temp4
.endmacro
/* END CHECK IF CAR NEED TO  BE towED AND tow CARS */

/* UPDATE CAR SPEED AFTER tow */
.macro update_car_speed_after_tow
	in temp4, SREG
	push temp3
	push temp4
	push temp5
	push temp8
	push zh
	push zl

	ldi zh, high(roadQ)							; the address of the no_of_items (number)	
    ldi zl, low(roadQ)

; Now the crashed cars have been towed, set every car speed to be 10
	ld temp8, z+								; temp8 has the value of number_of_items		
	ld temp5, z+								; temp5 has the value of head, z points to the first car speed in the queue

	add zl, temp5								; go to the first car speed

	clr temp5									; clear temp5, use it as a counter
update_car_speed_after_tow_loop:
	cp temp5, temp8								; compare counter and number_of_items
	brsh end_update_car_speed_after_tow		; breach if larger or equal

	ldi temp3, 10								; set the current speed to 10
	st z, temp3

	inc zl										; go to the next car speed
	inc temp5									; increment the counter
	rjmp update_car_speed_after_tow_loop		; jump back to update_car_speed_after_tow_loop

end_update_car_speed_after_tow:
	pop zl
	pop zh
	pop temp8
	pop temp5
	pop temp4
	pop temp3
	out SREG, temp4
.endmacro
/* END UPDATE CAR SPEED AFTER tow */

/* TURN TRAFFIC LIGHT RED */
.macro make_traffic_lights_red
    push zh
    push zl
	push temp4

    mov temp4, road_status

	; If emergency, ignore
	sbrc temp4, EMERGENCY_STATE
	rjmp end_make_traffic_lights_red

	; If light already red (state 1 or 3, i.e., bit 0 is 1), ignore 
	sbrc temp4, RED_LIGHT
	rjmp end_make_traffic_lights_red

	; Get direction. SKip if EAST_TO_WEST=0 i.e. travelling WEST_TO_EAST
	sbrc temp4, EAST_TO_WEST_DIRECTION
	rjmp cars_moving_towards_west
	rjmp cars_moving_towards_east

    
    ; if it's going west we check if there's any cars in the east queue
    cars_moving_towards_west:
        ldi zh, high(westQ)
        ldi zl, low(westQ)
		ld temp4, z			; first element of eastQ is number of cars

        tst temp4
        breq end_make_traffic_lights_red
	; check if there's cars in the current queue
		ldi zh, high(eastQ)
		ldi zl, low(eastQ)
		ld temp4, z
		tst temp4
		breq set_to_red
        rjmp timer
    
    cars_moving_towards_east:

        ldi zh, high(eastQ)
        ldi zl, low(eastQ)
        ld temp4, z
        tst temp4
        breq end_make_traffic_lights_red

	; check if there's cars in the current queue
		ldi zh, high(westQ)
		ldi zl, low(westQ)
		ld temp4, z
		tst temp4
		breq set_to_red
        rjmp timer
        
    timer:
        mov temp4, three_minutes_count
        cpi temp4, 180                ; ahs it been 3 minutes?
        brsh set_to_red
        rjmp end_make_traffic_lights_red

    set_to_red:
		
        ori road_status, (1<<RED_LIGHT)
        rjmp end_make_traffic_lights_red
        
    end_make_traffic_lights_red:
        pop temp4
        pop zl
        pop zh

.endmacro

/* MAKE TRAFFIC LIGHTS GREEN */
.macro make_traffic_lights_green
    push temp4
    tst num_cars            ; cars on the road means we can't change the light yet
    brne end_make_traffic_lights_green

	; Ignore if emergency
	sbrc road_status, EMERGENCY_STATE
	rjmp end_make_traffic_lights_green

	; If green light, ignore ; skip if redlight=1 i.e. green_lightoff
	sbrs road_status, RED_LIGHT
	rjmp end_make_traffic_lights_green



	; We have a red light, no emergency
	; Change the direction bit
    ldi temp4, (1<<EAST_TO_WEST_DIRECTION)
    eor road_status, temp4            ; flip the last bit to change direction
	; Turn on green light
    andi road_status, RED_LIGHT_MASK
    
	clr three_minutes_count
    		ldi temp4, 1
		mov three_seconds_count, temp4
    end_make_traffic_lights_green:

        pop temp4

.endmacro

/* LCD MACROS AND FUNCTIONS */
.macro do_lcd_command
	push temp1
	ldi temp1, @0
	call lcd_command
	call lcd_wait
	pop temp1
.endmacro
.macro do_lcd_command_register
	push temp1
	mov temp1, @0
	rcall lcd_command
	rcall lcd_wait
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

/* CLEAR CG RAM */
.macro clear_cgram
	push temp1
	do_lcd_command_register @0
	clr temp1
	do_lcd_data temp1
	do_lcd_data temp1
	do_lcd_data temp1
	do_lcd_data temp1
	do_lcd_data temp1
	do_lcd_data temp1
	do_lcd_data temp1
	do_lcd_data temp1

	;do_lcd_command 

	pop temp1

.endmacro
/* END CLEAR CG RAM */


.macro show_cars
	push zh
	push zl
	push temp8
	push temp9
	push temp3

	ldi zh, high(roadQ)
	ldi zl, low(roadQ)

	ldd temp3, z+1	; head
	subi temp3, -12

	add zl, temp3
	
	mov temp3, num_cars
	clr temp8

	ldi temp9, 0b1000_0011
loop:
	cp temp8, num_cars
	brsh end_loop
	ld temp3, z+

	do_lcd_command_register temp9
	to_decimal_three temp3
	do_lcd_data digit_1
	do_lcd_data digit_2
	do_lcd_data digit_3
	subi temp9, -3	
	inc temp8
	rjmp loop
	
end_loop:

	pop temp3
	pop temp9
	pop temp8
	
	pop zl
	pop zh
.endmacro
/* SHOW CAR POSITION ON ROAD */
show_car_position: 
	push temp1
	push temp2
	push temp3
	push temp4
	push temp5
	push temp6
	push temp7
	push temp8
	push temp9
	; Rationale: 8 positions in increments of 24 from 0 to (191)
	; Display number of cars in each bin (0-23, 24-47etc)
	in temp3, SREG
	push temp3

	push zh
	push zl

	ldi zh, high(roadQ)
	ldi zl, low(roadQ)

	mov temp5, zl				; temp5 contains last location in memory (for wrapping around)
	subi temp5, -22				; NEED TO DOUBLE CHECK THIS ISN'T ONE OFF, should be 23?

	ldd temp3, z+1				; temp3 contains head location
	subi temp3, -12				; Offset to get position
	; ZL points to start of queue, add 2 + 10 + head

	ldi temp1, 0				; let temp1 be lower bound
	ldi temp2, 23				; let temp2 be the upper bound

	clr temp7					; let temp7 be the count of cars in bin
	add temp3, zl				; temp3 stores first car

	;mov zl, temp3
	;ld temp3, z
	/*
	; TESTING 
	;mov zl, temp3
	;inc zl
	;inc zl
	;ldd temp3, z+12
	to_decimal temp3
	do_lcd_command 0b1100_0100
	do_lcd_data digit_1
	do_lcd_data digit_2
	rjmp out_of_bounds
	*/
/*END TESTIN */

	; From now on, temp3 contains location of current car
	; Where to put the character on the bottom row
	sbrs road_status, EAST_TO_WEST_DIRECTION
	ldi temp7, 0b1100_0100
		sbrc road_status, EAST_TO_WEST_DIRECTION
	ldi temp7, 0b1100_1011

	; Character to print out in CGRAM
	ldi temp8, 0
	; Location of CGRAM top row
	ldi temp9, 0b0100_0000

	rjmp loop_over_bins
out_of_bounds_jmp:
	jmp out_of_bounds
loop_over_bins:
	cpi temp2, 192		; If upper bound past 192, no more cars to print
	brsh out_of_bounds_jmp
	clear_cgram temp9					; clear character
	do_lcd_command_register temp9		; set custom character edit top row
	; Move ZH:ZL so it points to first car
	mov zl, temp3
	mov temp4, num_cars					; temp4 iterates over number of cars
try_get_position:
	tst temp4
	breq display_car_jmp					; no more cars in bin, display
	rjmp continue_get_car
display_car_jmp:
	jmp display_car
continue_get_car:
	cp zl, temp5						; Circular buffer
	brlo get_car_position
	subi zl, 10
get_car_position:
	ld temp6, z+					; load car position into z

	dec temp4
	; Check if car is in bin: if car below temp1 or car position above temp2, skip
	cp temp6, temp1
	brlo not_in_bin
	cp temp2, temp6
	brlo not_in_bin

	; Now, we are in the bin, increment
	; set car number, 
	; for cars in 1 to 5:
	; if car num = num: print line, otherwise print blank
	ldi temp6, 0b00011111
	do_lcd_data temp6
	rjmp try_get_position

	; For each [0-23], [24-47], ... [168,191] chunk, count number of cars
not_in_bin:
	; blank line
	ldi temp6, 0b00000000
	do_lcd_data temp6
	rjmp try_get_position

display_car:
	; move cursor back
	do_lcd_command_register temp7		; Where to print the character on the line
	sbrs road_status, EAST_TO_WEST_DIRECTION
	inc temp7
	sbrc road_status, EAST_TO_WEST_DIRECTION
	dec temp7

	subi temp9, -8						; Move to next cgrom location
	do_lcd_data temp8			
	inc temp8

	;; Move to the next bin
	subi temp1, -24
	subi temp2, -24
	jmp loop_over_bins

out_of_bounds:
	do_lcd_command 0b11111111						; get rid of cursor on bottom row
	pop zl
	pop zh
	pop temp3
	out SREG, temp3
	pop temp9
	pop temp8
	pop temp7
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
	push temp2
	; If west to east, show lights
	sbrc road_status, EMERGENCY_STATE
	rjmp emergency_led

	; Get traffic light state
	mov temp1, road_status
	andi temp1, TRAFFIC_STATE_BIT_MASK
	
	cpi temp1, 0
	breq west_light_on

	cpi temp1, 2
	breq east_light_on

	; otherwise lights off
	ldi temp1, LIGHTS_OFF
	out portc, temp1
	rjmp refresh_lights_end
west_light_on:
	ldi temp1, WEST_LIGHT
	out portc, temp1
	rjmp refresh_lights_end
east_light_on:
	ldi temp1, EAST_LIGHT

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
	pop temp2
	pop temp1
.endmacro


/* DISPLAY MACRO */
.macro refresh_display_top
	push temp1
	push temp2
	push zh
	push zl

	; print number of cars
	do_lcd_command 0b1000_0101
	ldi temp1, 'N'
	do_lcd_data temp1
	mov temp1, num_cars
	to_decimal temp1
	do_lcd_data digit_2

	do_lcd_command 0b1000_0000			; move to first row
	; print number of cars in west queue TODO

	ldi zh, high(westQ)
	ldi zl, low(westQ)
	ld temp2, z
	to_decimal temp2

	do_lcd_data digit_1
	do_lcd_data digit_2

	; print cars and direction
	; If direction bit is 0: west
	sbrs road_status, EAST_TO_WEST_DIRECTION
	rjmp west_car_pattern
	
	; we have east casrs:
	ldi temp1, '<'
		do_lcd_data temp1
	do_lcd_data temp1
	do_lcd_command 0b1000_1100
	ldi temp1, ' '
	do_lcd_data temp1
	do_lcd_data temp1

	/* UNCOMMENT THESE TO SHOW EQUALS 
	display_car_symbols num_cars	
	display_car_padding num_cars
	*/
	rjmp no_more_cars

west_car_pattern:
	do_lcd_command 0b1000_0010
	ldi temp1, ' '
	do_lcd_data temp1
	/* UNCOMMENT THESE TO SHOW EQUALS 
	display_car_padding num_cars
	display_car_symbols num_cars
	*/	
	; print space
	do_lcd_command 0b1000_0010
	ldi temp1, ' '
	do_lcd_data temp1
	do_lcd_data temp1
	do_lcd_command 0b1000_1100
	ldi temp1, '>'
	do_lcd_data temp1
	do_lcd_data temp1

no_more_cars:
	; print space

	ldi zh, high(eastQ)
	ldi zl, low(eastQ)
	ld temp2, z
	do_lcd_command 0b1000_1110
	to_decimal temp2

	do_lcd_data digit_1
	do_lcd_data digit_2
	pop zl
	pop zh
	pop temp2
	pop temp1
.endmacro

/* Refresh second row for input */
.macro refresh_bottom
	push temp1
	do_lcd_command 0b1100_0011
	ser temp1
	do_lcd_data temp1

	do_lcd_command 0b1100_1100
	ser temp1
	do_lcd_data temp1

	;sbrs road_status, CAR_DETECTED
	;rjmp end_refresh_bottom

	do_lcd_command 0b1100_0000						; move cursor to second row
	mov temp1, car_speed
	subi temp1, -10
	to_decimal temp1
	do_lcd_data digit_1
	do_lcd_data digit_2
	do_lcd_command 0b1111_1111

end_refresh_bottom:
	pop temp1
.endmacro

/* MAIN FUNCTIONALITY */
RESET:
/*
	initialise_queues
	ldi temp1, 11
	add_car_to_road temp1
	update_car_positions
	update_car_positions
	update_car_positions
	add_car_to_road temp1
	update_car_positions
	update_car_positions
	update_car_positions
	ldi temp1, 10
	add_car_to_road temp1
	update_car_positions
	update_car_positions
	update_car_positions
	ldi temp1, 15
	add_car_to_road temp1
	update_car_positions
	update_car_positions
	update_car_positions
	add_car_to_road temp1
	update_car_positions
	update_car_positions
	update_car_positions
	update_car_positions */

	ldi temp1, low(RAMEND)
	out SPL, temp1
	ldi temp1, high(RAMEND)
	out SPH, temp1

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

	clr road_status ; set west light on
	;ldi temp1, 5			; 3 cars on road
	clr temp1
	mov num_cars, temp1

	clr r14								; ensure the tow counter is set to 0

	do_lcd_command 0b00111000 ; 2x5x7
	call sleep_5ms
	do_lcd_command 0b00111000 ; 2x5x7
	call sleep_1ms
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00001000 ; display off
	do_lcd_command 0b00000001 ; clear display
	do_lcd_command 0b00000110 ; increment, no display shift
	do_lcd_command 0b00001110 ; Cursor on, bar, no blink

	clr three_minutes_count
	clr three_seconds_count
	clr timer_overflow_count
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

	ldi temp1, 3
	mov three_seconds_count, temp1

	sei									; set interrupt flag
	initialise_queues


	jmp check_keypad

	; Set traffic light from West to East on

/* PB1 (WEST) DETECTED EXT_INT0 */
; Set bit 6 in register road_status
EXT_INT0:
	push temp8
	call sleep_100ms

	cli
	sbrs road_status, CAR_DETECTED
	rjmp ignore_west

	do_lcd_command 0b11000010							; set the location of the cursor to be at the beginning of the second line
	ldi temp8, 'W'										; for testing purpose
	do_lcd_data temp8

	mov temp8, car_speed
	subi temp8, -10

	add_cars_to_direction_queue westQ, temp8

	;slow_down_add_cars westQ

	; which queue to go to depends on the road_status
	andi road_status, CAR_DETECTED_MASK			
	refresh_display_top
	refresh_bottom
ignore_west:
	sei
	pop temp8
	reti

/* PB0 (EAST) DETECTED EXT_INT1 */
; Set bit 5 in register road_status
EXT_INT1:
	cli
	call sleep_100ms

	push temp8
	sbrs road_status, CAR_DETECTED
	rjmp ignore_east
	; testing
	do_lcd_command 0b11000010 ; remove W if necessary

	ldi temp8, 'E'
	do_lcd_data temp8

	mov temp8, car_speed
	subi temp8, -10
	add_cars_to_direction_queue eastQ, temp8

	;slow_down_add_cars eastQ
	andi road_status, CAR_DETECTED_MASK
refresh_display_top
refresh_bottom
ignore_east:
	sei
	pop temp8
	reti

/* INTERNAL INTERRUPT TRIGGERED */
; This can be triggered under two condition:
; 1. We press the * button
; 2. Two cars have collided with each other
EXT_INT7:
	sbi porte, INTERRUPT_BIT
	cli
	push temp1
	call sleep_100ms
	call sleep_100ms
	call sleep_100ms

	; If emergency state is on, go out of emergency state 
	sbrc road_status, EMERGENCY_STATE
	rjmp exit_emergency
	do_lcd_command 0b1100_1110
	ldi temp1, 'E'
	do_lcd_data temp1
	ldi temp1, 'S'
	do_lcd_data temp1

	; Otherwise, set road status to be emergency state
	ori road_status, (1<<EMERGENCY_STATE)

	rjmp end_emergency
exit_emergency:
	do_lcd_command 0b1100_1110
	ldi temp1, 'E'
	do_lcd_data temp1
	ldi temp1, 'E'
	do_lcd_data temp1
	andi road_status, EMERGENCY_STATE_MASK

	rjmp end_emergency
end_emergency:
	; Reset interrupt bit 1
	pop temp1
	nop
	sei
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

	ori road_status, (1<<CAR_DETECTED)

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

	inc timer_overflow_count
	; Check if timer_overflow_counter >= 61
	mov temp1, timer_overflow_count
	cpi temp1, 61
	;cpi temp1, 100
	brne not_second_jmp
	jmp refresh	
not_second_jmp:
	jmp not_second
refresh:													; one second has passed
	; update timers:
	clr timer_overflow_count
	increment_timer_to three_minutes_count, 180
	increment_timer_to three_seconds_count, 3

	update_car_positions
	
	make_traffic_lights_red
	make_traffic_lights_green
	try_send_car

	refresh_display_top
	refresh_lights
	;show_cars
	call show_car_position

	jmp end_timer

not_second:
	rjmp end_timer
end_timer:

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