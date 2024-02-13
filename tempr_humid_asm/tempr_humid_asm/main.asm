;
; tempr_humid_asm.asm
;
; Created: 27.09.2022 18:45:10
; Author : omprig
;


; Replace with your application code

.include "m128Adef.inc"

.def temp = r16
.def delay2_value = r18
.def current_address = r19
.def twi_flag_counter = r20
.def divident_l = r21
.def divident_h = r22
.def quotient = r23
.def status_check = r24


.equ divisor = 0x0A

;--------Timer2 delay values

.equ delay2_9ms = 0xFF
.equ delay2_2ms = 0xE6
.equ delay2_50us = 0x04

;--------Timer1 delay values

.equ delay1_3s = 0x383F;0x545F


;------- LCD constants and commands

.equ LINE1_START_ADDR = 0x00; the start address of the first line of the LCD
.equ LINE2_START_ADDR = 0x40; the start adderss of the second line of the LCD
.equ LCD_DATA = PORTC; PORTC
.equ LCD_CONTROL_SIGNAL = PORTA; PORTA
.equ E = 0
.equ RS = 1

.equ CURS_DISP_SHIFT = 0x00; cursor or display shift
.equ ENTRY_MODE_SET = 0x06; increment cursor positio, without display shift
.equ FUNC_SET = 0x38; 8-bit mode, two lines, 5x8 character size
.equ DISP_ON_OFF = 0x0C; display on, cursor off, cursor position off
.equ SET_DDRAM_ADDR = 0x80; aplly or with opted address

;------------TWI

.equ TWI_START = 0x08
.equ TWI_SLA_W_A = 0x18
.equ TWI_SLA_W_N = 0x20
.equ TWI_BYTE_TR_A = 0x28
.equ TWI_SLA_R_A = 0x40
.equ TWI_BYTE_R_A = 0x50
.equ TWI_BYTE_R_N = 0x58

;------------AM2320

.equ AM2320_WRITE = 0xB8
.equ AM2320_READ = 0xB9
.equ CONTINUE_READING_SENSOR = 7
.equ OUTPUT_SENSOR_DATA = 6
.equ WRITE_TO_SENSOR= 0x10
.equ START_ADDRESS = 0x00
.equ N_BYTES = 0x04
.equ READ_SENSOR = 0x03

.ESEG

	.org 0x00
LCD_INIT_DATA:
	.DB "Temper =    . ", 0xDF, "CHumidity =   . %"

.DSEG

SENSOR_DATA:
	.BYTE 8

SENSOR_COMMAND:
	.BYTE 3

SENSOR_DIGITS:
	.BYTE 3

.CSEG
   .org 0x0000
		jmp main
   .org 0x0018
		jmp Timer_Counter1_COMPA_ISR
   .org 0x0042
		jmp TWI_ISR

   .org 0x0046
main:    ; main program
	ldi temp, low(RAMEND)
	out spl, temp
	ldi temp, high(RAMEND)
	out sph, temp ; stack initialization

	; LCD initialization
	
	ldi temp, 0xFF
	out DDRC, temp; PORTC as output

	ldi temp, 0x03
	out DDRA, temp; PA0 and PA1 as output
	
	ldi temp, CURS_DISP_SHIFT
	call send_command_LCD
	
	ldi temp, ENTRY_MODE_SET
	call send_command_LCD
	
	ldi temp, FUNC_SET
	call send_command_LCD	

	ldi temp, SET_DDRAM_ADDR
	call send_command_LCD
	
	ldi temp, low(LCD_INIT_DATA)
	out EEARL, temp
	ldi temp, high(LCD_INIT_DATA)
	out EEARH, temp

	ldi r17, 0
read_eeprom_1:
	sbic EECR, EEWE
	rjmp read_eeprom_1; wait for EEWE

	sbi EECR, EERE ; read enable

	in temp, EEDR
	call send_data_LCD

	in temp, EEARL
	inc temp
	out EEARL, temp ; set EEPROM address

	inc r17
	cpi r17, 0x10
	in temp, SREG
	sbrs temp, 1
	rjmp read_eeprom_1

	ldi temp, (SET_DDRAM_ADDR | LINE2_START_ADDR)
	call send_command_LCD  

	ldi r17, 0
read_eeprom_2:
	sbic EECR, EEWE
	rjmp read_eeprom_2


	sbi EECR, EERE

	in temp, EEDR
	call send_data_LCD

	in temp, EEARL
	inc temp
	out EEARL, temp

	inc r17
	cpi r17, 0x10
	in temp, SREG
	sbrs temp, 1
	rjmp read_eeprom_2

	ldi temp, DISP_ON_OFF
	call send_command_LCD

;----------------TWI Init

	ldi temp, 0x20
	sts TWBR, temp; adjust the speed
	ldi temp, 0x00
	sts TWSR, temp

;-------------- Sensor Init

	ldi ZL, LOW(SENSOR_COMMAND)
	ldi ZH, HIGH(SENSOR_COMMAND)

	ldi temp, READ_SENSOR
	st Z+, temp

	ldi temp, START_ADDRESS
	st Z+, temp

	ldi temp, N_BYTES
	st Z, temp

;----------------Timer/Counter1 Init

	ldi temp, HIGH(delay1_3s)
	out OCR1AH, temp

	ldi temp, LOW(delay1_3s)
	out OCR1AL, temp; set value for comparing

	ldi temp, (1 << PSR321); clear prescaler
	out SFIOR, temp

	ldi temp, (1 << OCIE1A)
	out TIMSK, temp ; enable OCF1A interrupt 

	ldi temp, (1 << WGM12) | (1 << CS12) | (1 << CS10); prescaler value 1024, ctc mode 
	out TCCR1B, temp; launche Timer/Counter1

	ldi twi_flag_counter, 0x00
	ldi r25, 0x00

	sei

main_loop:
	
	cpi r25, 0x01
	brne NEXT_START
	call continue_sensor_reading
	

NEXT_START:	
	cpi r25, 0x02
	brne main_loop
	call put_data_on_LCD

	rjmp main_loop

	
; send data to LCD

send_data_LCD:

	out LCD_DATA, temp
	
	sbi LCD_CONTROL_SIGNAL, RS
	ldi delay2_value, delay2_9ms
	call Timer2_delay

	sbi LCD_CONTROL_SIGNAL, E
	ldi delay2_value, delay2_9ms
	call Timer2_delay

	cbi LCD_CONTROL_SIGNAL, E
	ldi delay2_value, delay2_9ms
	call Timer2_delay

	ret

; send command for LCD

send_command_LCD:

	out LCD_DATA, temp

	cbi LCD_CONTROL_SIGNAL, RS
	ldi delay2_value, delay2_9ms
	call Timer2_delay

	sbi LCD_CONTROL_SIGNAL, E
	ldi delay2_value, delay2_9ms
	call Timer2_delay

	cbi LCD_CONTROL_SIGNAL, E
	ldi delay2_value, delay2_9ms
	call Timer2_delay
	ret

Timer_Counter1_COMPA_ISR:

	ldi r25, 0x00

	ldi current_address, AM2320_WRITE
	ldi twi_flag_counter, 0x00
	ldi temp, (1 << TWSTA) | (1 << TWINT) | (1 << TWIE) | (1 << TWEN)
	sts TWCR, temp; TWI start

	reti

TWI_ISR:

	lds status_check, TWSR
	andi status_check, 0xF8

	cpi status_check, TWI_START
	brne NEXT1
	call START_HANDLER
	reti

NEXT1:	
	cpi status_check, TWI_SLA_W_A
	brne NEXT2
	call SLA_W_A_HANDLER
	reti

NEXT2:
	cpi status_check, TWI_SLA_W_N
	brne NEXT3
	call SLA_W_N_HANDLER
	reti

NEXT3:
	cpi status_check, TWI_BYTE_TR_A
	brne NEXT4
	call BYTE_TR_A_HANDLER
	reti

NEXT4:
	cpi status_check, TWI_SLA_R_A
	brne NEXT5
	call SLA_R_A_HANDLER
	reti

NEXT5:
	cpi status_check, TWI_BYTE_R_A
	brne NEXT6
	call BYTE_R_A_HANDLER
	reti

NEXT6:
	cpi status_check, TWI_BYTE_R_N
	brne TWI_end

	lds temp, TWDR
	st Z, temp

	ldi r25, 0x02
		
	ldi temp, (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWSTO); generate stop bit
	sts TWCR, temp

TWI_end:	reti

START_HANDLER:
	
	sts TWDR, current_address
	ldi temp, (1 << TWINT) | (1 << TWEN) | (1 << TWIE)
	sts TWCR, temp
	ret

SLA_W_A_HANDLER:

	ldi current_address, AM2320_READ
	ldi twi_flag_counter, 0x00

	ldi ZL, LOW(SENSOR_COMMAND)
	ldi ZH, HIGH(SENSOR_COMMAND)

	ld temp, Z+
	sts TWDR, temp

	ldi temp, (1 << TWINT) | (1 << TWEN) | (1 << TWIE)
	sts TWCR, temp
	ret

SLA_W_N_HANDLER:
	
	ldi r25, 0x01

	ldi temp, (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWSTO); generate stop bit
	sts TWCR, temp

	ret

BYTE_TR_A_HANDLER:
	
	inc twi_flag_counter
	ldi temp, (1 << TWSTO) | (1 << TWINT) | (1 << TWEN) | (1 << TWIE)
	ldi r25, 0x01
	cpi twi_flag_counter, 0x03
	breq NEXT_TR
	; write next byte to sensor
	ld temp, Z+
	sts TWDR, temp
	ldi r25, 0x00
	ldi temp, (1 << TWINT) | (1 << TWEN) | (1 << TWIE)
	
NEXT_TR:
	
	sts TWCR, temp	
	ret

SLA_R_A_HANDLER:

	ldi twi_flag_counter, 0x00

	ldi delay2_value, delay2_50us; 50us
	call Timer2_Delay

	ldi ZL, LOW(SENSOR_DATA)
	ldi ZH, HIGH(SENSOR_DATA)

	ldi temp, (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWEA)
	sts TWCR, temp
	
	ret

BYTE_R_A_HANDLER:
	
	lds temp, TWDR
	st Z+, temp

	ldi temp, (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWEA)

	inc twi_flag_counter
	
	cpi twi_flag_counter, 0x07
	brne READ_NEXT
	ldi temp, (1 << TWINT) | (1 << TWEN) | (1 << TWIE) ; NACK

READ_NEXT:	
	sts TWCR, temp
	ret
	
Timer2_Delay:
	
	out OCR2, delay2_value

	ldi temp, (1 << PSR321); clear prescaler
	out SFIOR, temp

	ldi temp, (1 << CS20) | (1 << CS21) | (1 << WGM21); ctc mode, 64 value of the prescaler
	out TCCR2, temp; start timer2

delay_loop2:
	in temp, TIFR
	SBRS temp, OCF2

	rjmp delay_loop2 ; wait for OCF2 flag

	ldi temp, 0x00
	out TCCR2, temp

	in temp, TIFR
	ori temp, (1 << OCF2)
	out TIFR, temp; clear OCF2 manually
	
	ret

put_data_on_LCD:

	ldi twi_flag_counter, 0x00
	ldi r25, 0x00
	
	ldi XL, LOW(SENSOR_DATA + 0x02)
	ldi XH, HIGH(SENSOR_DATA)
	
	ld divident_h, X+
	ld divident_l, X+

	call hex_to_ascii_convertion
	
	ldi ZL, LOW(SENSOR_DIGITS + 0x03)
	ldi ZH, HIGH(SENSOR_DIGITS)

	ldi temp, (SET_DDRAM_ADDR | LINE2_START_ADDR | 0x0B)
	call send_command_LCD

	ld temp, -Z
	call send_data_LCD

	ld temp, -Z
	call send_data_LCD

	ldi temp, (SET_DDRAM_ADDR | LINE2_START_ADDR | 0x0E)
	call send_command_LCD

	ld temp, -Z
	call send_data_LCD
	
;---------------Temperature
	ld divident_h, X+
	ld divident_l, X

	ldi temp, (SET_DDRAM_ADDR | LINE1_START_ADDR | 0x09)
	call send_command_LCD

	ldi temp, 0x2D
	sbrs divident_h, 7
	ldi temp, 0x2B
	call send_data_LCD

	andi divident_h, 0x7F

	call hex_to_ascii_convertion

	ldi ZL, LOW(SENSOR_DIGITS + 0x03)
	ldi ZH, HIGH(SENSOR_DIGITS)

	ld temp, -Z
	call send_data_LCD

	ld temp, -Z
	call send_data_LCD
	
	ldi temp, (SET_DDRAM_ADDR | 0x0D)
	call send_command_LCD
	
	ld temp, -Z
	call send_data_LCD

	ret

continue_sensor_reading:
	
	ldi twi_flag_counter, 0x00
	ldi r25, 0x00
	ldi delay2_value, delay2_2ms // delay 2ms
	call Timer2_Delay
	
	ldi temp, (1 << TWSTA) | (1 << TWINT) | (1 << TWIE) | (1 << TWEN)
	sts TWCR, temp; TWI start

	ret

hex_to_ascii_convertion:

	ldi ZL, LOW(SENSOR_DIGITS)
	ldi ZH, HIGH(SENSOR_DIGITS)

	ldi quotient, 0x00

start_check:	
	cpi divident_h, 0x00
	breq next_condition
	rjmp start_divide

next_condition:	
	cpi divident_l, 0x0A
	brlo change_divident

start_divide:
	inc quotient
	subi divident_l, divisor
	sbci divident_h, 0x00
	rjmp start_check
	
change_divident:
	
	ldi temp, 0x30
	add divident_l, temp
	st Z+, divident_l

	cpi quotient, 0x0A
	brlo last_digit
	mov divident_l, quotient
	ldi quotient, 0x00
	rjmp start_divide

last_digit:
	ldi temp, 0x30
	add quotient, temp
	st Z, quotient
	ret