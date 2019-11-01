;
; Rutina para mandar datos a un LCD con I2C
;

.equ START=0b10100100
.equ LCD_W=0x40
.def DATA=r22
;comandos
.EQU SET_ENABLE = 0x04
.EQU FUNCTION_SET = 0x2C
.EQU SET_DISPLAY = 0x0C
.EQU CLEAN_DISPLAY = 0x01
.EQU ENTRY_MODE = 0x06
.EQU NEXT_LINE = 0xC0

;mascaras
.EQU MASK_4LSB = 0x0F
.EQU MASK_MSB = 0x80

.EQU ENABLE=2
.EQU RS=0
.EQU RW=1

.include "m328pdef.inc"

.cseg
	rjmp main



.org INT_VECTORS_SIZE

main:
		ldi  r16,low(ramend)
		out spl,r16
		ldi  r16,high(ramend)
		out spl,r16
		;inicializo stack

		rcall I2C_init
		rcall I2C_start
		ldi DATA,LCD_W	;mando sla + w
		rcall I2C_write

		ldi r16,	CLEAN_DISPLAY	;dato a mandar
		mov r17,r16	;copio r16 en r17
		swap r17		
		ANDI r16,~ MASK_4LSB ;  dejo los ultimos de r16 (más significativos)
 		ori r16,1<<ENABLE		; seteo el bit de enable en el registro a mandar
		ldi DATA,SET_ENABLE		
		rcall I2C_write	;seteo el eneable
		mov DATA,r16
		rcall I2C_write	; mando los ultimos 4 bits
		ANDI R17, ~MASK_4LSB ; dejo los ultimos de r17
		ori r17,1<<ENABLE
		mov DATA,r17
		rcall I2C_write	;mando los primeros 4bits
		rcall I2C_stop


	

			here:
				rjmp here


			ERROR:
					SBI DDRB, 5
					SBI PORTB, 5
				rjmp here
			I2C_init:
					ldi r16,0
					sts TWSR, r16	;preescaler en 0
					ldi r16,0x47
					sts TWBR,r16		;f clock=50k
					ldi r16,(1<<TWEN)
					sts TWCR,r16
					ret
			 I2C_start:
				ldi r16, (1<<TWINT)| (1<<TWSTA)| (1<<TWEN)
				sts TWCR,r16		;mando pulso de start
				wait1:
				lds r16,TWCR
				sbrs r16,TWINT
				rjmp wait1
				ret

		I2C_write:		
				sts TWDR,DATA		;no esta en I/O
				ldi r16,(1<<TWINT)|(1<<TWEN)
				sts TWCR,r16
			wait3:
				lds r16,TWCR
				sbrs r16,TWINT
				rjmp wait3
				ret

		I2C_stop:
				ldi r16, (1<<TWINT)| (1<<TWEN)| (1<<TWSTO)
				sts TWCR,r16	;mando pulso de stop
				ret