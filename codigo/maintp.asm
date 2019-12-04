;micro atmega328p
;fclock=16MHz

;Display con I2C
;fclock=16MHz


.include "m328pdef.inc"
.EQU CLEAR_LCD=1
.EQU CONFIG_LCD=0x33
.EQU MSBITS=0xF0
.EQU ENABLE=2




.dseg
Leido: .byte 2
Mostrar: .byte 3


.CSEG ; 
	JMP MAIN

.org INT0addr
	rjmp ISR_EXT_INT0

.ORG INT_VECTORS_SIZE

MAIN:
		LDI R21,HIGH(RAMEND)
		OUT SPH,R21
		LDI R21,LOW(RAMEND)
		OUT SPL,R21
		LDI R27,0X4E	;	0100 1110=0x4E

		;call show_msg
	;	call hex_to_ohm

		CALL I2C_INIT
		CALL I2C_START
		CALL I2C_WRITE ; Escribo dirección y W/R
	
		LDI R27,0b00001100	;0x0C

		call DELAY_PRUEBA
		ldi R16, 0x33
		call CMD_WRITE
		call DELAY_2MS
		ldi R16, 0x32
		call CMD_WRITE
		call DELAY_2MS
		ldi R16, 0x28
		call CMD_WRITE
		call DELAY_2MS
		ldi R16, 0x0C
		call CMD_WRITE
		call DELAY_2MS
		ldi R16, 0x01
		call CMD_WRITE
		call DELAY_2MS
		ldi R16, 0x06
		call CMD_WRITE	;termina la configuracion del lcd
		
		call load_z_msg

		medir:
		ldi r16, 0x01
		call CMD_WRITE
		
	
		call show_msg


		LDI R16,0b11000000	;salto al segundo renglon
		CALL CMD_WRITE
		/* INICIA		ADC*/

		LDI R16,0x0F
		OUT DDRC,R16	;PORT C(0-3)    como entrada
		;LDI R16,0x0F
		;OUT PORTC,R16	;RESISTENCIAS PULL-UP no hace falta
		
		
		LDI R16,(1<<REFS0);|(1<<ADLAR)
		STS ADMUX,R16		;Vref=AVcc	y  canal 0 (acd0=pc0)

		LDI R16,0
		STS ADCSRB,R16	;free running mod


		LDI R16,(1<<ADEN)|(1<<ADSC)|(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2)
		STS ADCSRA,R16	;empiezo conversion	a frecuencia  16MHz/128=125KHz


		wait_adc:	;espero a que termine la conversion
			LDS R16,ADCSRA
			SBRS R16,ADIF
			RJMP wait_adc

		LDS R17,ADCL		;parte baja	(2 bits)
		LDS R18,ADCH		;parte alta (8 bits)
		;mandar a una variable en ram
		ldi xh, high(Leido)	;cargo la direccion de Leido en x
		ldi xl, low(Leido)
		st x+,r18		;guardo el resultado de la conversion en Leido
		st x,r17

		lds r17, ADMUX
		sbrc r17, 3
		rcall hex_to_ohm
		sbrs r17, 3
		rcall hex_to_volt
		ldi xh, high(Mostrar)	;cargo la direccion de mostrar en x
		ldi xl, low(Mostrar)
		ld r20,x+
		
		rcall hex_to_ascii
		mov r16,r21
		rcall DATA_WRITE
	
		
		ld r20,x+
		;swap r20
		rcall hex_to_ascii
		mov r16,r21
		rcall DATA_WRITE
		ld r20,x
		;swap r20
		rcall hex_to_ascii
		mov r16,r21
		rcall DATA_WRITE

		rcall DELAY_PRUEBA
		rcall DELAY_PRUEBA
		rcall DELAY_PRUEBA

		jmp medir
		CALL I2C_STOP
		
		JMP LOOP

	LOOP:
		JMP LOOP


	;**********************   Display  ***********************************;
		
I2C_INIT:	
	;Configuro para que la frecuencia del clock sea 10 kHz
	LDI R21,1
	STS TWSR,R21 ; Pongo TWPS1 en 0 y TWPS0 en 1 para que Prescaler Value sea 4
	LDI R21,198
	STS TWBR,R21 ; TWBR vale 198
	LDI R21,(1<<TWEN)
	STS TWCR,R21 ;Este bit (TWEN) sirve para habilitar la comunicación i2c, SCL y SDA para i2c
RET

; *	*	*	*	*	*	*	*	*	*	*	*	*	*	* ;
I2C_START:
	LDI R21, (1<<TWINT)|(1<<TWSTA)|(1<<TWEN) ; cargo 10100100 en r21
	STS TWCR,R21 ; transmite la condición de start
I2C_WAIT_1:
	LDS R21,TWCR
	SBRS R21,TWINT
	RJMP I2C_WAIT_1 ; Se queda en loop hasta que Twint sea 1
RET ; Retorna cuando TWINT sea 1
; *	*	*	*	*	*	*	*	*	*	*	*	*	*	* ;

I2C_STOP:
	LDI R21, (1<<TWINT)|(1<<TWSTO)|(1<<TWEN) ; esto es cargar  10010100 en r21
	STS TWCR,R21 ; Transmite la condición de stop
RET

; *	*	*	*	*	*	*	*	*	*	*	*	*	*	* ;
I2C_WRITE:
	STS TWDR,R27
	LDI R21, (1<<TWINT)|(1<<TWEN)
	STS TWCR,R21
I2C_WAIT_2:
	LDS R21,TWCR
	SBRS R21,TWINT
	RJMP I2C_WAIT_2
RET ; Retorna cuando TWINT sea 1
; *	*	*	*	*	*	*	*	*	*	*	*	*	*	* ;

CMD_WRITE:		;recibe en r16 el comando
	PUSH R27

	MOV R27,R16
	ANDI R27,0XF0	; Se queda con la parte alta del comando
	ORI R27,0b00001000	; Prende el bit de brillo
	CALL I2C_WRITE ; Mando parte alta
	
	ORI R27,0b00000100 ; Prendo el bit de Enable	
	CALL I2C_WRITE ; Mando parte alta con Enable 1
	CALL SDELAY
	ANDI R27,0b11111011 ; Apago el bit de Enable
	CALL I2C_WRITE ; Mando parte alta con Enable 0
	CALL DELAY_100US

	MOV R27,R16
	SWAP R27
	
	ANDI R27,0XF0	; Se queda con la parte alta del comando
	ORI R27,0b00001000	; Prende el bit de brillo
	CALL I2C_WRITE
	
	ORI R27,0b00000100 ; Prendo el bit de Enable	
	CALL I2C_WRITE
	call SDELAY
	ANDI R27,0b11111011 ; Apago el bit de Enable
	CALL I2C_WRITE
	CALL DELAY_100US

	POP R27

RET

DATA_WRITE:        ;recibe en r16 el dato 
	PUSH R27
	MOV R27, R16
	ANDI R27, 0xF0
	ORI R27,0b00001000		
	CALL I2C_WRITE
	
 	ORI R27, 0b00000101		;RS=1 RW=0 EN=1
	CALL I2C_WRITE
	
	CALL SDELAY
	ANDI R27, 0b11111011	;EN=0 for H-to-L pulse
	CALL I2C_WRITE
	
	CALL DELAY_100US

	MOV R27, R16
	SWAP R27
	ANDI R27, 0xF0
	ORI R27, 0b00001001		;
	CALL I2C_WRITE
	
	ORI R27, 0b00000100		;EN = 1 for high pulse
	CALL I2C_WRITE
	
	CALL SDELAY
	ANDI R27, 0b11111011	;EN = 0 pulse H-to-L
	CALL I2C_WRITE
	
	CALL DELAY_100US
	POP R27
RET

;*********************************************************;
DELAY_PRUEBA:	;40m2
	push r17
	ldi R17, 20
LDR1:
	call DELAY_2MS
	dec R17
	brne LDR1
	pop r17
ret
DELAY_2MS:	;2mS
	push R17
	ldi R17, 20
LDR0:
	call DELAY_100US
	dec R17
	brne LDR0
	pop R17
ret
DELAY_100US:	;100 microS
	push R17
	push R18
	ldi R17, 40
	ldi R18, 20
DR0:	
	call SDELAY
	dec R17
	brne DR0
	ldi R17,40
	dec R18
	brne DR0
	pop R18
	pop R17
ret
SDELAY:
	NOP
	NOP
RET
;--------------------ADC---------------------

	hex_to_ascii:
		;rutina para convertir de hexa a ascii
		;entrada nibble bajo de r20
		;salida r21

		andi r20,0x0F	;dejo la parte baja de r20
		cpi r20,0x0A	; comparo con A(10)
		brlo num
		;a-f 
		subi r20,0x0A	;resto 10  ,queda entre 0 y 5
		ldi r21,'A'		;cargo 'A'
		add r21,r20	;sumo , queda entre 0x65 y 0x70
		ret
		num:
			ldi r21,'0'	; cargo '0' en ascii
			add r21,r20	;sumo r20 que está entre 0 y 9
		ret



	ISR_EXT_INT0:
		push r16	;guardo r16
		push r17
		;	eicra= 1<<isc01	flanco des int0
		;	eimsk= 1<<int0
		;
		lds r16, ADMUX
		andi r16, 0x03;	dejo los 3 mux
		inc r16
		cpi r16,3
		brsh change
		ldi r16, 0
		change:
		STS ADMUX,R16	
		rcall change_msg
		reti


		hex_to_volt:
			;guarda en Mostrar el valor convertido de Leido
			;ADLAR = 0
			push r16
			push r20
			push r21
			push r22
			push r23


			ldi xh, high(Leido)	;cargo la direccion de Leido en x
			ldi xl, low(Leido)
			ld r20, x+	;cargo parte alta de leido en r20
			ld r21,x	;cargo parte baja de leido en r21

			ldi xh, high(Mostrar)	;cargo la direccion de Leido en x
			ldi xl, low(Mostrar)

			ldi r23, 0
			mov r16, r21
			loop_v1:
				mov r21, r16
				inc r23
				subi r16, 200
			brsh loop_v1
			cpi r21, 0xaf
			brsh next_v1
			cpi r20,1
			brlo next_v1
			ror r20
			ror r22
			add r21,r22
			rjmp loop_v1
			next_v1:
				dec r23
				st x+,r23	;guardo
				ldi r23,0
				mov r16,r21
			loop_v2:
				mov r21, r16
				inc r23
				subi r16, 20
			brsh loop_v2
			dec r23
			st x+,r23	;guardo
			ldi r23,0
			mov r16,r21
			loop_v3:
				mov r21, r16
				inc r23
				subi r16, 2
			brsh loop_v3
			dec r23
			st x,r23	;guardo
			
			pop r23
			pop r22
			pop r21
			pop r20
			pop r16
		ret

		hex_to_ohm:
			
			push r20
			push r21
			push r22
			push r23
 			push r24

			ldi xh, high(Leido)	;cargo la direccion de Leido en x
			ldi xl, low(Leido)
			ld r20, x+	;cargo parte alta de leido en r20
			ld r21,x	;cargo parte baja de leido en r21

			ldi xh, high(Mostrar)	;cargo la direccion de Leido en x
			ldi xl, low(Mostrar)

			ldi r24,0  ;resultado
			ldi r22,3
			ldi r23,0xff
			sub r22,r20;resto
			sub r23,r21
			cp r22,r20		;1
			brlo ohm_1				
			st x+,r24
			ldi r24,0
			ror r21
			ror r20
			ohm_1:
			cp r22,r20		;2
			brlo ohm_2
			sub r20, r22
			sbc r21,r23
			inc r24
			rjmp ohm_1
			ohm_2:
				st x+,r24
				ldi r24,0
				ror r21
				ror r20
			ohm_3:
			cp r22,r20
			brlo ohm_end
			sub r20, r22
			sbc r21,r23
			inc r24
			rjmp ohm_3
			ohm_end:
			st x,r24
			pop r24
			pop r23
			pop r22
			pop r21
			pop r21
		ret

		change_msg:
		; cambia el mensaje con Z
			push r16
			
			loop_change:
				lpm r16,z+
				cpi r16, '|'
				breq done
				cpi r16, '&'
				breq reset_msg
			rjmp loop_change
			reset_msg:
				rcall load_z_msg
				done:
			pop r16
		ret

		show_msg:
	; tiene que inicializarse z
	; muestra en el display el mensaje en rom
			push r16
			push r17
			push r18
			mov r17,zh
			mov r18, zl	;guardo z inicial
			loop_msg:
			lpm r16,z+	; cargo z en r16

			cpi r16, '|'
			breq end

			cpi r16, '&'
			breq end
			rcall DATA_WRITE
			rjmp loop_msg
		end:
			mov zh, r17
			mov zl, r18
			pop r18
			pop r17
			pop r16
		ret

			cargar:
				rcall DATA_WRITE
			nop
			ret
		
		load_z_msg:
			ldi zh, high(Mensaje<<1)	
			ldi zl, low(Mensaje<<1)
		ret






		Mensaje: .db "Tension  |  Corriente  |  Resistencia  &"