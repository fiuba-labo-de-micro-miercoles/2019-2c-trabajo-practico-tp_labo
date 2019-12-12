;micro atmega328p
;fclock=16MHz

;Display con I2C
;fclock=16MHz


.include "m328pdef.inc"
.EQU CLEAR_LCD=1
.EQU CONFIG_LCD_1=0x33
.EQU CONFIG_LCD_2=0x32
.EQU CONFIG_LCD_3=0x28
.EQU MSBITS=0xF0
.EQU ENABLE=2
.EQU MOD_ERROR= 1
.EQU MASK_MUX= 0b11111100


.dseg
Leido: .byte 2
Mostrar: .byte 3


.CSEG ; 
	JMP MAIN



.ORG INT_VECTORS_SIZE

MAIN:
		LDI R21,HIGH(RAMEND)
		OUT SPH,R21
		LDI R21,LOW(RAMEND)
		OUT SPL,R21

		LDI R27,0X4E	;	0100 1110=0x4E

	
	;call hex_to_volt

		CALL I2C_INIT
		CALL I2C_START
		CALL I2C_WRITE ; Escribo dirección y W/R
	
		LDI R27,0b00001100	;0x0C

		call DELAY_PRUEBA
		ldi R16, CONFIG_LCD_1
		call CMD_WRITE
		call DELAY_2MS
		ldi R16, CONFIG_LCD_2
		call CMD_WRITE
		call DELAY_2MS
		ldi R16, CONFIG_LCD_3
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

		; ----configuro el ADC-----------------

		
		;LDI R16,0x0F
		;OUT PORTC,R16	;RESISTENCIAS PULL-UP no hace falta


		ldi r16, 1
		out PORTB, r16		;pull-up port b0
		
		LDI R16,(1<<REFS0)
		STS ADMUX,R16		;Vref=AVcc	y  canal 0 (acd0=pc0)	ADLAR  en 0

		LDI R16,0
		STS ADCSRB,R16	;free running mod


		

		call load_z_msg	;cargo mensaje inicial (corriente)
		

		medir:
		
		ldi r16, 0x01
		call CMD_WRITE
		
		call load_z_msg		;;cargo mensaje inicial (corriente) 
		in  r16 , PINB			;leo puerto b
		andi r16 , ~MASK_MUX	;dejo los 2 ultimos bits
		cpi r16, MOD_ERROR		;comparo con convinacion invalida
		breq ERROR_m
		
		sbrc r16, 0		;traduzco de PINB a MUX
		ldi r16 , 1
		lds r17, ADMUX		;leo ADMUX
		andi r17 , MASK_MUX		;Borro los ultimos bits
		or r17 , r16			;configuro el MUX con el contenido de PINB
		sts ADMUX , r17

		
		inc r16	;cargo el mensaje
		loop_s_msg:
			dec r16
			breq end_loop_s_msg		
			call change_msg		
		rjmp loop_s_msg

		ERROR_m:
			rcall ERROR_MSG	;cargo mensaje de error
			rjmp delay_m

	end_loop_s_msg:
		call show_msg		;muestro mensaje cargado


		LDI R16,0b11000000	;salto al segundo renglon
		CALL CMD_WRITE


		/* INICIA		ADC*/
		LDI R16,0
		STS ADCSRB,R16	;free running mod

		LDI R16,(1<<ADEN)|(1<<ADSC)|(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2)
		STS ADCSRA,R16	;empiezo conversion	a frecuencia  16MHz/128=125KHz


		wait_adc:	;espero a que termine la conversion
			LDS R16,ADCSRA
			SBRS R16,ADIF
		RJMP wait_adc

		LDS R17,ADCL		;parte baja	(8 bits)
		LDS R18,ADCH		;parte alta (2 bits)
		;mandar a una variable en ram
		ldi xh, high(Leido)	;cargo la direccion de Leido en x
		ldi xl, low(Leido)
		st x+,r18		;guardo el resultado de la conversion en Leido
		st x,r17

		lds r17, ADMUX		;leo el modo
		andi r17 , MASK_MUX
		cpi r17, 0
		brne not_ohm
		;sbrs r17, 1
		rcall hex_to_ohm		; modo resistencia
		;sbrc r17, 1
		not_ohm:
		rcall hex_to_volt			;modo tension y corriente

		ldi xh, high(Mostrar)	;cargo la direccion de mostrar en x
		ldi xl, low(Mostrar)

		ld r20,x+			;cargo 1er digito
		rcall hex_to_ascii
		mov r16,r21
		rcall DATA_WRITE
		
		ldi r20, 0b10100101		;cargo un punto
		mov r16,r21
		rcall DATA_WRITE

		ld r20,x+			;cargo 2do digito
		rcall hex_to_ascii
		mov r16,r21
		rcall DATA_WRITE

		ld r20,x		;cargo 3er digito
		rcall hex_to_ascii
		mov r16,r21
		rcall DATA_WRITE

		delay_m:		;delay de 120 ms
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


		hex_to_ohm:
			
			push r20
			push r21
			push r22
			push r23
 			push r24
			push r16

			ldi xh, high(Leido)	;cargo la direccion de Leido en x
			ldi xl, low(Leido)
			ld r20, x+	;cargo parte alta de leido en r20
			ld r21,x	;cargo parte baja de leido en r21

			ldi xh, high(Mostrar)	;cargo la direccion de Leido en x
			ldi xl, low(Mostrar)
			cpi r20, 1
			brlo not_inf
			cpi r21, 0x9a		;comparo con el valor maximo posible (2V)
			brlo not_inf
			ldi r24, 9
			st x+,r24
			st x+,r24
			st x,r24
			ret

			not_inf:
			ldi r24,0  ;resultado
			ldi r22,3	;H
			ldi r23,0xff	;L
			sub r22,r20;resto
			sub r23,r21

			cp r20, r22		;comparo, si r20 menor a r22 , cargo un 0 y multiplico
			brsh ohm_1
		;	cp r20, r22	;si r20=r22 , comparo r21 y r23
		;	breq comp_1
		;	rjmp ohm_1		;paso a restar
			comp_1:
		;	cp r21,r23
		;	brlo ohm_1				
			st x+,r24
			ldi r24,10
			mov r16, r20 ; guardo r20
			mul r21, r24	;	multiplico por 10
			mov r21 ,r0		;L
			mov r20, r1		;H
			mul r16, r24	
			add r20 , r0		;sumo
			ldi r24,0
			ohm_1:
				cp r20,r22		;comparo , si es menor cargo 0
				brsh comp_2		;
				rjmp ohm_2
				comp_2:
				cp r20 , r22
				brne restar_1		;si r20 mayor a r22 resto
				cp r21,r23			;si r20 = r22 comparo r21 y r23
				brlo ohm_2
				restar_1:
				sub r21,r23
				sbc r20, r22
				inc r24
			rjmp ohm_1
			ohm_2:
				st x+,r24	;cargo 
				ldi r24,10
				mov r16 , r20
				mul r21,r24	;	multiplico por 10
				mov r21, r0		;L
				mov r20, r1		;H
				mul r16, r24	
				add r20 , r0		;sumo
				ldi r24, 0
			ohm_3:
				cp r22,r20
				brlo ohm_end
				sub r21,r23
				sbc r20, r22		;resto
				cp r21,r23
				brlo ohm_end
				sbc r21,r23
				inc r24
				rjmp ohm_3
			ohm_end:
				st x,r24
				pop r16
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

		ERROR_msg:
		; muestra en el display el mensaje en rom
			push r16
			push r17
			push r18
			mov r17,zh
			mov r18, zl	;guardo z inicial

			ldi zh, high(Mensaje_ERROR<<1)	
			ldi zl, low(Mensaje_ERROR<<1)	;cargo puntero z
			loop_msg_error:
			lpm r16 , z+ ; cargo z en r16

			cpi r16, '&'
			breq end_error
			rcall DATA_WRITE
			rjmp loop_msg_error
		end_error:
			mov zh, r17
			mov zl, r18
			pop r18
			pop r17
			pop r16
		ret
			


		Mensaje: .db "  Resistencia | Corriente  |  Tension  &"
		Mensaje_ERROR: .db "ERROR  &"

			hex_to_volt:
			;guarda en Mostrar el valor convertido de Leido
			;ADLAR = 0
			push r20
			push r21
			push r22
			push r23
			


			ldi xh, high(Leido)	;cargo la direccion de Leido en x
			ldi xl, low(Leido)
			ld r20, x+	;cargo parte alta de leido en r20
			ld r21,x	;cargo parte baja de leido en r21
			;test
			;ldi r20 , 2
			;------
			ldi xh, high(Mostrar)	;cargo la direccion de Leido en x
			ldi xl, low(Mostrar)

			ldi r23, 0		;resultado
			
			loop_v1:
				cpi r21,200		;comparo con 1V 
				brsh next_v1	;si es mayor paso a restar
				dec r20				;si no decremento 
				brmi loop_v2	;si r20 es cero y r21 es menor a 1V, paso al siguiente loop 
				next_v1:
				;mov r16 ,r21		;guardo el anterior
				inc r23				
				subi r21, 200
			rjmp loop_v1

			loop_v2:
				st x+,r23	;guardo
				ldi r23,0

			next_v2:
				cpi r21,20		;comparo con 100mV 
				brlo loop_v3	;si es mayor paso a restar

				inc r23				
				subi r21, 20
			rjmp next_v2

			loop_v3:
			st x+,r23	;guardo
			ldi r23,0

			next_v3:
				cpi r21,2		;comparo con 10mV 
				brlo end_v	;si es mayor paso a restar
				inc r23				
				subi r21, 2
			rjmp next_v3
			end_v:
			st x,r23	;guardo

			
			pop r23
			pop r22
			pop r21
			pop r20
		ret
