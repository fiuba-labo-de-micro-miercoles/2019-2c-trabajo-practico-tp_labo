;micro atmega328p
;fclock=16MHz

.include "m328pdef.inc"
.equ CLEAR_LCD=1
.equ CONFIG_LCD_1=0x33
.equ CONFIG_LCD_2=0x32
.EQU CONFIG_LCD_3=0x28
.equ MOVE_SECOND_LCD=0xc0
.equ MOVE_FIRST_LCD=0x80
.equ MSBITS=0xF0
.equ ENABLE=2
.equ MOD_ERROR= 1
.equ MASK_MUX= 0b11111100
.equ RESET_TIME_0=0xfc
.equ RESET_TIME_1=0xff

.dseg
Leido: .byte 2
Mostrar: .byte 3
Modo: .byte 1

.cseg ; 
	jmp main

.org OC1Aaddr
		rjmp ISR_T0_CP

.org INT_VECTORS_SIZE

main:
		ldi r21,high(RAMEND)
		out SPH,r21
		ldi r21,low(RAMEND)
		out SPL,r21
		ldi r27,0X4E	;	0100 1110=0x4E
		call I2C_INIT
		call I2C_START
		call I2C_WRITE ; Escribo dirección y W/R
		ldi r27,0b00001100	;0x0C
		call DELAY_PRUEBA
		ldi r16, CONFIG_LCD_1
		call CMD_WRITE
		call DELAY_2MS
		ldi r16, CONFIG_LCD_2
		call CMD_WRITE
		call DELAY_2MS
		ldi r16, CONFIG_LCD_3
		call CMD_WRITE
		call DELAY_2MS
		ldi r16, 0x0C
		call CMD_WRITE
		call DELAY_2MS
		ldi r16, 0x01
		call CMD_WRITE
		call DELAY_2MS
		ldi r16, 0x06
		call CMD_WRITE	;termina la configuracion del lcd
		ldi r16, 1; ----configuro el ADC-----------------
		out PORTB, r16		;pull-up port b0
		ldi r16,(1<<REFS0)
		sts ADMUX,r16		;Vref=AVcc	y  canal 0 (acd0=pc0)	ADLAR  en 0
		ldi r16,0
		sts ADCSRB,r16	;free running mod
		ldi r16, RESET_TIME_0			;cargo el valor a comparar L
		sts OCR1AL, r16	
		ldi r16, RESET_TIME_1			;cargo el valor a comparar H
		sts OCR1AH, r16	
		ldi r16 , (1<<OCIE1A)
		sts TIMSK1 , r16			;activo interrupcion por comparacion A
		ldi r16, (1<<WGM12)|(1<<CS12)|(1<<CS10)
		sts TCCR1B , r16	;configuro timer 1	a 16Mhz/1024=15.6KHz modo ctc
		ldi r16 , (1<<SE)
		out  SMCR , r16	; activo sleep mode
loop:
		rcall medir		
		sei ; activo interrupciones
		sleep ; mando a modo idle, hasta la interrupcion
		cli
		rjmp loop

medir:
		call load_mode
		call convert
		call show
		ret
load_mode:
	push r16
	push r17
	ldi xh, high(Modo)	;cargo la direccion de modo en x
	ldi xl, low(Modo)
	ld r17, x
	in  r16 , PINB			;leo puerto b
	andi r16 , ~MASK_MUX	;dejo los 2 ultimos bits
	cp r17, r16	;comparo con el modo anterior
	breq iguales
	st x, r16 ;guardo nuevo modo
	cpi r16, MOD_ERROR		;comparo con convinacion invalida
	breq ERROR_m
	sbrc r16, 0		;traduzco de PINB a MUX
	ldi r16 , 1
	lds r17, ADMUX		;leo ADMUX
	andi r17 , MASK_MUX		;Borro los ultimos bits
	or r17 , r16			;configuro el MUX con el contenido de PINB
	sts ADMUX , r17
	call load_z_msg		;cargo mensaje inicial (corriente) 
	inc r16	;cargo el mensaje
loop_s_msg:
	dec r16
	breq end_loop_s_msg		
	call change_msg		
	rjmp loop_s_msg
ERROR_m:
	rcall ERROR_MSG	;cargo mensaje de error
	rjmp loop
iguales:
	pop r17
	pop r16
	ret
end_loop_s_msg:
	call show_msg		;muestro mensaje cargado
	ldi r16,0b11000000	;salto al segundo renglon
	call CMD_WRITE
	pop r17
	pop r16
	ret
convert:
	ldi r16,0 
	sts PRR, r16;activo el adc
	ldi r16,(1<<ADEN)|(1<<ADSC)|(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2)
	sts ADCSRA,r16	;empiezo conversion	a frecuencia  16MHz/128=125KHz
wait_adc:	;espero a que termine la conversion
	lds r16,ADCSRA
	sbrs r16,ADIF
	rjmp wait_adc
	lds r17,ADCL		;parte baja	(8 bits)
	lds r18,ADCH		;parte alta (2 bits)
	;mandar a una variable en ram
	ldi xh, high(Leido)	;cargo la direccion de Leido en x
	ldi xl, low(Leido)
	st x+,r18		;guardo el resultado de la conversion en Leido
	st x,r17
	lds r17, ADMUX		;leo el modo
	andi r17 , MASK_MUX
	cpi r17, 0
	brne not_ohm
	rcall hex_to_ohm		; modo resistencia
not_ohm:
	rcall hex_to_volt			;modo tension y corriente
	ldi r16, (1<<PRADC)
	sts PRR, r16	;desactivo el adc
	ret
show:
	ldi r16, MOVE_SECOND_LCD
	call CMD_WRITE
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
	ret
delay_m:		;delay de 120 ms
	rcall DELAY_PRUEBA
	rcall DELAY_PRUEBA
	rcall DELAY_PRUEBA
	ret
	call I2C_STOP
	;**********************   Display  ***********************************;	
I2C_INIT:	
	;Configuro para que la frecuencia del clock sea 10 kHz
	ldi r21,1
	sts TWSR,r21 ; Pongo TWPS1 en 0 y TWPS0 en 1 para que Prescaler Value sea 4
	ldi r21,198
	sts TWBR,r21 ; TWBR vale 198
	ldi r21,(1<<TWEN)
	sts TWCR,r21 ;Este bit (TWEN) sirve para habilitar la comunicación i2c, SCL y SDA para i2c
ret
; *	*	*	*	*	*	*	*	*	*	*	*	*	*	* ;
I2C_START:
	ldi r21, (1<<TWINT)|(1<<TWSTA)|(1<<TWEN) ; cargo 10100100 en r21
	sts TWCR,r21 ; transmite la condición de start
I2C_WAIT_1:
	lds r21,TWCR
	sbrs r21,TWINT
	rjmp I2C_WAIT_1 ; Se queda en loop hasta que Twint sea 1
ret ; Retorna cuando TWINT sea 1
; *	*	*	*	*	*	*	*	*	*	*	*	*	*	* ;
I2C_STOP:
	ldi r21, (1<<TWINT)|(1<<TWSTO)|(1<<TWEN) ; esto es cargar  10010100 en r21
	sts TWCR,r21 ; Transmite la condición de stop
ret
; *	*	*	*	*	*	*	*	*	*	*	*	*	*	* ;
I2C_WRITE:
	sts TWDR,r27
	ldi r21, (1<<TWINT)|(1<<TWEN)
	sts TWCR,r21
I2C_WAIT_2:
	lds r21,TWCR
	sbrs r21,TWINT
	rjmp I2C_WAIT_2
ret ; Retorna cuando TWINT sea 1
; *	*	*	*	*	*	*	*	*	*	*	*	*	*	* ;
CMD_WRITE:		;recibe en r16 el comando
	push r27
	mov r27,r16
	andi r27,0XF0	; Se queda con la parte alta del comando
	ori r27,0b00001000	; Prende el bit de brillo
	call I2C_WRITE ; Mando parte alta
	ori r27,(1<<ENABLE) ; Prendo el bit de Enable	
	call I2C_WRITE ; Mando parte alta con Enable 1
	call SDELAY
	andi r27,~(1<<ENABLE) ; Apago el bit de Enable
	call I2C_WRITE ; Mando parte alta con Enable 0
	call DELAY_100US
	mov r27,r16
	swap r27
	andi r27,0XF0	; Se queda con la parte alta del comando
	ori r27,0b00001000	; Prende el bit de brillo
	call I2C_WRITE
	ori r27,(1<<ENABLE) ; Prendo el bit de Enable	
	call I2C_WRITE
	call SDELAY
	andi r27,~(1<<ENABLE) ; Apago el bit de Enable
	call I2C_WRITE
	call DELAY_100US
	pop r27
ret

DATA_WRITE:        ;recibe en r16 el dato 
	push r27
	mov r27, r16
	andi r27, MSBITS
	ori r27,0b00001000		
	call I2C_WRITE
 	ori r27, 0b00000101		;RS=1 RW=0 EN=1
	call I2C_WRITE
	call SDELAY
	andi r27, ~(1<<ENABLE)	;EN=0 for H-to-L pulse
	call I2C_WRITE
	call DELAY_100US
	mov r27, r16
	swap r27
	andi r27, MSBITS
	ori r27, 0b00001001		;
	call I2C_WRITE
	ori r27, (1<<ENABLE)		;EN = 1 for high pulse
	call I2C_WRITE
	call SDELAY
	andi R27, ~(1<<ENABLE)	;EN = 0 pulse H-to-L
	call I2C_WRITE
	call DELAY_100US
	pop r27
ret
;*********************************************************;
DELAY_PRUEBA:	;40m2
	push r17
	ldi r17, 20
LDR1:
	call DELAY_2MS
	dec r17
	brne LDR1
	pop r17
ret
DELAY_2MS:	;2mS
	push r17
	ldi r17, 20
LDR0:
	call DELAY_100US
	dec r17
	brne LDR0
	pop r17
ret
DELAY_100US:	;100 microS
	push r17
	push r18
	ldi r17, 40
	ldi r18, 20
DR0:	
	call SDELAY
	dec r17
	brne DR0
	ldi r17,40
	dec r18
	brne DR0
	pop r18
	pop r17
ret
SDELAY:
	nop
	nop
ret
;--------------------ADC---------------------
hex_to_ascii:
	;rutina para convertir de hexa a ascii
	;entrada nibble bajo de r20
	;salida r21
	andi r20,~MSBITS	;dejo la parte baja de r20
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
comp_1:		
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
change_msg:	; cambia el mensaje con Z
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
	ldi r16,1	;clear
	call CMD_WRITE
	ldi r16 ,MOVE_FIRST_LCD
	call CMD_WRITE
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
	ldi r16,1	;clear
	call CMD_WRITE
	ldi r16 ,MOVE_FIRST_LCD
	call CMD_WRITE
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
		;guarda en Mostrar el valor convertido de Leido  ADLAR = 0
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
	ldi r23, 0		;resultado		
loop_v1:
	cpi r21,200		;comparo con 1V 
	brsh next_v1	;si es mayor paso a restar
	dec r20				;si no decremento 
	brmi loop_v2	;si r20 es cero y r21 es menor a 1V, paso al siguiente loop 
next_v1:
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

ISR_T0_CP:
	nop
	reti