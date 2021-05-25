
; Copyright (c) 2021 ozforester. All rights reserved.
; Use of this source code is goverened by a MIT license
; that can be found in the LICENSE file.

; ATmega8 @4MHz internal osc. (H:D9, L:23)
;
; scans twi bus for pcf8574 extension
; Use pcf8574A quasi-bidirectional twi(i2c) extension
; Led D7 signals any twi error.
;
; Checks a button via twi extension
; and turns a led if it holds

#include <avr/io.h>

#undef  __SFR_OFFSET
#define __SFR_OFFSET 0
#define START 	  0x08
#define REP_START 0x10
#define SLAW_ACK  0x18
#define SLAW_NACK 0x20
#define DATA_ACK  0x28
#define DATA_NACK 0x30

temp    = 16			; common reg
counter = 17			; delay counter reg
address = 19			; twi address reg
data    = 20			; twi data reg

.section .text			; code section
.global __vector_default        ;
.global TIMER0_OVF_vect         ;
.global main                    ;

main:				;
	rcall timer_init	; timer0
	rcall twi_init		; 100kHz prescaled
	rcall twi_scan		; goes down 127..0 (0 = no answer)
        cpi address, 0          ; scan result
        brne main1              ; pcf answered
	rcall error		; error led
	main1:			; main main )
	rcall pcf_init		; R
	rcall pcf_led_on_off	; blink led P7 pcf8574
	loop:			;
	rjmp loop		; waste a time
ret				;

pcf_led_on_off:			;
	pcfled:			;
	sbr data, 1<<3		; P7 p-u (off)
	rcall twi_start		;
	rcall twi_sla_w		;
	rcall twi_data_w	;
        rcall twi_stop          ;
        rcall delay_1s          ; delay
        rcall twi_start         ;
        rcall twi_sla_w         ;
        cbr data, 1<<3          ; P7 sink (on)
        rcall twi_data_w        ;
        rcall twi_stop          ;
        rcall delay_1s          ; delay
	rjmp pcf_led_on_off	;
ret				;

pcf_init:			; R (no sink)
        rcall twi_start         ;
        rcall twi_sla_w         ;
	ser data		;
	rcall twi_data_w	;
        rcall twi_stop          ;
ret				;

twi_scan:			;
	ldi address, 127	; will step backward
	twscan1:		;
	rcall twi_start		; start/rep_start
	rcall twi_sla_w		; slave addres + w
	cpi temp, 0xFF		; touched?
	brne twscan2		; if not cleared
	clr temp		; (temp=0x00)
	subi address, 1		; else next address
	cpi address, 0		; last address
	breq twscan2		; exit with zero address
	rjmp twscan1		;
	twscan2:		;
	rcall twi_stop		;
ret				;

twi_data_w:					;
	mov temp, data				;
	out TWDR, temp				; data
	ldi temp, ((1<<TWINT)|(1<<TWEN))	;
	out TWCR, temp				; go
	tdw1:					;
	in temp, TWCR				;
	sbrs temp, TWINT			; sent?
	rjmp tdw1				; not yet, repeat
	in temp, TWSR				; ok, read status
	andi temp, 0xF8				;
	cpi temp, DATA_ACK			;
	breq tdw2				; ok
        rcall error	                        ; led
	tdw2:					;
ret						;

twi_sla_w:					;
	mov temp, address			; slave address (e.g. 0x3F as pcf of mine)
	lsl temp				; address<<  + 0, R/W = W
	out TWDR, temp				;
	ldi temp, ((1<<TWINT)|(1<<TWEN))	;
	out TWCR, temp				; go
	slaw1:					;
        in temp, TWCR                           ;
        sbrs temp, TWINT                        ; sent
        rjmp slaw1                              ;
        in temp, TWSR                           ; read status
        andi temp, 0xF8                         ;
        cpi temp, SLAW_ACK                      ;
	breq slaw2				; acked (stay temp untouched)
	;rcall error				; uncomment if used w/o autodetect
	ser temp				; else (temp=0xFF)
	slaw2:					;
ret						;

twi_start:						;
	ldi temp, ((1<<TWINT)|(1<<TWSTA)|(1<<TWEN))	;
	out TWCR, temp					; go
	start1:						;
	in temp, TWCR					;
	sbrs temp, TWINT				; sent
        rjmp start1                                     ;
	in temp, TWSR					; read status
	andi temp, 0xF8					;
	cpi temp, START					; start acked?
	breq start2					;
        cpi temp, REP_START                             ;
        breq start2                                     ; maybe rep. acked?
	rcall error					; error led
	start2:						;
ret							; acked

twi_stop:						;
	ldi temp, ((1<<TWINT)|(1<<TWEN)|(1<<TWSTO))	;
        out TWCR, temp                                  ;
ret							;

twi_init:			; 100 kHz @4MHz clock
	ldi temp, 0b00001100	; 4000000 / ( 16 + 2*TWBR * 4^TWPS )
	out TWBR, temp		; 12
        in temp, TWSR	        ;
	ori temp, 0b00000011	; 64
	out TWSR, temp		;
ret				;

timer_init:			; TIMER0 (65 ms @4MHz)
        ldi temp, (1<<CS00)     ;
        ori temp, (1<<CS02)     ;
        out TCCR0, temp         ;
        ldi temp, (1<<TOIE0)    ;
        out TIMSK, temp         ;
        sei                     ;
ret				;

error:  sbi DDRD, 7             ; D7 out
        sbi PORTD, 7            ; high
        err: rjmp err           ; privet medved
ret                             ;

delay_1s:			;
        rcall delay_500ms       ;
	rcall delay_500ms	;
ret

delay_500ms:			;
        rcall delay_100us       ;
        rcall delay_100us       ;
        rcall delay_100us       ;
        rcall delay_100us       ;
	rcall delay_100us	;
ret				;

delay_100us:                    ;
	ldi counter, 4		;
	delay:			;
        cpi counter, 0          ;
        brne delay              ;
ret                             ;

TIMER0_OVF_vect:                ;
        cli                     ;
        push temp               ; save temp
        in temp, SREG           ; save SREG
        push temp		;
        ldi temp, 155           ; 25 us
        out TCNT0, temp         ;
        subi counter, 1         ; delay counter--
        pop temp		;
        out SREG, temp          ; restore SREG
        pop temp                ; restore temp
        sei                     ;
reti                            ;

__vector_default:	reti	;

.end
