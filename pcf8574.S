
; Copyright (c) 2021 ozforester. All rights reserved.
; Use of this source code is goverened by a MIT license
; that can be found in the LICENSE file.

; ATmega8 @4MHz internal osc. (H:D9, L:23)
;
; scans twi bus for pcf8574 extension
; Use pcf8574A quasi-bidirectional twi(i2c) extension
; bus pulled-up with 8k2 (2k2 take a walk)
; Led D7 signals any twi error.
;
; Checks a button via twi extension and turns a led

#include <avr/io.h>

#undef  __SFR_OFFSET
#define __SFR_OFFSET  0
#define START 	   0x08
#define REP_START  0x10
#define SLAW_ACK   0x18
#define DATAW_ACK  0x28
#define SLAR_ACK   0x40
#define DATAR_ACK  0x50
#define DATAR_NACK 0x58

#define P0 0
#define P1 1
#define P2 2
#define P3 3
#define P4 4
#define P5 5
#define P6 6
#define P7 7

temp    = 16			; common reg
counter = 17			; delay reg
address = 18			; twi address reg
data    = 19			; twi mirror reg

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
	rcall pcf_init		; small p-u all pins
	rcall pcf_read_button	; and press a button
	loop:			;
	rjmp loop		; waste a time
ret				;

pcf_read_button:                ;
	sbr data, 1<<P7		; led pin pull-up (off)
        sbr data, 1<<P6         ; button pin pull-up (may be pulled down with a button)
	rcall pcf_write		; write a new state
	prb:			;
        rcall pcf_read          ; first read 1 but after one press is always 0
        sbrc data, P6           ; skip if pulled up
        rjmp prb               	;
        rcall pcf_led_on        ; led on if pulled down
	rcall delay_1s		;
        rcall pcf_led_off       ; led off
        rjmp pcf_read_button    ; loop
ret                             ;

pcf_led_off:                    ;
        sbr data, 1<<P7         ; P7 p-u (off)
	rcall pcf_write		;
ret                             ;

pcf_led_on:                     ;
        cbr data, 1<<P7         ; P7 sink (on)
        rcall pcf_write         ;
ret                             ;

pcf_write:			;
        rcall twi_start         ;
        rcall twi_sla_w         ;
        rcall twi_data_w        ; data >> TWDR
        rcall twi_stop          ;
ret				;

pcf_read:                       ;
        rcall twi_start         ;
        rcall twi_sla_r         ;
        rcall twi_data_r        ; data << TWDR
        rcall twi_stop          ;
ret                             ;

pcf_init:			; all 00000000 (sink)
        rcall twi_start         ;
        rcall twi_sla_w         ;
	clr data		; 0x00
	rcall twi_data_w	;
        rcall twi_stop          ;
ret				;

twi_scan:			;
	ldi address, 127	; goes down
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

twi_data_r:                                     ;
        ldi temp, ((1<<TWINT)|(1<<TWEN))        ; reset flag +nack
        out TWCR, temp                          ;
        tdr1:                                   ;
        in temp, TWCR                           ; check if
        sbrs temp, TWINT                        ; flag set?
        rjmp tdr1                               ; not yet, repeat
	in data, TWDR				; data<-
ret                                             ;

twi_data_w:					;
        tdw1:                                   ;
        in temp, TWCR                           ;
        sbrs temp, TWINT                        ; ready?
        rjmp tdw1                               ; not yet, repeat
	out TWDR, data				; data->
	ldi temp, ((1<<TWINT)|(1<<TWEN))	;
	out TWCR, temp				; go
	tdw2:					;
	in temp, TWCR				;
	sbrs temp, TWINT			; sent?
	rjmp tdw2				; not yet, repeat
	in temp, TWSR				; check status
	andi temp, 0xF8				;
	cpi temp, DATAW_ACK			; 0x28
	breq tdw3				; ok
        rcall error	                        ; led
	tdw3:					;
ret						;

twi_sla_r:                                      ;
        mov temp, address                       ; slave address (e.g. 0x3F as pcf of mine)
        lsl temp                                ; <<address + 0, R/W = W
	ori temp, 0x01				; +1, R/W = R
        out TWDR, temp                          ;
        ldi temp, ((1<<TWINT)|(1<<TWEN))        ;
        out TWCR, temp                          ; go
        slar1:                                  ;
        in temp, TWCR                           ;
        sbrs temp, TWINT                        ; sent
        rjmp slar1                              ;
        in temp, TWSR                           ; read status
        andi temp, 0xF8                         ;
        cpi temp, SLAR_ACK                      ; 0x40
        breq slar2                              ; acked
        rcall error                             ;
        slar2:                                  ;
ret                                             ;

twi_sla_w:					;
	mov temp, address			; slave address (e.g. 0x3F as pcf of mine)
	lsl temp				; <<address + 0, R/W = W
	out TWDR, temp				;
	ldi temp, ((1<<TWINT)|(1<<TWEN))	;
	out TWCR, temp				; go
	slaw1:					;
        in temp, TWCR                           ;
        sbrs temp, TWINT                        ; sent
        rjmp slaw1                              ;
        in temp, TWSR                           ; read status
        andi temp, 0xF8                         ;
        cpi temp, SLAW_ACK                      ; 0x18
	breq slaw2				; acked (stay temp untouched if scanned)
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
	cpi temp, START					; start acked? 0x08
	breq start2					;
        cpi temp, REP_START                             ; 0x10
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

error:  sbi DDRD, 7             ; D7 out
        sbi PORTD, 7            ; high
        err: rjmp err           ; privet medved
ret                             ;

delay_1s:			;
        rcall delay_500ms       ;
	rcall delay_500ms	;
ret

delay_500ms:			;
        rcall delay_100ms       ;
        rcall delay_100ms       ;
        rcall delay_100ms       ;
        rcall delay_100ms       ;
	rcall delay_100ms	;
ret				;

delay_100ms:                    ;
	ldi counter, 4		;
	delay:			;
        cpi counter, 0          ;
        brne delay              ;
ret                             ;

timer_init:                     ; TIMER0 (ovfl in 65 ms @4MHz)
        ldi temp, (1<<CS00)     ;
        ori temp, (1<<CS02)     ;
        out TCCR0, temp         ;
        ldi temp, (1<<TOIE0)    ;
        out TIMSK, temp         ;
        sei                     ;
ret                             ;

TIMER0_OVF_vect:                ;
        cli                     ;
        push temp               ; save temp
        in temp, SREG           ; save SREG
        push temp		;
        ldi temp, 159           ; (ovfl in 25 ms)
        out TCNT0, temp         ;
        subi counter, 1         ; delay counter--
        pop temp		;
        out SREG, temp          ; restore SREG
        pop temp                ; restore temp
        sei                     ;
reti                            ;

__vector_default:	reti	;

; "Lord, help us"

.end

