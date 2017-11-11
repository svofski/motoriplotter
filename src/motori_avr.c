#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>

#include "configs.h"
#include "motori.h"

// I/O Pins
#define MRESET 2		///< PORTB.2 is motor reset (global)
#define MSLEEP 1		///< PORTB.1 is motor sleep (global)
#define MENABLE 0		///< PORTB.0 is motor enable (global)

#define DIR 0			///< PORTC.0, PORTC.4	X/Y dir
#define STEP 1			///< PORTC.1, PORTC.5	X/Y step
#define MS1 2			///< PORTC.2, PORTC.6	X/Y microstep select 1
#define MS2 3			///< PORTC.3, PORTC.7	X/Y microstep select 2

#define PENPCM 6		///< PORTD.6 pen servo control (1.5ms zero)

volatile uint8_t pen_relax = 255;
volatile uint8_t pen_pulse_counter = 0;		        ///< used for servo control

/* Callbacks defined in motori.c */
void msleep_enable(uint8_t enable);
void msleep_callback(void);
void motion_callback(uint8_t borderflags);
uint8_t get_pen_status(void);
volatile int16_t get_motor_pace(void);

/// Only initialize port directions and pullups
void init_io() {
	DDRC=0xff;	// outputs
	DDRD=0xff;
	DDRB=0xff;
	DDRA=0xff & (~_BV(XSUP) | _BV(YSUP));	        // XSUP, YSUP are inputs
	PORTA = _BV(XSUP)|_BV(YSUP);			// enable pull-up on XSUP,YSUP
	PORTB = 0x00;
	PORTC = 0x00;
	PORTD = 0x00;
}

/// Initialize Timer 0. 
/// Timer0 is used for stepper control and speed ramping.
/// @see SIG_OUTPUT_COMPARE0A
void timer0_init() {
	TCCR0A = 2; // CTC
	TCCR0B = 4; // clk/256 prescaler
	TIMSK0 = _BV(OCIE0A);
	OCR0A = 0xff;
}

/// Initialize sleep control timer
/// @see SLEEP_PERIOD
void timer1_init() {
	msleep_enable(0);
	
	TCCR1A = 0;		// normal mode, disconnect all
	TCCR1B = 5;		// clk/1024
	TCNT1 = SLEEP_PERIOD;
	TIMSK1 = _BV(TOIE1);
}


/// Initialize Timer 2.
/// Timer2 is used to control the pen-lifting servo.
/// @see SIG_OVERFLOW2
void timer2_init() {
	TCCR2A = 0x00; // clear OC2B on compare match, normal mode
	TCCR2B = 0x02; // clk/8 prescaler
	TIMSK2 = _BV(TOIE2);
}

/// Initialize motor controllers: wake up, enable, reset and wait 100ms.
void motori_init() {
	PORTB |= _BV(MSLEEP);
	PORTB &= ~(_BV(MENABLE) | _BV(MRESET));
	_delay_ms(20);
	PORTB |= _BV(MRESET);
	_delay_ms(100);
}

/// Enable motors
/// @param enable true if motors should be enabled
void motori_enable(uint8_t enable) {
	if (enable) {
		PORTB &= ~(_BV(MENABLE) | _BV(MRESET));
		PORTB |= _BV(MRESET);
	} else {
		PORTB |= _BV(MENABLE);
	}
}


/// Put the motors to sleep. Waits 1ms after sleep has ended.
/// @param zzz true if sleeping
void motori_sleep(uint8_t zzz) {
	if (zzz) {
		if ((PORTB & _BV(MSLEEP)) != 0) {
			printf_P(PSTR("\nSLEEP\n"));
			PORTB &= ~_BV(MSLEEP);
		}
	} else {
		if ((PORTB & _BV(MSLEEP)) == 0) {
			printf_P(PSTR("\nWAKE\n"));
			PORTB |= _BV(MSLEEP);
			_delay_ms(1);
		}
	}
}


// issue the actual step commands to stepper controllers
// direction +1/-1, 0 == stand still
void step(int8_t xdir, int8_t ydir) {
	// permanently keeps MS1+MS2 for both motori
	PORTC  = 0xcc; 
	PORTC |= (ydir < 0 ? 0 : _BV(DIR+4)) | (xdir < 0 ? 0 : _BV(DIR));
	
	// issue step signals
	PORTC |= (ydir ? _BV(STEP+4) : 0) | (xdir ? _BV(STEP) : 0);	
}

void global_enable_interrupts()
{
    sei();
}

void global_set_sleep_mode_idle()
{
    set_sleep_mode(SLEEP_MODE_IDLE);
}

void pen_solenoid_update()
{
    PORTD = (PORTD & ~_BV(7)) | (get_pen_status()  ? 0: _BV(7));
}

void pen_servo_set(uint8_t value)
{
    pen_relax = value;
}

void do_idle()
{
}

/// Timer 0 compare match handler. The motion is here.
///
/// Every time when TCNT0 matches the OCR0A, which corresponds to the currently set motor pace,
/// this handler is invoked. movestep() calls step(), where the actual motor commanding impulses
/// are formed.
///
ISR(TIMER0_COMPA_vect) {
	OCR0A = get_motor_pace();
	uint8_t borderflags = ~PINA & 0x3f;
        motion_callback(borderflags);
}

/// Timer 2 Overflow Interrupt controls the pen-lifting servo.
ISR(TIMER2_OVF_vect) {
	if (pen_relax != 0) {
		if (pen_pulse_counter == 0) {
			pen_relax--;
			PORTD |= _BV(PENPCM);
		} else if (pen_pulse_counter == 15 && get_pen_status() == 0) {
			PORTD &= ~_BV(PENPCM);
		} else if (pen_pulse_counter == 16) {
			PORTD &= ~_BV(PENPCM);
		}
	}
	pen_pulse_counter = (pen_pulse_counter + 1) & 0x7f;	
}

/// Timer 1 Overflow Interrupt puts the motors to sleep if the host forgets about us
ISR(TIMER1_OVF_vect) {
    msleep_callback();
}

