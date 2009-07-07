///\file motori.c
/// This is the main module of Mot&ouml;ri the Plotter firmware.
/// Basic I/O, initialization, interrupt handlers and main program loop are here.
///
/// For the main loop see do_stuff()
///
/// @author Viacheslav Slavinsky
///
///\mainpage Plotter Firmware
/// \section Description
/// This is the firmware for Mot&ouml;ri the Plotter. It consists of basic ATmega644 I/O configuration code, 
/// low-level motor and pen control, basic linear motion algorithms and a scanner/parser that can
/// understand a limited subset of HPGL and translate it into motor steps. Linear motion is achieved by 
/// the means of Bresenham's line algorithm. 
///
/// To prevent shaking and mechanical oscillations motor speed is ramped up and down smoothly.
///
/// In my hardware the motors are very different (one from a Canon printer, other from Epson). The
/// Y-axis motor has lower resolution, or more travel per step. This is why there are not only separate
/// scale coefficients for X and Y axes, but X and Y maximal speeds are also different.
///
/// Since plotter is a very slow device, the communication with computer requires flow control.
/// This implementation uses RTS/CTS flow control, although the plotter wouldn't bother if the host
/// PC can't accept data. 
///
/// \section The Code
///  - motori.h, motori.c	this is the main module with I/O, main loop, interrupts etc
///  - line.h				has the Bresenham's
///  - arc.h				converts an arc description into a series of chords
///  - hpgl.h				scans the input and parses commands
///  - usrat.h				serial i/o
///  - shvars.h				global variables shared among modules
///  - configs.h			global defines
///
/// \section License
/// BSD License. Just use it. It will burn your motors though.
///
/// \section Permanent Location
/// This documentation, source code and other files can be found at http://sensi.org/~svo
///
/// Yours truly,\n
///    Viacheslav Slavinsky
#include "configs.h"

#include <inttypes.h>
#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "usrat.h"
#include "shvars.h"
#include "line.h"
#include "arc.h"
#include "htext.h"
#include "scale.h"
#include "hpgl.h"
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

///< PORTA
#define XSUP	0
#define XINF	1
#define YSUP	2
#define YINF	3
#define ZSUP	4
#define ZINF	5


volatile int16_t motor_pace = MOTOR_PACE_SLOW;	///< current motor pace, to be loaded to OCR0A
volatile int16_t motor_pace_goal;				///< end speed ramp when motor_pace reaches this
volatile uint8_t motor_accel_ctr = 0;			///< step counter for ramping
volatile int8_t  motor_accel = 0;				///< signed, speed ramp direction -1 -> speed up, 1 -> slow down

volatile int16_t accel_strokesteps; 			///< length of current line in motor steps
volatile int16_t accel_decelthresh; 			///< at this step start slowing down

#define sgn(x) ((x)==0?0:((x)<1)?-1:1)

volatile uint8_t pen_status = 255; 			///< pen status: 0 = up
volatile uint8_t pen_relax = 255;

volatile uint8_t pen_pulse_counter = 0;		///< used for servo control

double  alpha, last_alpha, alpha_delta;		///< may be used if SLOW_QUALITY

volatile int16_t msleep_counter = -1;

#define SEEK0_NULL	0
#define SEEK0_SEEK	1
#define SEEK0_DONE 	0x80
volatile uint8_t seeking0;						///< state of seeking home position

void grinding_halt() {	
	printf_P(PSTR("\n\007HALT\n"));
	for(;;);
}

/// Only initialize port directions and pullups
void init_io() {
	DDRC=0xff;	// outputs
	DDRD=0xff;
	DDRB=0xff;
	DDRA=0xff & (~_BV(XSUP) | _BV(YSUP));	// XSUP, YSUP are inputs
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

void msleep_enable(uint8_t enable) {
	if (enable) {
		if (msleep_counter == -1) {
			msleep_counter = SLEEP_COUNTER;
		}
	} else {
		msleep_counter = -1;
	}
}

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

void set_acceleration(ACCEL_MODE accel_mode, uint8_t steep) {
	switch (accel_mode) {
	case ACCEL_FIXSLOW:
		motor_accel = 0;
		motor_pace = MOTOR_PACE_SLOW;
		break;
	case ACCEL_FIXFAST:
		motor_accel = 0;
		motor_pace = steep ? MOTOR_PACE_FASTY : MOTOR_PACE_FASTX;
		break;
	case ACCEL_FIXMEDIUM:
		motor_accel = 0;
		motor_pace = (MOTOR_PACE_SLOW-MOTOR_PACE_FASTX)/2;
		break;
	case ACCEL_ACCEL:
		motor_pace_goal = steep ? MOTOR_PACE_FASTY : MOTOR_PACE_FASTX;
		//printf_P(PSTR("accel_strokesteps=%d"), accel_strokesteps);
		
		// Acceleration with pen up:
		// it can be done much faster than with the pen down because overshooting
		// is not a big problem. But to avoid bad kicks, it's better to keep
		// the speed slower at short distances. X-mass is big.
		if (pen_status == 0) {
			if (steep) {
				motor_pace_goal /= 2;
			} else {
				if (accel_strokesteps < ACCEL_XTHRESH1) {
				} else if (accel_strokesteps < ACCEL_XTHRESH2) { 
					motor_pace_goal /= 2; 
				} else {
					motor_pace_goal /= 4;
				}
			}
		}
		motor_accel_ctr = ACCEL_STEPS_RAMP;
		if (motor_pace != motor_pace_goal) motor_accel = -1;
		break;
	case ACCEL_DECEL:
		motor_pace_goal = MOTOR_PACE_SLOW;
		motor_accel_ctr = ACCEL_STEPS_RAMP;
		if (motor_pace != motor_pace_goal) motor_accel = 1;
		break;
	}
}

double calc_angle(int16_t x2, int16_t y2) {
	alpha = atan2(y2-stepper_loc.y, x2-stepper_loc.x)*180/M_PI;
	double dA = fabs(last_alpha-alpha);
	
	//printf("A=%4.2f, |dA|=%4.2f\n", alpha, dA);
	
	return dA;
}


void pen_control(uint8_t down) {
	if (pen_status != down) {
		while(!motors_ready());
		if (down) {
			_delay_ms(25);
			pen_relax = 255;
			pen_status = 1;
		} else {
			pen_relax = 255;
			pen_status = 0;
		}
	
		PORTD = (PORTD & ~_BV(7)) | (pen_status  ? 0: _BV(7));
		
		_delay_ms(100);
	}
}

void plotter_init() {
	timer0_init();
	timer1_init();
	timer2_init();
	set_acceleration(ACCEL_FIXSLOW, 0);
	
	stepper_loc.x = stepper_loc.y = 0;
	user_loc.x = user_loc.y = 0.0;
	alpha = 0;
	last_alpha = 0;

	text_init();
	translate_init();
	pen_control(0);
	
	seeking0 = SEEK0_NULL;
}


/// Main loop routine.
///
/// Could be re-implemented as a state machine as complexity increases. So far there are only 3 states:
/// 	- Default: receive uart data, pass it to scanner and handle commands
///
///		- Arc tesselation: happens when an arc is being drawn
///
///		- Initialization: waits for the head to get home before doing a complete reset
void do_stuff() {
	STEPPER_COORD dstx, dsty;
	char c;
	uint8_t labelchar;
	uint8_t penny;
	static uint8_t arc_active = 0, initializing = 0, char_active = 0;
	STEPPER_COORD speedup_length, slowdown_length;

	dstx = dsty = -1;
	if (seeking0 != SEEK0_NULL) {
		while (seeking0 != SEEK0_DONE);
		seeking0 = SEEK0_NULL;
		plotter_init();
		printf_P(PSTR("\nHOME\n"));
	} else if (initializing) {
		while(!motors_ready());
		initializing = 0;
		plotter_init();
	} else if (arc_active) {
		arc_active = arc_next(&dstx,&dsty);
	} else if (char_active) {
		char_active = text_char(0, &dstx, &dsty, &penny);
		pen_control(penny);
	} else {
		if (uart_available()) {
			msleep_enable(0);
			motori_sleep(0);
			
			uart_putchar(c = uart_getc());

			switch(hpgl_char(c, &dstx, &dsty, &labelchar)) {
			case CMD_PU:
				pen_control(0);
				break;
			case CMD_PD:
				pen_control(1);
				break;
			case CMD_PA:
				break;
			case CMD_ARCABS:
				arc_active = arc_init();
				break;
			case CMD_INIT:
				// 1. home
				// 2. init scale etc
				dstx = dsty = 0;
				initializing = 1;
				break;
			case CMD_SEEK0:
				seeking0 = SEEK0_SEEK;
				break;
			case CMD_LB0:
				text_beginlabel();
				break;
			case CMD_LB:
				if (labelchar != 0) {
					//printf_P(PSTR("[%c]"), labelchar);
					char_active = text_char(labelchar, &dstx, &dsty, &penny);
				}
				//text_active = 1;
				break;
			case CMD_SI:
				text_scale_cm(numpad[0], numpad[1]);
				break;
			case CMD_SR:
				text_scale_rel(numpad[0], numpad[1]);
				break;
			case CMD_DI:
				text_direction(numpad[0], numpad[1]);
				break;
			default:
				break;
			}
		} else {
			msleep_enable(1);
		}
	}
	
	// have destination, will travel
	if (dstx >= 0  && dsty >= 0) {
#ifdef SLOW_QUALITY			
		last_alpha = alpha;
		alpha_delta = calc_angle(dstx,dsty);
#endif
		
		while(!motors_ready());

#ifdef SLOW_QUALITY
		// wait until the gantry settles down
		if (alpha_delta > 60) {
			_delay_ms(50);
		}
#endif
		//printf_P(PSTR("PRE-PACE: %d; "), motor_pace);
		accel_strokesteps = movestep(dstx, dsty);
		set_acceleration(ACCEL_FIXSLOW, 0);
		set_acceleration(ACCEL_ACCEL, move_is_steep());

		speedup_length = (motor_pace-motor_pace_goal)*ACCEL_STEPS_RAMP;
		
		slowdown_length = (motor_pace-motor_pace_goal)*ACCEL_STEPS_RAMP;
		
		if ((speedup_length + slowdown_length) < accel_strokesteps) {
			accel_decelthresh = accel_strokesteps - slowdown_length;
		} else {
			accel_decelthresh = accel_strokesteps/2;
		}
		
		//printf_P(PSTR("\nMOVE TO: (%d %d) strokesteps=%d dthresh=%d pace_goal=%d start_pace=%d speedup=%d slowdown=%d\n"), 
		//	dstx, dsty, accel_strokesteps, accel_decelthresh, motor_pace_goal, motor_pace, 
		//	speedup_length, slowdown_length);
		//printf_P(PSTR("User: (%5.2f,%5.2f)\n"), user_loc.x, user_loc.y);
	}
}

int main() {
	init_io();

	set_sleep_mode(SLEEP_MODE_IDLE);

    usart_init((F_CPU/(16*115200))-1);
	
    printf_P(PSTR("HEART OF THE INTERSTELLAR LINER\n"));
	
	motori_init();

	plotter_init();
	
	hpgl_init();
	
	sei();
	
	set_acceleration(ACCEL_FIXFAST, 0);
	seeking0 = SEEK0_SEEK;
	
	for (;;) {
		do_stuff();
	}
	
	return 0;
}


/// Timer 0 compare match handler. The motion is here.
///
/// Every time when TCNT0 matches the OCR0A, which corresponds to the currently set motor pace,
/// this handler is invoked. movestep() calls step(), where the actual motor commanding impulses
/// are formed.
///
void SIG_OUTPUT_COMPARE0A( void ) __attribute__ ( ( signal ) );  
void SIG_OUTPUT_COMPARE0A( void ) {
	int16_t stepnumber;
	
	OCR0A = motor_pace;
	
	borderflags = ~PINA & 0x3f;
	
	if (seeking0 == SEEK0_SEEK) {
		if ((borderflags & (_BV(XSUP)|_BV(YSUP))) == (_BV(XSUP)|_BV(YSUP))) {
			seeking0 = SEEK0_DONE;
		}
		step(borderflags & _BV(XSUP) ? 0 : -1, borderflags & _BV(YSUP) ? 0 : -1);
		return;
	}

	//if (!motors_ready()) {
	//	if (motor_pace < (MOTOR_PACE_FASTY/4) || motor_pace > 255) {
	//		printf_P(PSTR("\nERROR\nMotor pace=%d\n"), motor_pace);
	//		grinding_halt();
	//	}
	//}
	
	
	stepnumber = movestep(-1,-1);
	
	if (stepnumber == accel_decelthresh) {
		if (accel_decelthresh != 0) {
			set_acceleration(ACCEL_DECEL, move_is_steep());
		}
	}

	if (stepnumber != -1 && motor_accel) {
		if (--motor_accel_ctr == 0) {
			motor_accel_ctr = ACCEL_STEPS_RAMP; 
			motor_pace += motor_accel;
			if (motor_pace == motor_pace_goal) {
				motor_accel = 0;
			}
		}
	}
}

/// Timer 2 Overflow Interrupt controls the pen-lifting servo.
void SIG_OVERFLOW2( void ) __attribute__ ( ( signal ) );  
void SIG_OVERFLOW2( void ) {
	if (pen_relax != 0) {
		if (pen_pulse_counter == 0) {
			pen_relax--;
			PORTD |= _BV(PENPCM);
		} else if (pen_pulse_counter == 15 && pen_status == 0) {
			PORTD &= ~_BV(PENPCM);
		} else if (pen_pulse_counter == 17) {
			PORTD &= ~_BV(PENPCM);
		}
	}
	pen_pulse_counter = (pen_pulse_counter + 1) & 0x7f;	
}

/// Timer 1 Overflow Interrupt puts the motors to sleep if the host forgets about us
void SIG_OVERFLOW1( void ) __attribute__ ( ( signal ) );  
void SIG_OVERFLOW1( void ) {
	if (msleep_counter == -1) return;

	if (msleep_counter == 0) {
		motori_sleep(1);
	} else {
		--msleep_counter;
	}
}

// $Id$
