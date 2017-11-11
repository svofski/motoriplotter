///\file motori.c
/// This is the main module of Mot&ouml;ri the Plotter firmware.
/// Basic I/O, initialization, interrupt handlers and main program loop are here.
///
/// For the main loop see do_stuff()
///
/// @author Viacheslav Slavinsky
///
/// \mainpage Plotter Firmware
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
/// See the Files section for the complete reference.
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

#include <motori_hw.h>

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
#include "path.h"


volatile int16_t motor_pace = MOTOR_PACE_SLOW;	        ///< current motor pace, to be loaded to OCR0A
volatile int16_t motor_pace_goal;			///< end speed ramp when motor_pace reaches this
volatile uint8_t motor_accel_ctr = 0;			///< step counter for ramping
volatile int8_t  motor_accel = 0;			///< signed, speed ramp direction -1 -> speed up, 1 -> slow down

volatile int16_t accel_strokesteps; 			///< length of current line in motor steps
volatile int16_t accel_decelthresh; 			///< at this step start slowing down
volatile uint8_t accel_mode = 1;			///< 0 = constant speed, see AS command

#define sgn(x) ((x)==0?0:((x)<1)?-1:1)

volatile uint8_t pen_status = 255; 			///< pen status: 0 = up


double  alpha, last_alpha, alpha_delta;		        ///< may be used if SLOW_QUALITY

volatile int16_t msleep_counter = -1;

#define SEEK0_NULL	0
#define SEEK0_SEEK	1
#define SEEK0_DONE 	0x80
volatile uint8_t seeking0;				///< state of seeking home position

volatile uint16_t speed_skip = SPEED_SKIP;		///< only perform stepping once in this many times
volatile uint16_t speed_skip_ctr = 1;

int32_t path_stepnumber;

void grinding_halt() {	
	printf_P(PSTR("\n\007HALT\n"));
	for(;;);
}

/// Enable/disable entering sleep mode
void msleep_enable(uint8_t enable) {
	if (enable) {
		if (msleep_counter == -1) {
			msleep_counter = SLEEP_COUNTER;
		}
	} else {
		msleep_counter = -1;
	}
}

void msleep_callback()
{
	if (msleep_counter == -1) return;

	if (msleep_counter == 0) {
		motori_sleep(1);
	} else {
		--msleep_counter;
	}
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
		if ((pen_status == 0) || accel_mode) {
			motor_pace_goal = steep ? MOTOR_PACE_FASTY : MOTOR_PACE_FASTX;
                        if (pen_status == 0) {
                            motor_pace_goal = MOTOR_PACE_PENUP;
                        }
#ifdef SIM
                        fprintf(stderr, "set ACCEL_ACCEL goal=%d, speed_skip=%d ", motor_pace_goal,
                                speed_skip);
#endif
#if 0
			if (speed_skip != 0) motor_pace_goal *= 2;	// not too fast for laser
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
#endif
			motor_accel_ctr = ACCEL_STEPS_RAMP;
		}
		if (motor_pace != motor_pace_goal) motor_accel = -1;
#ifdef SIM
                fprintf(stderr, "final=%d\n", motor_pace_goal);
#endif
		break;
	case ACCEL_DECEL:
#ifdef SIM
                fprintf(stderr, "set ACCEL_DECEL\n");
#endif
		if ((pen_status == 0) || accel_mode) {
			motor_pace_goal = MOTOR_PACE_SLOW;
			motor_accel_ctr = ACCEL_STEPS_RAMP;
		}
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

void set_speed(double value) {
	if (value >= 0) {
		speed_skip = (int)round(value);
	} else {
		speed_skip = SPEED_SKIP;
	}
}

void set_acceleration_mode(double value) {
	accel_mode = value != 0.0;
}

void update_decelthresh() {
    STEPPER_COORD speedup_length, slowdown_length;

    uint32_t path_steplength = path_step_length_from_start();
    speedup_length = (motor_pace-motor_pace_goal)*ACCEL_STEPS_RAMP;
    slowdown_length = (MOTOR_PACE_SLOW-motor_pace_goal)*ACCEL_STEPS_RAMP;

    if ((speedup_length + slowdown_length) < path_steplength) {
        accel_decelthresh = path_steplength - slowdown_length;
    } else {
        accel_decelthresh = path_steplength / 2;
    }
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
    uint8_t finish_path = 0;
    uint8_t newpath = 0;
    static uint8_t arc_active = 0, initializing = 0, char_active = 0;

    dstx = dsty = -1;
    if (seeking0 != SEEK0_NULL) {
        while (seeking0 != SEEK0_DONE) do_idle();
        seeking0 = SEEK0_NULL;
        plotter_init();
        printf_P(PSTR("\nHOME\n"));
    } else if (initializing) {
        while(!motors_ready()) do_idle();
        initializing = 0;
        plotter_init();
    } else if (arc_active) {
        arc_active = arc_next(&dstx,&dsty);
    } else if (char_active) {
        char_active = text_char(0, &dstx, &dsty, &penny);
        pen_control(penny);
    } else {
        if (path_can_add() && uart_available()) {
            msleep_enable(0);
            motori_sleep(0);

            //uart_putchar(c = uart_getc());
            putchar(c = getchar()); //fflush(stdout);

            switch(hpgl_char(c, &dstx, &dsty, &labelchar)) {
                case CMD_PU:
                    if (pen_status != 0) {
                        finish_path = 1;
                    }
                    break;
                case CMD_PD:
                    if (pen_status != 1) {
                        finish_path = 2;
                    }
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
                    finish_path = 1;
                    break;
                case CMD_SEEK0:
                    seeking0 = SEEK0_SEEK;
                    finish_path = 1;
                    break;
                case CMD_LB0:
                    finish_path = 1;
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
                case CMD_VS:
                    set_speed(numpad[0]);
                    break;
                case CMD_AS:
                    set_acceleration_mode(numpad[0]);
                    break;
                default:
                    break;
            }
        } else {
            msleep_enable(1);
        }
    }

    if (!finish_path && dstx != -1 && dsty != -1) {
        /* Path can be continued because pen state stays the same */
        /* Check if the angle permits doing so */
        float a = path_angle_to(dstx, dsty);
#ifdef SIM
        fprintf(stderr, "angle=%3.1f\n", a);
#endif
#define CURVEANGLE 15
        if (a > CURVEANGLE) {// && a < (360-CURVEANGLE)) {
            finish_path = 3;
        }
    }

    if (finish_path) {
#ifdef SIM
        fprintf(stderr, "flushing path, count=%d "
                    " will add waypoint to %d %d after\n", path_count(), dstx, dsty);
#endif
        STEPPER_COORD px, py;
        while (path_count() != 0) {
            while (!motors_ready()) do_idle();
            path_dequeue(&px, &py); 
#ifdef SIM
            fprintf(stderr, "(f) after dequeue, path_count()=%d, to: %d %d\n",
                    path_count(), px, py);
#endif
            movestep(px, py);
        }
        while (!motors_ready()) do_idle();
        switch (finish_path) {
            case 1: pen_control(0); break;    /* pen up */
            case 2: pen_control(1); break;    /* pen down */
            case 3: break;                    /* sharp angle */
        }
        path_new();
        newpath = 1;
#ifdef SIM
        fprintf(stderr, "flushed path (%d)\n", finish_path);
#endif
        path_stepnumber = 0;
    }

    if (dstx >= 0 && dsty >= 0) {
        //if (path_count() == 0) newpath = 1;
        //if (motors_ready()) newpath |= 1;

        path_add(dstx, dsty);
        //fprintf(stderr, "waypoint to %d,%d len=%d\n", dstx, dsty, path_step_length_from_start());

        update_decelthresh();
    }

    if (motors_ready()) {
        if (path_count() != 0) {
            path_dequeue(&dstx, &dsty); 

            movestep(dstx, dsty);
            //if (newpath) {
                set_acceleration(ACCEL_FIXSLOW, 0);
                set_acceleration(ACCEL_ACCEL, move_is_steep());
            //}
            
            update_decelthresh();
        }
    }
}

/// Called from TIMER0_COMPA_vect.
/// Stepper motion pulses are issued here.
void motion_callback(uint8_t borderflags)
{
    if (seeking0 == SEEK0_SEEK) {
        if ((borderflags & (_BV(XSUP)|_BV(YSUP))) == (_BV(XSUP)|_BV(YSUP))) {
            seeking0 = SEEK0_DONE;
        }
        step(borderflags & _BV(XSUP) ? 0 : -1, borderflags & _BV(YSUP) ? 0 : -1);
        return;
    }

    if (speed_skip != 0) {
        speed_skip_ctr--;
        if ((speed_skip_ctr != 0) && (pen_status != 0)) {
            return;
        } else {
            speed_skip_ctr = speed_skip;
        }
    }

    // movestep(-1,-1) returns ordinal number of the current step during movement
    //int16_t stepnumber = movestep(-1,-1);
    movestep(-1, -1);

    ++path_stepnumber;
#if SIM
    fprintf(stderr, "step#%d pace=%d decelthresh=%d\n", path_stepnumber, 
            motor_pace, accel_decelthresh);
#endif

    if (path_stepnumber == accel_decelthresh) {
        if (accel_decelthresh != 0) {
            set_acceleration(ACCEL_DECEL, move_is_steep());
        }
    }

    //if (stepnumber != -1 && motor_accel) {
    if (motor_accel) {
        if (--motor_accel_ctr == 0) {
            motor_accel_ctr = ACCEL_STEPS_RAMP; 
            motor_pace += motor_accel;
            if (motor_pace == motor_pace_goal) {
                motor_accel = 0;
            }
        }
    }
}

uint8_t get_pen_status()
{
    return pen_status;
}

uint8_t get_motor_pace()
{
    return motor_pace;
}

void pen_control(uint8_t down) {
    if (pen_status != down) {
        while(!motors_ready()) do_idle();
        if (down) {
            DELAY_MS(PEN_DOWN_DELAY);
            pen_servo_set(255);
            //pen_relax = 255;
            pen_status = 1;
        } else {
            //pen_relax = 255;
            pen_servo_set(255);
            pen_status = 0;
        }

        pen_solenoid_update();

        DELAY_MS(PEN_LIFT_DELAY);
    }
}


int main() {
	init_io();

	global_set_sleep_mode_idle();

        usart_init((F_CPU/(16*115200))-1);
	
        printf_P(PSTR("HEART OF THE INTERSTELLAR LINER\n"));
	
	motori_init();

	plotter_init();
	
	hpgl_init();
	
        global_enable_interrupts();
	
	set_acceleration(ACCEL_FIXFAST, 0);
	seeking0 = SEEK0_SEEK;

	for (;;) {
                do_idle();
		do_stuff();
	}
	
	return 0;
}

// $Id$
