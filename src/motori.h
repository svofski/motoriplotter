#ifndef _MOTORI_H
#define _MOTORI_H

///< Absolute coordinates used for stepper motion.
///< Negative means invalid.
typedef int16_t STEPPER_COORD;		

///< User coordinates used in input, arc calculation etc.
typedef double USER_COORD;

typedef struct _stepper_xy {
	STEPPER_COORD x;
	STEPPER_COORD y;
} STEPPER_POINT;

typedef struct _user_xy {
	USER_COORD x;
	USER_COORD y;
} USER_POINT;

/// Acceleration modes.
/// See configs.h for marginal values.
typedef enum _acceleration_modes {
	ACCEL_FIXSLOW = 0,		///< Move slowly, fixed speed
	ACCEL_FIXFAST,			///< Move fast, fastest possible speed
	ACCEL_FIXMEDIUM,		///< Fixed average speed
	ACCEL_ACCEL,			///< Ramp speed up
	ACCEL_DECEL,			///< Ramp speed down
} ACCEL_MODE;


/// Issue the actual step commands to stepper controllers.
/// @param xdir,ydir Direction +1/-1, 0 == stand still
void step(int8_t xdir, int8_t ydir);

/// Set motor acceleration profile.
///
/// Since the motors are not equal, speed values depend on the prevalent moving axis.
/// @param accel_mode acceleration mode
/// @param steep true if Y axis is the main axis.
///
/// @see move_is_steep()
void set_acceleration(ACCEL_MODE accel_mode, uint8_t steep);


/// Controls the pen position. Both servo and solenoid actuators.
/// Causes immediate delay to allow for the pen to settle.
///
/// @param down true if pen should be down, 0 if raised
void pen_control(uint8_t down);
void pen_immediate(uint8_t down);

/// Initialize plotter state. Move to home position, then reset everything, including motors and timers.
/// Reset user scale and translation, raise the pen.
void plotter_init();


#endif
