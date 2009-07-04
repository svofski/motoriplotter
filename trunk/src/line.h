#ifndef _LINE_H_
#define _LINE_H_

#include <inttypes.h>

/// @returns true if motors are not moving
uint8_t motors_ready();

/// Initialize a line, or perform a subsequent line step.
///
/// If motors_ready(), initiate a new line from the current location (stepper_loc)
/// to specified (x1,y1). Subsequent steps should make calls with (-1,-1) arguments.
/// Each line step will will call back external function step(x,y), until movement
/// is done.
///
/// The movement is calculated using adopted Bresenham's algorithm.
///
/// @returns amount of steps after the first call
/// @returns ordinal number of the current step during movement
/// @see step(x,y) (motori.h)
int16_t movestep(int16_t x1, int16_t y1);

/// @returns true if X and Y coordinates are swapped
uint8_t move_is_steep(); // y axis is prevalent

#endif
