#ifndef _ARC_H
#define _ARC_H

#include <inttypes.h>

/// Initialize the arc based on current location and scratchpad data.
/// @see numpad
/// @see user_loc
/// @returns 0 if the arc is degenerate (0 degrees or R=0)
int16_t arc_init();

/// Calculate the next chord. 
/// @param x next x in absolute stepper coordinates
/// @param y next y in absolute stepper coordinates
/// @returns 0 if this is the last chord
uint8_t arc_next(int16_t* x, int16_t* y);

#endif
