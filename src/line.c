#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <avr/pgmspace.h>

#include "motori.h"
#include "shvars.h"

static uint8_t moving = 0;
static uint8_t zteep;

void move_reset() {
	moving = 0;
}

uint8_t motors_ready() {
	return !moving;
}

uint8_t move_is_steep() {
	return zteep;
}

#define SWAP(a,b) {(a)^=(b);(b)^=(a);(a)^=(b);}

int16_t movestep(int16_t x1, int16_t y1) {
	static int16_t twoDy, twoDyTwoDx, E, x, y, xend, stepcount;
	static int8_t xstep, ystep;
	static uint8_t skippy;

	static int stepX, stepY;

	
	if (!moving && x1 != -1 && y1 != -1) {	// begin new line
		int x0 = stepper_loc.x;
		int y0 = stepper_loc.y;
		int Dx = x1 - x0;
		int Dy = y1 - y0;
		
		zteep = abs(Dy) >= abs(Dx);
		//printf("steep=%d from (%d %d) (%d v %d)\n", zteep, stepper_loc.x, stepper_loc.y, Dx, Dy);
		if (zteep) {
			SWAP(x0, y0);
			SWAP(x1, y1);
			// recompute Dx, Dy after swap
			Dx = x1 - x0;
			Dy = y1 - y0;
		}
		
		xstep = 1;
		if (Dx < 0) {
			xstep = -1;
			Dx = -Dx;
		}
		ystep = 1;
		if (Dy < 0) {
			ystep = -1;
			Dy = -Dy;
		}
		
		twoDy = 2*Dy;
		twoDyTwoDx = twoDy - 2*Dx; 
		E = twoDy - Dx;
		y = y0;
		x = x0;
		xend = x1;
		
		moving = Dx != 0;
		skippy = 0;
		
		//printf_P(PSTR("zteep:%d nsteps:%d\n"), zteep, Dx);
		
		// don't make a step yet
		stepcount = 0;
		return Dx;
	} 
	// continue drawing, loop iteration
	// loop variables:
	// x, y, E, twoDyTwoDx, ystep, xstep, twoDy
	if (moving) {
#ifdef YSKIPPY	
		skippy++;
		//if (zteep && ((skippy & 0x1) != 1)) {
		//	return -1;
		//}
		if (zteep && ((skippy & 0x11) == 0x10)) {	// skip one out of 4
			return -1;
		}
#endif		
		int makeYstep = 0;
		
		if (E > 0) {
			E += twoDyTwoDx;
			y += ystep;
			makeYstep = 1;
		} else {
			E += twoDy;
		}
		x += xstep;
		stepcount++;
		
		if (zteep) {
			stepper_loc.x = y;
			stepper_loc.y = x;
			stepX = makeYstep ? ystep : 0;
			stepY = xstep;
		} else {
			stepper_loc.x = x;
			stepper_loc.y = y;
			stepX = xstep;
			stepY = makeYstep ? ystep : 0;
		}
		
		if (x == xend) {
			moving = 0;
		}
		step(stepX, stepY);
	}
	
	return stepcount;
}