#include <stdio.h>
#include <pgmspace.h>
#include <math.h>
#include <inttypes.h>

#include "motori.h"
#include "shvars.h"
#include "scale.h"
#include "configs.h"
#include "htext.h"

USER_POINT fontscale;
USER_POINT charorigin;
USER_POINT labelorigin;

USER_COORD sintheta, costheta;

void text_init() {
	text_setscale(10,10);
	text_direction(0, 1);
}

void text_setscale(double sx, double sy) {
	fontscale.x = sx;
	fontscale.y = sy;

	printf_P(PSTR("Text scale is (%f, %f)\n"), fontscale.x, fontscale.y);
}

void text_scale_cm(double cx, double cy) {
	double sx = cx*10/7/0.025;
	double sy = cy*10/10/0.025;
	userprescale(sx, sy, &fontscale.x, &fontscale.y);
	text_setscale(fontscale.x, fontscale.y);
}

void text_scale_rel(double rx, double ry) {
	USER_POINT prect = scale_P1P2();
	double sx = rx/100 * prect.x/7;	// character size in plotter units
	double sy = ry/100 * prect.y/10;	
	text_setscale(sx, sy);
}

void text_direction(double cost, double sint) {
	sintheta = -sint;
	costheta = cost;
	
	printf_P(PSTR("Label rotation: sin=%f cos=%f\n"), sintheta, costheta);
}

static void rotate(double* x, double* y) {
	double xc = *x - charorigin.x;
	double yc = *y - charorigin.y;
	double xrot = xc*costheta + yc*sintheta;
	double yrot = -xc*sintheta + yc*costheta;
	
	*x = xrot + charorigin.x;
	*y = yrot + charorigin.y;
}

static PGM_P coffs;

void text_beginlabel() {
	labelorigin = user_loc;
	printf_P(PSTR("Label origin: (%f,%f)\n"), labelorigin.x, labelorigin.y);
}

uint8_t text_char(uint8_t c, STEPPER_COORD* dstx, STEPPER_COORD* dsty, uint8_t* pen) {
	double xd, yd;
	uint8_t encoded;
	static uint8_t noadvance = 0;
	
	if (c != 0) {
		noadvance = 0;
		switch (c) {
		case '\r':
			printf_P(PSTR("CR"));
			xd = labelorigin.x;
			yd = labelorigin.y;
			//rotate(&xd, &yd);
			userscale(xd, yd, dstx, dsty, &user_loc.x, &user_loc.y);
			charorigin = user_loc;
			*pen = 0;
			encoded = 1;
			coffs = charset0[0];
			noadvance = 1;
			break;
		case '\n':
			printf_P(PSTR("LF"));
			xd = user_loc.x;
			yd = user_loc.y - fontscale.y*10;
			rotate(&xd, &yd);
			userscale(xd, yd, dstx, dsty, &user_loc.x, &user_loc.y);
			charorigin = user_loc;
			labelorigin = user_loc;
			*pen = 0;
			encoded = 1;
			coffs = charset0[0];
			noadvance = 1;
			break;
		default:
			coffs = charset0[c];
			*pen = 0;
			charorigin = user_loc;
			
			encoded = 1;
			//printf_P(PSTR("coffs=%x first=%o"), charset0[c], *charset0[c]);
			break;
		} 
	} else {
		encoded = *coffs++;

		*pen = encoded & 0200 ? 1 : 0;
		
		if (encoded) {
			xd = charorigin.x + fontscale.x * ((encoded >> 4) & 007);
			yd = charorigin.y + fontscale.y * ((encoded & 017) - 4);
		} else {
			if (!noadvance) {
				xd = charorigin.x + fontscale.x * 5;
				yd = charorigin.y;
			}
		}

		if (!noadvance) {
			rotate(&xd, &yd);
			
			userscale(xd, yd, dstx, dsty, &user_loc.x, &user_loc.y);
		}
	}
	
	return encoded != 0;
}
