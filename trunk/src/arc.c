#include <inttypes.h>
#include <math.h>

#include "shvars.h"
#include "arc.h"
#include "scale.h"


static int16_t arc_step;
static double arc_stepangle, arc_xc, arc_yc, arc_a0, arc_r, arc_phi;

int16_t arc_init() {
	arc_phi = numpad[2]*M_PI/180.0;
	
	arc_stepangle = fabs(numpad[3]);
	if (arc_phi < 0) arc_stepangle = -arc_stepangle;
	arc_stepangle *= M_PI/180.0;
	
	arc_step = 0;
	arc_xc = numpad[0];
	arc_yc = numpad[1];
	arc_a0 = atan2(user_loc.y-arc_yc, user_loc.x-arc_xc);
	if (arc_a0 < 0) arc_a0 += 2*M_PI;
	
	arc_r = hypot(user_loc.x-arc_xc, user_loc.y-arc_yc);
	
	//printf_P(PSTR("AA: (%f,%f) r=%f stepangle=%f a0=%f aend=%f\n"),
	//	arc_xc, arc_yc, arc_r, arc_stepangle, arc_a0*180/M_PI, (arc_a0+arc_phi)*180/M_PI);
	
	return arc_phi != 0.0 && arc_r != 0.0;
}

uint8_t arc_next(int16_t* x, int16_t* y) {
	uint8_t cont = 1;
	
	arc_step++;
	double alpha = arc_step*arc_stepangle;
	if (fabs(alpha) > fabs(arc_phi)) {
		alpha = arc_phi;
		cont = 0;
	}
	alpha += arc_a0;
	
	double xd = arc_xc + arc_r*cos(alpha);
	double yd = arc_yc + arc_r*sin(alpha);

	//printf_P(PSTR("ARC SPAN: (%6.1f %6.1f)-(%6.1f %6.1f)\n"), user_loc.x, user_loc.y, xd, yd);	
	
	userscale(xd, yd, x, y, &user_loc.x, &user_loc.y);
	
	return cont;//arc_step < arc_steps ? 1 : 0;
}

