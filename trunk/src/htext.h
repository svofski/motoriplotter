#ifndef _HTEXT_H
#define _HTEXT_H

#include <inttypes.h>
#include <avr/pgmspace.h>

#include "motori.h"

extern PGM_P charset173[256];
extern PGM_P charset0[256];


void text_init();
void text_setscale(double sx, double sy);
void text_scale_cm(double cx, double cy);
void text_scale_rel(double rx, double ry);
void text_direction(double cost, double sint);

void text_beginlabel();
uint8_t text_char(uint8_t c, STEPPER_COORD* dstx, STEPPER_COORD* dsty, uint8_t* pen);

#endif