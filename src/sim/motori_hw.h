#pragma once

#include <inttypes.h>
#include <fakeavr.h>

void init_io(void);
void timer0_init(void);
void timer1_init(void);
void timer2_init(void);
void motori_init(void);
void motori_sleep(uint8_t zzz);
void step(int8_t xdir, int8_t ydir);
void pen_servo_set(uint8_t relax_value);
void pen_solenoid_update(void);
void global_enable_interrupts(void);
void global_set_sleep_mode_idle(void);
void do_idle(void);
void DELAY_MS(const long ms);
