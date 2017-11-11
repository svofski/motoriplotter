#pragma once

/* Terminate the current path if the specified pen state is different */
void path_new();
STEPPER_COORD path_add(STEPPER_COORD x, STEPPER_COORD y);
int8_t path_count();
int path_can_add();
void path_dequeue(STEPPER_COORD * x, STEPPER_COORD * y);
float path_angle_to(STEPPER_COORD x, STEPPER_COORD y);
int32_t path_step_length();
int32_t path_step_length_from_start();
