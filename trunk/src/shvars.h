#ifndef _SHVARS_H
#define _SHVARS_H

#include "motori.h"

extern char scratchpad[];		///< character scratchpad used by scanner
extern double numpad[];		///< stored parameters of AA and similar commands
extern int32_t ip_pad[];		///< stored parameters of IP command (4)
extern int32_t sc_pad[];		///< stored parameters of SC command (4)

extern STEPPER_POINT stepper_loc;      ///< absolute stepper coordinates

extern USER_POINT user_loc;		///< rounded location in user coordinate system (used for arc calculation)

extern uint8_t borderflags;	///< Margin flags: MSB [0 0  ZINF ZSUP YINF YSUP XINF XSUP] LSB

#endif