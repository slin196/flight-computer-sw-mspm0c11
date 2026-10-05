


#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "sensorfusion.h"

#ifdef FIXED_POINT
typedef struct {
	uint8_t pitch_rate;
	uint8_t roll_rate;
	uint8_t yaw_rate;
	uint8_t thrust_vel;
} motor_cmds;

#else

typedef struct {
	float pitch_rate;
	float roll_rate;
	float yaw_rate;
} motor_cmds;

#endif

void pid_controller(motor_cmds* m_controll, attitude state, attitude desired);
void controller_init();

#endif