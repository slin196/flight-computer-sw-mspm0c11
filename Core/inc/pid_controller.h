

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "sys_settings.h"

#ifdef FIXED_POINT

typedef struct _pid_object {
	uint16_t integral;
	uint8_t int_sign; //0 for positive, 1 for negative
	uint8_t Kp;
	uint8_t Kd;
	uint8_t Ki;
} pid_object;

void init_pidobj(pid_object* pid, uint8_t kp, uint8_t kd, uint8_t ki);
int16_t pid_controll(pid_object* pid, uint16_t state, uint16_t desired, uint8_t dt);
#else

typedef struct _pid_object{
	float integral;
	float Kp;
	float Kd;
	float Ki;
} pid_object;

void init_pidobj(pid_object* pid, float kp, float kd, float ki);
float pid_controll(pid_object* pid, float state, float desired, float dt);

#endif

#endif