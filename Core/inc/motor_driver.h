


#ifndef MOTOR_DRIVER
#define MOTOR_DRIVER
#include "sys_settings.h"

#ifdef FIXED_POINT
void motor_drv_set_attitude(uint16_t thrust_cmd, uint16_t roll_cmd, uint16_t yaw_cmd, uint16_t pitch_cmd);
#else
void motor_drv_set_attitude(float thrust_cmd, float roll_cmd, float yaw_cmd, float pitch_cmd);
#endif


#endif