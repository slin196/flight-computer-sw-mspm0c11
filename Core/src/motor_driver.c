
/*
 * motor_driver.c
 *
 * Created: 08/01/2026 14:20:35
 *  Author: slin9
 */ 

#include <stdint.h>
#include "../inc/motor_driver.h"
#include "Peripherals/pwm_ifc.h"
#include "../inc/sys_settings.h"

extern pwm_if pwm_ifc_obj;

uint8_t motor_drv_set_throttle(uint8_t motor, uint8_t throttle) {
	if (!((motor >= 1) && (motor<=4))) return 0;
	pwm_ifc_obj.set_duty_cycle_for_motor_pwm(motor, throttle);
	return 1;
}

#ifdef FIXED_POINT

void motor_drv_set_attitude(uint16_t thrust_cmd, uint16_t roll_cmd, uint16_t yaw_cmd, uint16_t pitch_cmd) {
	uint16_t r = roll_cmd/2;
	uint16_t y = yaw_cmd/2;
	uint16_t p = pitch_cmd/2;
	uint16_t m1 = thrust_cmd - r + p + y;
	uint16_t m2 = thrust_cmd - r - p - y;
	uint16_t m3 = thrust_cmd + r - p + y;
	uint16_t m4 = thrust_cmd + r + p - y;
	motor_drv_set_throttle(1, m1);
	motor_drv_set_throttle(2, m2);
	motor_drv_set_throttle(3, m3);
	motor_drv_set_throttle(4, m4);
}

#else

void motor_drv_set_attitude(float thrust_cmd, float roll_cmd, float yaw_cmd, float pitch_cmd) {
	float r = roll_cmd/2.0f;
	float y = yaw_cmd/2.0f;
	float p = pitch_cmd/2.0f;
	volatile float m1 = thrust_cmd - r + p + y;
	volatile float m2 = thrust_cmd - r - p - y;
	volatile float m3 = thrust_cmd + r - p + y;
	volatile float m4 = thrust_cmd + r + p - y;
	if (m1<0) m1 = 1.0f;
	else if (m1>255.0f) m1 = 254.0f;
	if (m2<0) m2 = 1.0f;
	else if (m2>255.0f) m2 = 254.0f;
	if (m3<0) m3 = 1.0f;
	else if (m3>255.0f) m3 = 254.0f;
	if (m4<0) m4 = 1.0f;
	else if (m4>255.0f) m4 = 254.0f;
	/*motor_drv_set_throttle(1, 0); //TODO: natürlich die richtigen werte rein
	motor_drv_set_throttle(2, 0);
	motor_drv_set_throttle(3, 0);
	motor_drv_set_throttle(4, 0);*/
	motor_drv_set_throttle(1, (uint8_t)m1);
	motor_drv_set_throttle(2, (uint8_t)m2);
	motor_drv_set_throttle(3, (uint8_t)m3);
	motor_drv_set_throttle(4, (uint8_t)m4);
}

#endif