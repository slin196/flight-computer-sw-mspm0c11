
#ifndef PWM_IFC_H
#define PWM_IFC_H

#include <stdint.h>

typedef struct {
	void (*set_pwm_frequency) (uint16_t freq_khz);    //set frequency of all motor pwms in kHz
	void (*set_duty_cycle_for_motor_pwm) (uint8_t motor, uint8_t duty_cylce); //set duty cycle for a pwm signal controlling a motor
} pwm_if;


#endif