



#include "pwm_ifc.h"
#include "../Bindings/pwm_implements.h"


pwm_if pwm_ifc_obj = {//TODO: noch richtige werte zuweisen.
	.set_pwm_frequency = 0,
	.set_duty_cycle_for_motor_pwm = set_duty_cycle
};