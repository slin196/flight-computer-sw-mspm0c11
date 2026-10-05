/*
 * pwm_driver.h
 *
 * Created: 19/02/2026 09:43:27
 *  Author: slin9
 */ 


#ifndef PWM_DRIVER_H_
#define PWM_DRIVER_H_

typedef struct {
	uint8_t top;
	uint8_t fill;
	uint8_t pwm_freq_khz;
	uint8_t pwm_freq_khz_nom;
	//TODO: noch weiter ausfüllen
} timer_obj_8bit;

typedef struct {
	uint8_t top_l;
	uint8_t top_h;
	uint8_t pwm_freq_khz;
	uint8_t pwm_freq_khz_nom;
	//TODO: noch weiter ausfüllen
} timer_obj_16bit;
//TODO: die beiden vll. noch in einem enum vereinen

typedef union {
	struct {
		uint8_t top;
		uint8_t fill;
		uint8_t pwm_freq_khz;
		uint8_t pwm_freq_khz_nom;
		//TODO: noch weiter ausfüllen
	}; //timer_obj_8bit;

	struct {
		uint8_t top_l;
		uint8_t top_h;
		uint8_t pwm_freq_khz_16;
		uint8_t pwm_freq_khz_nom_16;
		//TODO: noch weiter ausfüllen
	}; //timer_obj_16bit
} timer_obj;


void init_pwm_driver(uint16_t);
void set_pwm_duty_cylce(timer_obj, uint8_t);
void config_pwms(uint16_t);
void activate_pwm_outputs();

#endif /* PWM_DRIVER_H_ */