/*
 * PID_Controller.c
 *
 * Created: 02/01/2026 18:05:38
 *  Author: slin9
 */ 

#include <stdint.h>
#include "../inc/sys_settings.h"
#include "../inc/helper_functions.h"
#include "../inc/pid_controller.h"

/*#define Kp
#define Kd
#define Ki*/

#define MAX_SIGNED_16_BIT 32767
#define MIN_SIGNED_16_BIT -32768


//TODO: um das hier noch allgemeiner zu machen ein PID interface machen und dann diese lib atmega328p spezifisch machen
/*void calculatePID_328p_c(pid_object* pid, uint16_t measured, uint16_t desired, uint8_t dt) {//TODO: das hier wird die ausgefeilte version
	int8_t neg = 0;
	int16_t error;
	uint16_t output = 0;
	if (desired>measured) {
		error = (int16_t)(0x7fff & (desired - measured));
	} else {
		error = (int16_t)(0x7fff & (measured - desired));
		neg = 1;
	}
	output += pid->Kp * error;
	
	output += pid->Kd;
		
	pid->integral;
    uint16_t integral_term = pid->Ki * pid->integral;
}*/
	
#ifdef FIXED_POINT

void init_pidobj(pid_object* pid, uint8_t kp, uint8_t kd, uint8_t ki) {
	pid->integral = 0;
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
	//Kp*1;
}
	//TODO: eigentlich ist das hier nicht 328p spezifisch. man k�nnte das einfach als fixed point bibliothek hernehmen.d.h. das hier in die controller lib 
	//hauen oder zumindest umbennen.
int16_t pid_controll(pid_object* pid, uint16_t state, uint16_t desired, uint8_t dt) {//TODO: das hier wird die ausgefeilte version
	int8_t sign = 1;
	uint16_t error;
	int16_t output = 0;
	int16_t error_sgn;
	
	if (desired>state) {
		error = desired - state;
	} else {
		error = state - desired;
		sign = -1;
	}
	
	if (error & (1<<15)) {
		error_sgn = MIN_SIGNED_16_BIT;
	}
	else {
		error_sgn = error;
		error_sgn *= sign;
	}

	int16_t p_term = ((int16_t)pid->Kp) * error_sgn;
	//TODO: wie l�uft das wenn das h�chste bit des uint8_t datentyps besetzt ist? wird es dann
	//als negativ interpretiert? noch herausfinden, sonst besteht hier die m�glichkeit eines bugs
	output += p_term;
	
	//TODO: stell noch sicher das bei den multiplikation �berlaufe ausgeschlossen werden
	pid->integral += error_sgn*(int16_t)dt; //TODO: wie l�uft das wenn das h�chste bit des uint8_t datentyps besetzt ist? wird es dann 
	//als negativ interpretiert? noch herausfinden, sonst besteht hier die m�glichkeit eines bugs
	int16_t i_term = pid->Ki*pid->integral;
	
	output += i_term;
	//ANMICH: wenn durch den PID nur die attitude der drohne korrigiert wird, dann sollte eigentlich ein 16 bit werd absolut ausreichen
	//weil winkel im gradma� nicht gr��er als 360 werden sollten, im bogenma� sind die werte noch kleiner. die anzahl der 
	//nachkommastellengenauigkeit wird hier vll. bestimmender
	
	int8_t bits_left;
	if ((bits_left = num_of_free_msbs_after_16bit_mult(pid->Kd, error)) < 0) return -1;
	
	uint16_t d_term = (pid->Kd * error << bits_left) / dt;
	d_term >>= bits_left; 	
	output += sign * (int16_t)d_term;
	
	return output;
}
#else

void init_pidobj(pid_object* pid, float kp, float kd, float ki) {
	pid->integral = 0.0f;
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
	//Kp*1;
}

float pid_controll(pid_object* pid, float state, float desired, float dt) {
	
	float error = desired - state;

	float output = pid->Kp * error;
	
	//TODO: stell noch sicher das bei den multiplikation �berlaufe ausgeschlossen werden
	pid->integral += error*dt; 

	float i_term = pid->Ki * pid->integral;
	
	//output += i_term; //TODO: comment wieder zu code werden lassens
	//ANMICH: wenn durch den PID nur die attitude der drohne korrigiert wird, dann sollte eigentlich ein 16 bit werd absolut ausreichen
	//weil winkel im gradma� nicht gr��er als 360 werden sollten, im bogenma� sind die werte noch kleiner. die anzahl der 
	//nachkommastellengenauigkeit wird hier vll. bestimmender
	
	float d_term = pid->Kd * error / dt;
	output += d_term;
	
	return output;
}

#endif