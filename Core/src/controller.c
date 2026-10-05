/*
 * controller.c
 *
 * Created: 07/01/2026 21:18:21
 *  Author: slin9
 */

#include <stdint.h>
#include "../inc/controller.h"
#include "../inc/pid_controller.h"

//TODO: das hier noch in floating point version machen und in dasd pid controll file packen?

pid_object pid_roll, pid_pitch, pid_yaw, pid_thrust; //pid_pitch_rate, pid_roll_rate, pid_yaw_rate;


void controller_init() {
	init_pidobj(&pid_roll, 1.0, 1.0, 1.0); //TODO: noch die richtigen werte finden
	init_pidobj(&pid_pitch, 1.0, 1.0, 1.0);
	init_pidobj(&pid_yaw, 1.0, 1.0, 1.0);
	init_pidobj(&pid_thrust, 1.0, 1.0, 1.0);
}

void pid_controller(motor_cmds* m_controll, attitude state, attitude desired)  {
	
	float dt = 4; //TODO: noch richtigen wert hier einbauen und eine fixed und floating point version machen

	m_controll->roll_rate = pid_controll(&pid_roll, state.roll, desired.roll, dt);
	
	m_controll->pitch_rate = pid_controll(&pid_pitch, state.pitch, desired.pitch, dt);
	
	m_controll->yaw_rate = pid_controll(&pid_yaw, state.yaw, desired.yaw, dt);

	//m_controll->thrust_vel = pid_controll(&pid_thrust, state.thrust, desired.thrust, dt);

	//m_controll->pitch_rate
	
	/*m_controll->thrust_vel = control.pid_control(&pid_thrust, state.thrust, desired.thrust);
	
	m_controll->pitch_rate = control.pid_control(&pid_pitch_rate, gyro.vel_x, pitch_rate);
	
	m_controll->roll_rate = control.pid_control(&pid_roll_rate, gyro.vel_y, roll_rate);
	
	m_controll->yaw_rate = control.pid_control(&pid_yaw_rate, gyro.vel_z, yaw_rate);*/
}
