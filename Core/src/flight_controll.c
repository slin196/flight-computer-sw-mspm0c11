/*
 * flight_controll.c
 *
 * Created: 10/01/2026 20:13:52
 *  Author: slin9
 */ 

#include <stdint.h>
#include "../inc/sensorfusion.h"
#include "../inc/flight_controll.h"
#include "../inc/pid_controller.h"

//volatile flight_controller_ifc* fc;
pid_object pid_yaw, pid_pitch, pid_roll, pid_thrust;

void fc_init_pid_obj(pid_object* pid, uint8_t Kp, uint8_t Ki, uint8_t Kd) {
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->integral = 0;
}

void fc_init(volatile flight_controller_ifc* obj) { //HÄ das argument wird gar nicht genutzt
	fc_init_pid_obj(&pid_yaw, 1,2,3);
	fc_init_pid_obj(&pid_roll, 1,2,3);
	fc_init_pid_obj(&pid_pitch, 1,2,3);
}

/*             
   m1-------m2
        |
		|
		|
   m4-------m3
*/
/*void fc_motor_controll(int16_t yaw, int16_t pitch, int16_t roll, uint16_t thrust) {
	int16_t m1 = thrust + pitch/2 + roll/2 - yaw/2;
	int16_t m2 = thrust + pitch/2 - roll/2 + yaw/2;
	int16_t m3 = thrust - pitch/2 - roll/2 - yaw/2;
	int16_t m4 = thrust - pitch/2 + roll/2 + yaw/2;
	fc->m1_controll(m1);
	fc->m2_controll(m2);
	fc->m3_controll(m3);
	fc->m4_controll(m4);
}*/

void fc_steer_drone(flight_controller_ifc* fc) {
	sensordata gyro_data, accel_data;
	attitude desired, actual;
	uint16_t thrust = 0;
	motor_cmds m_controll;
	fc->get_sensor_data(&gyro_data, &accel_data);
	fc->calculate_attitude(&actual, gyro_data, accel_data); 
	fc->retrieve_commands(&desired);
	fc->pid_controller(&m_controll, desired, actual);
	fc->motor_controll(m_controll.yaw_rate, m_controll.pitch_rate, m_controll.roll_rate, thrust);
}