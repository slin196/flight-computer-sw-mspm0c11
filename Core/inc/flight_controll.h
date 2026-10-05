

#ifndef FLIGHT_CONTROLL_H
#define FLIGHT_CONTROLL_H

#include "controller.h"
#include "sys_settings.h"

typedef struct {
	uint8_t (*get_sensor_data)(sensordata*, sensordata*);
	void (*calculate_attitude)(attitude*, sensordata, sensordata);
	void (*retrieve_commands)(attitude*);//die umwandlung der werte sollte die ensprechende funktion im crsf treiber machen
	//schlie�lich, damit der fc unabh�ngig vom protokoll bleibt
	void (*pid_controller)(motor_cmds* m_controll, attitude state, attitude desired);
#ifdef FIXED_POINT
	void (*motor_controll)(uint16_t, uint16_t, uint16_t, uint16_t);
#else
	void (*motor_controll)(float, float, float, float);
#endif
	void (*m1_controll)(int16_t);
	void (*m2_controll)(int16_t);
	void (*m3_controll)(int16_t);
	void (*m4_controll)(int16_t);
} flight_controller_ifc;

void fc_steer_drone(flight_controller_ifc* fc);

#endif
