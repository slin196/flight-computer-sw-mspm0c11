/*
 * command_receiver_ifc.c
 *
 * Created: 28/02/2026 21:49:57
 *  Author: slin9
 */ 

#include "../inc/sensorfusion.h"


typedef struct comlink_interface {
	 uint16_t (*get_thrust)();
	 void (*get_desired_attitude)(attitude*);
}comlink_interface;
