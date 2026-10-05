/*
 * sensorfusion.h
 *
 * Created: 09/01/2026 19:36:42
 *  Author: slin9
 */ 


#ifndef SENSORFUSION_H_
#define SENSORFUSION_H_

#include "sys_settings.h"
#include <stdint.h>

#ifdef FIXED_POINT

typedef union {
	/* For precision tracking of both fast and slow
	motions, the parts feature a user-programmable gyroscope full-scale range of �250, �500, �1000, and
	�2000�/sec (dps) and a user-programmable accelerometer full-scale range of �2g, �4g, �8g, and �16g. see p. 7*/
	struct {
		int16_t vel_x; //unit: �/s
		int16_t vel_y;
		int16_t vel_z;
	};
	struct {
		int16_t a_x; //unit: g = 9.81 m/s�
		int16_t a_y;
		int16_t a_z;
	};
} sensordata;

typedef struct {
	int16_t pitch; //TODO: gradma� oder bogenma�? //TODO: umsetzen: oberen 8 bit sind vorkomma, untere 8-bit nachkomma, bisjetzt: kein nachkomma
	int16_t roll;  //TODO: oberen 8 bit sind vorkomma, untere 8-bit nachkomma
	int16_t yaw;   //oberen 8 bit sind vorkomma, untere 8-bit nachkomma
	int16_t thrust;  //oberen 8 bit sind vorkomma, untere 8-bit nachkomma
} attitude;

void complementary_filter(attitude* state, sensordata gyro, sensordata acc, uint8_t dt)

#else 

typedef union {
	/* For precision tracking of both fast and slow
	motions, the parts feature a user-programmable gyroscope full-scale range of �250, �500, �1000, and
	�2000�/sec (dps) and a user-programmable accelerometer full-scale range of �2g, �4g, �8g, and �16g. see p. 7*/
	struct {
		float vel_x; //unit: degrees/s
		float vel_y;
		float vel_z;
	};
	struct {
		float a_x; //unit: g = 9.81 m/(s*s)
		float a_y;
		float a_z;
	};
} sensordata;

typedef struct {
	float pitch; //TODO: ob der nackommabereich wichtig ist? eine auflösung auf ein grad ist auch sehr präzise und würde integerrechnung
				 //erlauben, die höchstwahrscheinlich schneller ist
	float roll;  
	float yaw;   
} attitude;


void complementary_filter(attitude* state, sensordata gyro, sensordata acc);

#endif



#endif /* SENSORFUSION_H_ */