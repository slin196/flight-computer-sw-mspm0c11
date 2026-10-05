/*
 * sys_settings.h
 *
 * Created: 30/12/2025 18:04:47
 *  Author: slin9
 */ 


#ifndef SYS_SETTINGS_H_
#define SYS_SETTINGS_H_

/*define the main-loop runtime in seconds*/
#define MAIN_LOOP_RUNTIME_IN_S (0.004f)

/*define wether fixed or floating point hw is used*/
//#define FIXED_POINT

/*macros that define which IMU-Hardware is used*/
#define MPU60x0

/*macros which defines usage of fixed or floating point*/
//#define FIXED_POINT

/*macros that define the ucontroller plattform*/
#define MSPM0C110x //MSPM0C110x Series


/*macros which define algorithm for sensorfusion*/
#define COMPLEMENTARY_FILTER
//#define KALMAN_FILTER

#endif /* SYS_SETTINGS_H_ */