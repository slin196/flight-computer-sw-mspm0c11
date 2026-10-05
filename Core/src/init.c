/*
 * init.c
 *
 * Created: 13/03/2026 15:32:06
 *  Author: slin9
 */ 
#include <stdint.h>
#include "../inc/sys_settings.h" //TODO: hier dann die ganzen makros rein die dann die codegenerierung festlegen
#include "../inc/crsf_drv.h"
#include "../inc/flight_controll.h"
#include "../inc/controller.h"
#include "../inc/motor_driver.h"
#ifdef MPU60x0
#include "Peripherals/MPU60x0.h"
#endif
extern volatile uint8_t rx_buffer_head;
extern volatile uint8_t rx_buffer_tail;
extern volatile uint8_t rx_buffer[255];
extern volatile uint8_t rx_buffer_space;

typedef void* init_data;

typedef struct uart_init_data {
	volatile uint8_t* rx_buffer_addr;
	volatile uint8_t* rx_buffer_tail_ptr;
	volatile uint8_t* rx_buffer_head_ptr;
	volatile uint8_t* rx_buffer_space_ptr;
} crsf_drv_init_;

flight_controller_ifc fc = {
#ifdef MPU60x0
	.get_sensor_data = mpu60x0_drv_get_sensor_data_v1,
	/*#elif defined () //TODO: du weißt was zu tun ist
	#elif defined () */
#endif
#ifdef COMPLEMENTARY_FILTER
	.calculate_attitude = complementary_filter,
#elif defined (KALMAN_FILTER)
//TODO: du weißt schon
#endif
#ifdef CRSF
	.retrieve_commands = crsf_drv_convert_channels_values_into_attitude,
#endif
	.pid_controller = pid_controller,
	.motor_controll = motor_drv_set_attitude
};

/*typedef struct crsf_drv_init {
	volatile uint8_t* rx_buffer_addr;
	volatile uint8_t* rx_tail_addr; //pointer to the next byte to read in the buffer
	volatile uint8_t* rx_head_addr; //pointer to the latest received byte
	volatile uint8_t* rx_buffer_space_ptr;
	//TODO: das ganz hier noch generischer machen, indem du anstatt uint8_t* typ einen typedef machst, damit der
	//head und tail der queue des uart moduls vom beliebigen typ sein k�nnen.
}crsf_drv_init;*/

/*crsf_drv_init_ uart_data = {
	.rx_buffer_addr = rx_buffer,
	.rx_buffer_tail_ptr = &rx_buffer_tail,
	.rx_buffer_head_ptr = &rx_buffer_head,
	.rx_buffer_space_ptr = &rx_buffer_space
};*/

void core_init() {
#ifdef MPU60x0
	while (mpu60x0_drv_init(3,3,7)); //TODO: das hier vll. noch abstrakter machen. die jeweiligen hw-spezifischen IMU funktionen docken an ein
	//IMU interface an. grundsätzliche fähigkeiten wie gyro configurieren, achsenwerte auslesen, etc.
#endif
	controller_init();
	//crsf_driver_init((init_data*)&uart_data);
}


