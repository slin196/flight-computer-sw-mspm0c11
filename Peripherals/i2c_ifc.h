
/*
 * I2C_Driver.c
 *
 * Created: 29/12/2025 17:49:49
 *  Author: slin9
 */ 

#include <stdint.h>

typedef enum {
	OP_RES_OK,
	OP_RES_PENDING,
	OP_RES_ERROR
} I2C_OPRES;

typedef struct i2c_ifc {
	uint8_t (*i2c_init)(); //TODO: hier noch initdata als funktionsargument festlegen und was alles an init-parametern �bergeben werden kann
	uint8_t (*i2c_transmit)(uint8_t* tx_status, uint8_t addr, uint8_t data);
	uint8_t (*i2c_receive)(uint8_t* data, uint8_t size);
	uint8_t (*i2c_start_com)(uint8_t* msg, uint8_t msgSize);
	uint8_t (*i2c_imu_reg_read)(uint8_t imu_addr, uint8_t reg_addr, uint8_t* readval);
	uint8_t (*i2c_imu_reg_write)(uint8_t imu_addr, uint8_t reg_addr, uint8_t writeval);
} i2c_ifc;



