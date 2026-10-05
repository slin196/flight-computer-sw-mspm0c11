/*
 * MPU6000.h
 *
 * Created: 31/12/2025 10:26:04
 *  Author: slin9
 */ 


#ifndef MPU6000_H_
#define MPU6000_H_

#include "../Core/inc/sensorfusion.h" //TODO: die datentypen die hier mit diesem file
// gebraucht werden schaffen eine gewisse abh�ngigkeit, das verringert die modularit�t diese files,, da evtl. eine l�sung finden
#include <stdint.h>

#define FS_SEL 3
#define XG_ST 5
#define YG_ST 6
#define ZG_ST 7
#define AFS_SEL 3
#define TEMP_FIFO_EN 7
#define XG_FIFO_EN 6
#define YG_FIFO_EN 5
#define ZG_FIFO_EN 4
#define ACCEL_FIFO_EN 3
#define FIFO_EN 6

#define REG_SMPRT_DIV         0x19
#define REG_CONFIG            0x1A
#define REG_GYRO_CONIG        0x1B
#define REG_ACLO_CONIG        0x1C
#define REG_FIFO_EN           0x23
#define REG_I2C_MST_CTRL      0x24
#define REG_INT_PIN_CFG		  0x36
#define REG_INT_ENABLE		  0x38
#define REG_SIGNAL_PATH_RESET 0x68
#define REG_PWR_MGMT_1        0x6B
//#define REG_FIFO_R_W		  0x74
#define WHO_AM_I	          0x75

#define REG_GYRO_X_H 0x43
#define REG_GYRO_X_L 0x44
#define REG_GYRO_Y_H 0x45
#define REG_GYRO_Y_L 0x46
#define REG_GYRO_Z_H 0x47
#define REG_GYRO_Z_L 0x48

#define REG_ACC_X_H 0x3B
#define REG_ACC_X_L 0x3C
#define REG_ACC_Y_H 0x3D
#define REG_ACC_Y_L 0x3E
#define REG_ACC_Z_H 0x3F
#define REG_ACC_Z_L 0x40

#define REG_FIFO_COUNT_H  0x72
#define REG_FIFO_COUNT_L  0x73
#define REG_FIFO_R_W      0x74
#define REG_USER_CTRL     0x6A
#define MPU_60x0_I2C_ADDR 0xd0 /*oder 0xd2 //TODO: sicherzstellen, das du hier auch wirklich die richtige addresse hast*/
                            //TODO: theoretisch ist die addresse ja 0x68, du hast es nur nach 1 nach links verschoben weil du auf dem atmega
							//noch manuell das read/write bit zur addresse hinzufügt, bei anderen uControllern macht man das per register
							//und das erste bit wird dann abgeschnitten, d.h. die addresse ist dann falsch, vll. kann man das noch generischer machen

typedef struct {
	uint8_t fs_range_gyro:2; //full-scale range gyroscope 0-3 0 min 3 max full scale range
	uint8_t fs_range_acc:2;  //full-scale range accelerometer 0-3 0 min 3 max full scale range
	uint8_t bandwith:3; //bandwith accelerometer and gyroscope 0-7 7 min 0 max bandwith
	uint8_t sample_rate;
} MPU_60x0_obj;

uint8_t mpu60x0_drv_init(uint8_t, uint8_t, uint8_t);
uint8_t _mpu60x0_drv_config_IMU(MPU_60x0_obj);
uint8_t mpu60x0_drv_burst_read_reg(uint8_t, uint8_t*, uint8_t);
uint8_t mpu60x0_drv_read_reg(uint8_t);
uint8_t mpu60x0_drv_burst_read_fifo(uint8_t*);
uint8_t mpu60x0_drv_write_reg(uint8_t, uint8_t);
uint8_t mpu60x0_drv_get_sensor_data(MPU_60x0_obj, volatile sensordata*, volatile sensordata*, uint8_t*);
uint8_t mpu60x0_drv_get_sensor_data_v1(sensordata*, sensordata*); 

#endif /* MPU6000_H_ */
