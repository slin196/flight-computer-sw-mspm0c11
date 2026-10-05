


#include "i2c_ifc.h"
#include "../Bindings/i2c_implements.h"


i2c_ifc i2c_ifc_obj = {
	.i2c_imu_reg_read =  i2c_imu_reg_single_read,
	.i2c_imu_reg_write = i2c_imu_reg_single_write
};
