/*
 * MPU60x0.c
 *
 * Created: 31/12/2025 09:50:12
 *  Author: slin9
 */ 

#include <stdint.h>
#include "../Core/inc/sys_settings.h"
#include "MPU60x0.h"
#include "i2c_ifc.h"
#include "../Core/inc/sensorfusion.h" //TODO: die datentypen die hier mit diesem file
// gebraucht werden schaffen eine gewisse abh�ngigkeit, das verringert die modularit�t diese files,, da evtl. eine l�sung finden
//TODO: dieses file hier vll. noch in einen Ordner IMUs packen, der ein unterordner von peripherals ist. für jedes peripherals
// eines gewissen typs einen ordner in die die files für eine gewisse hw-auusprägung sind.


#define NUM_OF_MPU_60x0

MPU_60x0_obj obj_instance; //TODO das ganze hier vll. noch in einen array wie beim mc ethernettreiber;

extern i2c_ifc i2c_ifc_obj;

/*ret_val: 0 no error during init/0 != error*/
uint8_t mpu60x0_drv_init(uint8_t range_gyro, uint8_t range_acc, uint8_t bandwith) {
	obj_instance.fs_range_gyro = range_gyro;
	obj_instance.fs_range_acc = range_acc;
	obj_instance.bandwith = bandwith;
	return _mpu60x0_drv_config_IMU(obj_instance);
}

uint8_t _mpu60x0_drv_config_IMU(MPU_60x0_obj cfg_data) {
	//mpu60x0_drv_write_reg(REG_SIGNAL_PATH_RESET, 7);
	//mpu60x0_drv_write_reg(REG_I2C_MST_CTRL, 13);
	/*Upon power up, the MPU-60X0 clock source defaults to the internal oscillator. However, it is highly
	recommended that the device be configured to use one of the gyroscopes (or an external clock
	source) as the clock reference for improved stability. The clock source can be selected according to
	the following table.*/
	uint8_t init_stat = 0;
	init_stat |= mpu60x0_drv_write_reg(REG_PWR_MGMT_1, 1); //select gyroscope as clock reference
	init_stat |= mpu60x0_drv_write_reg(REG_CONFIG, 0x0); //disable FSYNC and set gyro and accelo-bandwith to maximum, see p. 13
	//now lets configure the sample rate
	uint8_t SMPLRT_DIV = 31;
	init_stat |= mpu60x0_drv_write_reg(REG_SMPRT_DIV, SMPLRT_DIV); //divide 8khz gyrosamplerate by 32 to achieve 4ms
	//sample period which is main loop runtime see p. 12, 13 //TODO: der wert den hier reinschreibst ist hardwarespezifisch
	//auf anderen ucontrollerplattformen wird das anders sein, noch generischer machen, d.h. sample rate als funktionsargument
	//übergeben
	//TODO: self test des gyro und accelo
	uint8_t reg_val = cfg_data.fs_range_gyro << FS_SEL;
	init_stat |= mpu60x0_drv_write_reg(REG_GYRO_CONIG, reg_val); //set Full Scale Range of gyro to maximum value, see p. 14
	reg_val = cfg_data.fs_range_acc << AFS_SEL;
	init_stat |= mpu60x0_drv_write_reg(REG_ACLO_CONIG, reg_val); //set Full Scale Range of accelo to maximum value, see p. 15
	//now lets enable write of gyro and accelerometer values to fifo, see p. 16
	reg_val = (1<<XG_FIFO_EN)|(1<<ZG_FIFO_EN)|(1<<YG_FIFO_EN)|(1<<ACCEL_FIFO_EN);
	init_stat |= mpu60x0_drv_write_reg(REG_FIFO_EN, reg_val);
	reg_val = (1<<2)|(1<<FIFO_EN);; //enable fifo, set everthing else to 0, see p. 38
	init_stat |= mpu60x0_drv_write_reg(REG_USER_CTRL, reg_val);
	reg_val = (1<<4); //clear isr bits on any read, see p. 26
	init_stat |= mpu60x0_drv_write_reg(REG_INT_PIN_CFG, reg_val);
	reg_val = 1; //only enable the data ready interrupt, see p. 27
	init_stat |= mpu60x0_drv_write_reg(REG_INT_ENABLE, reg_val);
	return init_stat;
}

/*1 = Error, 0 = No Error during writing*/
uint8_t mpu60x0_drv_write_reg(uint8_t reg_addr, uint8_t reg_data) {
	uint8_t mpu_addr = (uint8_t)MPU_60x0_I2C_ADDR;
	return i2c_ifc_obj.i2c_imu_reg_write(mpu_addr, reg_addr, reg_data); //start a write to desired register
}

uint8_t mpu60x0_drv_read_reg(uint8_t reg_addr) {
	uint8_t readval;
	uint8_t imu_addr = (uint8_t)MPU_60x0_I2C_ADDR;
	i2c_ifc_obj.i2c_imu_reg_read(imu_addr, reg_addr, &readval);
	//mpu60x0_drv_burst_read_reg(reg_addr, &readval, 1);
	return readval; //TODO: besser: �ber den return value den succes der op zur�ckgeben
}

uint8_t mpu60x0_drv_burst_read_reg(uint8_t reg_addr, uint8_t* buffer, uint8_t bursts) {
	uint8_t success = 0;
	uint8_t mpu_addr = MPU_60x0_I2C_ADDR;
	uint8_t call_reg[2] = {mpu_addr, reg_addr};
	//to read a MPU register first a write and then a read to the register must be performed
	call_reg[0] &= ~(1<<0); //set write bit

	i2c_ifc_obj.i2c_start_com(call_reg, 2); //start a write to desired register

	call_reg[0] |= (1<<0); //set read bit
	i2c_ifc_obj.i2c_start_com(call_reg, bursts);  //now lets read the register
	
	while (!i2c_ifc_obj.i2c_receive(buffer,bursts));
	
	return success; //TODO: hier noch den wirklichen success richtig r�berbringen
}


/*TODO: das hier liefert keine wirklichen werte, irgendwas l�uft da falsch: untersuchungsergebnisse
der der fifo buffer count value liefert denke ich immer null zur�ck, ein "normales" (kein burst) auslesen des fifo register
liefert meist dieselben werte wie der burst read, manchmal aber tats�chlich realistische werte*/
uint8_t mpu60x0_drv_burst_read_fifo(uint8_t* buffer) {//TODO: vll bursts �berhaupt nicht als parameter �bergeben sondern einfach den bufferpointer
	//auslesen und diesen als burst zahl �bergeben
	//TODO: noch den fall pr�fen ob es einen fifo overflow gab und dann entsprechend behandeln
	uint8_t fifo_count_h, fifo_count_l;
	
	fifo_count_h = mpu60x0_drv_read_reg(REG_FIFO_COUNT_H); //read fifo_count_h first to ensure most current fifo count value, fifo_count needs to be read high to low
	fifo_count_l = mpu60x0_drv_read_reg(REG_FIFO_COUNT_L); 
	//discard the upper 8 bits of fifo count. only latest gyro and accelerometer data is necessary
	//if ((fifo_count_h) || (fifo_count_l)) 
	mpu60x0_drv_burst_read_reg(REG_FIFO_R_W, buffer, 12); //TODO: die anzahl der ausgelesen bytes sollte eigentlich
	//steuerbar sein. entweder �ber funktionsparameter �bergeben oder �ber den fifo_counter
	return ((fifo_count_h) | (fifo_count_l)); //TODO: hier noch den succes der operation zur�ckgeben
}



#ifdef FIXED_POINT

/*
this function reads the gyroscope and accelerometer registers and converts the read ADC values into the proper
values in �/s and g based on the configuration of the sensors
*/
uint8_t mpu60x0_drv_get_sensor_data(MPU_60x0_obj cfg, volatile sensordata* accel, volatile sensordata* gyro, uint8_t* fifo_data) {
	
	//uint8_t fifo_data[12];
	mpu60x0_drv_burst_read_fifo(fifo_data);

	//Data is written to the FIFO in order of register number (from lowest to highest). see p. 44
	//...assign
	uint8_t acc_xout_h = fifo_data[0]; 
	uint8_t acc_xout_l = fifo_data[1]; 
	uint8_t acc_yout_h = fifo_data[2]; 
	uint8_t acc_yout_l = fifo_data[3]; 
	uint8_t acc_zout_h = fifo_data[4]; 
	uint8_t acc_zout_l = fifo_data[5];
	uint8_t gyro_xout_h = fifo_data[6];
	//int8_t gyro_xout_l = fifo_data[7]; //lower 8-bit of the gyros measurements are discard see further down
	uint8_t gyro_yout_h = fifo_data[8];
	//int8_t gyro_yout_l = fifo_data[9];
	uint8_t gyro_zout_h = fifo_data[10];
	//int8_t gyro_zout_l = fifo_data[11];
	
	uint8_t lsb_sensitivity;
	int16_t offset;
	//see page 31
	switch (cfg.fs_range_acc) { 
		case 0: lsb_sensitivity = 131; offset = 4; break;
		case 1: lsb_sensitivity = 65; offset = 8; break;
		case 2: lsb_sensitivity = 33; offset = 16; break;
		case 3: lsb_sensitivity = 16; offset = 32; break;//TODO: die nachkommawerte der lsb sensitivity werden hier abgeschnitten. vll noch eine 
		default: return -1;
		//l�sung finden die auf den nach kommabereich mit ber�cksichtigt
	}
	//TODO: dieses ganze file ist eigentlich ein HW-unabh�ngiges modul: am besten den ganzen code so um schreiben das es modularer wird, 
	//er mit dem atmega spezifischen teil �ber interfaces agiert
	
	accel->a_x = ((int16_t)((((uint16_t)acc_xout_h)<<8 | acc_xout_l) / lsb_sensitivity)) - offset;
	
	accel->a_y = ((int16_t)((((uint16_t)acc_yout_h)<<8 | acc_yout_l) / lsb_sensitivity)) - offset;
	
	accel->a_z = ((int16_t)((((uint16_t)acc_zout_h)<<8 | acc_zout_l) / lsb_sensitivity)) - offset;
	
	switch (cfg.fs_range_gyro) { //see page 29
		case 0: lsb_sensitivity = 14 - 8; offset = 250; break;
		case 1: lsb_sensitivity = 13 - 8; offset = 500; break;
		case 2: lsb_sensitivity = 12 - 8; offset = 1000; break;
		case 3: lsb_sensitivity = 11 - 8; offset = 2000; break; //2**11 = 2048; -8 because lower 8-bit part of measurement can simply be discarded
		default: return -1;
	}
	
	//gyro_xzyout_low 's are not needed since lowest possible right shift is 11 bits, so they simply do not matter
	gyro->vel_x = ((int16_t)((uint16_t)(gyro_xout_h >> lsb_sensitivity))) - offset;
	
	gyro->vel_y = ((int16_t)((uint16_t)(gyro_yout_h >> lsb_sensitivity))) - offset;
	
	gyro->vel_z = ((int16_t)((uint16_t)(gyro_zout_h >> lsb_sensitivity))) - offset;
	
	return 1;
}
//die hier nehmen, zwar nicht optimal, aber wenigstens funktioniert sie
uint8_t mpu60x0_drv_get_sensor_data_v1(sensordata* accel, sensordata* gyro) {
	//TODO, das hier ist nimmt fixed point arithmetic an, vll. noch eine version die floating point macht, theoretisch genauer?
	
	//Data is written to the FIFO in order of register number (from lowest to highest). see p. 44
	//...assign
	uint8_t acc_xout_h = mpu60x0_drv_read_reg(REG_ACC_X_H);
	uint8_t acc_xout_l = mpu60x0_drv_read_reg(REG_ACC_X_L);
	uint8_t acc_yout_h = mpu60x0_drv_read_reg(REG_ACC_Y_H);
	uint8_t acc_yout_l = mpu60x0_drv_read_reg(REG_ACC_Y_L);
	uint8_t acc_zout_h = mpu60x0_drv_read_reg(REG_ACC_Z_H);
	uint8_t acc_zout_l = mpu60x0_drv_read_reg(REG_ACC_Z_L);
	uint8_t gyro_xout_h = mpu60x0_drv_read_reg(REG_GYRO_X_H);
	uint8_t gyro_xout_l = mpu60x0_drv_read_reg(REG_GYRO_X_L); //lower 8-bit of the gyros measurements are discard see further down
	uint8_t gyro_yout_h = mpu60x0_drv_read_reg(REG_GYRO_Y_H);
	uint8_t gyro_yout_l = mpu60x0_drv_read_reg(REG_GYRO_Y_L);
	uint8_t gyro_zout_h = mpu60x0_drv_read_reg(REG_GYRO_Z_H);
	uint8_t gyro_zout_l = mpu60x0_drv_read_reg(REG_GYRO_Z_L);
	
	uint8_t lsb_sensitivity;
	uint16_t velocity, subcomma_value, subcomma_mask, subcomma_area;

	//TODO: dieses ganze file ist eigentlich ein HW-unabh�ngiges modul: am besten den ganzen code so um schreiben das es modularer wird,
	//er mit dem atmega spezifischen teil �ber interfaces agiert
		//gyro_xzyout_low 's are not needed since lowest possible right shift is 11 bits, so they simply do not matter
		//TODO: wahrscheinlich musst du hier auch noch durch die LSB sensitivity teilen, wie beim accelo
	switch (obj_instance.fs_range_gyro) {
		case 0: lsb_sensitivity = 131; break;
		case 1: lsb_sensitivity = 66; break;
		case 2: lsb_sensitivity = 33; break;
		case 3: lsb_sensitivity = 16; break; //2**11 = 2048; -8 because lower 8-bit part of measurement can simply be discarded
		default: return -1;
	}
	//TODO: theoretisch konne man noch den subcomma bereich ermitteln aber das lassen wir erst mal 4.1�,4� oder 4.5� ist jetzt nich so
	//entscheidend
    velocity = (((uint16_t) gyro_xout_h)<<8)|gyro_xout_l;
	gyro->vel_x = ((int16_t)velocity) / lsb_sensitivity; //convert to �/s
		
	velocity = (((uint16_t) gyro_yout_h)<<8)|gyro_yout_l;
	gyro->vel_y = ((int16_t)velocity) / lsb_sensitivity; //convert to �/s
		
	velocity = (((uint16_t) gyro_zout_h)<<8)|gyro_zout_l;
	gyro->vel_z = ((int16_t)velocity) / lsb_sensitivity; //convert to �/s
	
	switch (obj_instance.fs_range_acc) { //see page 29   
		/*the offset is the full-scale range. that is the minimum value has to be the negative end of the full scale range
		lsb sensitivity is 2**16/(2*full_scale_range)*/
		case 0: lsb_sensitivity = 14; break;
		case 1: lsb_sensitivity = 13; break;
		case 2: lsb_sensitivity = 12; break;
		case 3: lsb_sensitivity = 11; break; //2**11 = 2048;
		default: return -1;
	}
	velocity = ((uint16_t)acc_xout_h << 8) | acc_xout_l;
	if (velocity & (1<<15)) {//if negative, build two complements to attain value in magnitue
		velocity = (~velocity) + 1; //twos complement
	}	
	//lets calculate subcomma value
	subcomma_mask = (1 << (lsb_sensitivity + 1))-1;
	subcomma_area = (velocity & subcomma_mask)<<5; //TODO: <<5 assumes lsb_sensitivity = 11,
	subcomma_value = subcomma_area/2048; // TODO: 2048 assumes lsb_sensitivity = 11, see 29
	
	velocity >>= lsb_sensitivity;
	accel->a_x = (int16_t)velocity << 5 | subcomma_value; //TODO: das hier k�nnte man automatisch mit 16-lsb_sensitivity machen
	
	velocity = ((uint16_t)acc_yout_h << 8) | acc_yout_l;
	if (velocity & (1<<15)) {//if negative, build two complements to attain value in magnitue
		velocity = (~velocity) + 1; //twos complement
	}
		//lets calculate subcomma value
	subcomma_mask = (1 << (lsb_sensitivity + 1))-1;
	subcomma_area = (velocity & subcomma_mask)<<5; //TODO: <<5 assumes lsb_sensitivity = 11,
	subcomma_value = subcomma_area/2048; // TODO: 2048 assumes lsb_sensitivity = 11, see 29
	velocity >>= lsb_sensitivity;
	accel->a_y = (int16_t)velocity << 5 | subcomma_value; //TODO: das hier k�nnte man automatisch mit 16-lsb_sensitivity machen
	
	velocity = ((uint16_t)acc_zout_h << 8) | acc_zout_l;
	if (velocity & (1<<15)) {//if negative, build two complements to attain value in magnitue
			velocity = (~velocity) + 1; //twos complement
	}
	//lets calculate subcomma value
	subcomma_mask = (1 << (lsb_sensitivity + 1))-1;
	subcomma_area = (velocity & subcomma_mask)<<5; //TODO: <<5 assumes lsb_sensitivity = 11, 
	subcomma_value = subcomma_area/2048; // TODO: 2048 assumes lsb_sensitivity = 11, see 29
	velocity >>= lsb_sensitivity;
	accel->a_z = (int16_t)velocity << 5 | subcomma_value; //TODO: das hier k�nnte man automatisch mit 16-lsb_sensitivity machen
	
	return 1;
}	

#else

//void mpu60x0_drv_get_sensor_data_v1_helper() {} //TODO: diese funktion noch static inline machen, für maximale performance? allerdings
//würde dann der vorteil der codeersparnis verloren gehen

//return accelerometer values in g's and gyroscope values in degrees per second

uint8_t mpu60x0_drv_get_sensor_data_v1(sensordata* accel, sensordata* gyro) {

	//Data is written to the FIFO in order of register number (from lowest to highest). see p. 44
	//...assign
	uint16_t acc_xout_h = mpu60x0_drv_read_reg(REG_ACC_X_H);
	uint8_t acc_xout_l = mpu60x0_drv_read_reg(REG_ACC_X_L);
	uint16_t acc_yout_h = mpu60x0_drv_read_reg(REG_ACC_Y_H);
	uint8_t acc_yout_l = mpu60x0_drv_read_reg(REG_ACC_Y_L);
	uint16_t acc_zout_h = mpu60x0_drv_read_reg(REG_ACC_Z_H);
	uint8_t acc_zout_l = mpu60x0_drv_read_reg(REG_ACC_Z_L);
	uint16_t gyro_xout_h = mpu60x0_drv_read_reg(REG_GYRO_X_H);
	uint8_t gyro_xout_l = mpu60x0_drv_read_reg(REG_GYRO_X_L); //lower 8-bit of the gyros measurements are discard see further down
	uint16_t gyro_yout_h = mpu60x0_drv_read_reg(REG_GYRO_Y_H);
	uint8_t gyro_yout_l = mpu60x0_drv_read_reg(REG_GYRO_Y_L);
	uint16_t gyro_zout_h = mpu60x0_drv_read_reg(REG_GYRO_Z_H);
	uint8_t gyro_zout_l = mpu60x0_drv_read_reg(REG_GYRO_Z_L);
	
	float lsb_sensitivity;
	int16_t velocity;

	//TODO: dieses ganze file ist eigentlich ein HW-unabh�ngiges modul: am besten den ganzen code so um schreiben das es modularer wird,
	//er mit dem atmega spezifischen teil �ber interfaces agiert
	//gyro_xzyout_low 's are not needed since lowest possible right shift is 11 bits, so they simply do not matter
	//TODO: wahrscheinlich musst du hier auch noch durch die LSB sensitivity teilen, wie beim accelo
	switch (obj_instance.fs_range_gyro) {
		case 0: lsb_sensitivity = 131.0; break;
		case 1: lsb_sensitivity = 66.0; break;
		case 2: lsb_sensitivity = 33.0; break;
		case 3: lsb_sensitivity = 16.0; break; //2**11 = 2048; -8 because lower 8-bit part of measurement can simply be discarded
		default: return -1;
	}
	//TODO: eigentlich ist das hier alles codeduplizierung vll. kann man das noch eleganter schreiben
    velocity = (int16_t)((gyro_xout_h<<8)|gyro_xout_l);
	gyro->vel_x = ((float)velocity) / lsb_sensitivity; //convert to degrees/s
		
	velocity = (int16_t)((gyro_yout_h<<8)|gyro_yout_l);
	gyro->vel_y = ((float)velocity) / lsb_sensitivity; //convert to degrees/s
		
	velocity = (int16_t)((gyro_zout_h<<8)|gyro_zout_l);
	gyro->vel_z = ((float)velocity) / lsb_sensitivity; //convert to degrees/s
	
	switch (obj_instance.fs_range_acc) { //see page 29   
		/*the offset is the full-scale range. that is the minimum value has to be the negative end of the full scale range
		lsb sensitivity is 2**16/(2*full_scale_range)*/
		case 0: lsb_sensitivity = 16384.0; break;
		case 1: lsb_sensitivity = 8192.0; break;
		case 2: lsb_sensitivity = 4096.0; break;
		case 3: lsb_sensitivity = 2048.0; break; //2**11 = 2048;
		default: return -1;
	}
	/*TODO: achtung wenn du das high und low register getrennt ließt musst du sichergehen, dass die daten beide von der gleichen messung sind und
	in der zwischenzeit nicht das register mit einer neuen messung geupdatet wurde siehe dazu folgendes im datasheet:
	 The data within the accelerometer sensors’ internal register set is always updated at the Sample
	Rate. Meanwhile, the user-facing read register set duplicates the internal register set’s data values
	whenever the serial interface is idle. This guarantees that a burst read of sensor registers will read
	measurements from the same sampling instant. Note that if burst reads are not used, the user is
	responsible for ensuring a set of single byte reads correspond to a single sampling instant by
	checking the Data Ready interrupt. */

	velocity = (int16_t)((acc_xout_h << 8) | acc_xout_l);
	accel->a_x = ((float)velocity) / lsb_sensitivity;
	//accel->a_x *= 9.81f; //convert g in m/s²
	
	velocity = (int16_t)((acc_yout_h << 8) | acc_yout_l);
	accel->a_y = ((float)velocity) / lsb_sensitivity;
	//accel->a_y *= 9.81f; //convert g in m/s²
	
	velocity = (int16_t)((acc_zout_h << 8) | acc_zout_l);
	accel->a_z = ((float)velocity) / lsb_sensitivity;
	//accel->a_z *= 9.81f; //convert g in m/s²

	return 1;
}

#endif