

#include <stdint.h>
#include "../Core/inc/sys_settings.h"

#ifdef MSPM0C110x

#include <ti/driverlib/dl_i2c.h>
#include <ti\devices\msp\peripherals\hw_i2c.h>

#define I2C0_BASE                   (0x400F0000U)     /*!< Base address of module I2C0 */

#ifdef MPU60x0

volatile uint8_t _i2c_imu_reg_single_read(uint8_t imu_addr, uint8_t reg_addr, uint8_t* readval) {
	//TODO: auf seite 940 "16.2.4.1.3 Read On TX Empty" im datenblatt ist erläutert wie man elegant in sw-
	//einen busteilnehmer ließt, wenn alles proffesionell stehen soll, mach das nach
    I2C_Regs * I2C_INST = (I2C_Regs *)I2C0_BASE;
	imu_addr >>= 1; //discard msb bit
	//DL_I2C_flushControllerTXFIFO(I2C_INST);
	uint16_t timeout = 500;
	uint16_t counter = 0;
	uint8_t array[] = {reg_addr};
	while(!(DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
		counter++; 
		if (counter == timeout)  return 2;
	};
	DL_I2C_flushControllerTXFIFO(I2C_INST);
	DL_I2C_fillControllerTXFIFO(I2C_INST, array, 1);																/*DL_I2C_CONTROLLER_STOP_DISABLE*/
	DL_I2C_startControllerTransferAdvanced(I2C_INST, imu_addr, I2C_MSA_DIR_TRANSMIT, 1,\
	 DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_DISABLE, DL_I2C_CONTROLLER_ACK_ENABLE);
	//TODO: achtung wenn diese funktion zu oft hintereinander aufgerufen wird, füllt sich die fifo und es werden nur noch die reg_addr in den 2 bytes übertragen, 
	//check machen der sicherstellt das alles richtig läuft
	////TODO: 1 vll. auf 2 setzen ist die frage ob die mpu address auch als byte mitzählt oder nur die registeraddresse
	//while (DL_I2C_isControllerTXFIFOFull(I2C_INST)==1);
	//for (volatile uint16_t i = 0; i<(volatile uint16_t)1000; ++i);//TODO: bei endgültiger lösung zu entfernen
	//while(DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE);
	counter = 0;
	while (!(DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY)) {
		counter++; 
		if (counter == timeout) return 2;
	};
	counter = 0;
	//while(DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
	while (DL_I2C_getControllerStatus(I2C_INST) & (DL_I2C_CONTROLLER_STATUS_BUSY)) {
		counter++; 
		if (counter == timeout) return 2;
	}; //TODO: noch eine timeout-funktion
	/*while(!(DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_DATA_ACK));*/
	DL_I2C_startControllerTransferAdvanced(I2C_INST, imu_addr, I2C_MSA_DIR_RECEIVE, 1,\
	 DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_ENABLE, DL_I2C_CONTROLLER_ACK_DISABLE); 
	 //TODO: 1, das burst length argument legt beim empfangen die anzahl an zu empfangenden bytes fest nehme ich an, d.h. 1 byte soll empfangen warden, dann NACK und STOP
        //vll. legt es auch die anzahl an bytes fest die in der kommunikation versendet warden, dann auf 2 setzen
		//while (DL_I2C_isControllerBusy(I2C_INST));
	counter = 0;
	while(DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE) {
		counter++; 
		if (counter == timeout) return 2;
	};
	counter = 0;
	//while(!(DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS));
	while (DL_I2C_getControllerStatus(I2C_INST) & (DL_I2C_CONTROLLER_STATUS_BUSY)) {
		counter++; 
		if (counter == timeout) return 2;
	};

	volatile uint32_t status;
	counter = 0;
	do {
		++counter;
		status = DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_ERROR;
	} while((DL_I2C_getControllerRXFIFOCounter(I2C_INST) == 0) && (!status) && (counter != timeout)); //TODO: über ein gewisses flag das dann gecleart wäre das ganze hier sicherer
	if (counter == timeout) return 2;
	if (status) return 1; //TIMEout noch mit einem timer des ucontroller machen damit das wirklich parallel läuft, sonst unnötiger overhead
	//und eleganter. theoretisch können sich noch ungelesene bytes in der rxfifo befinden, bevor das hier angeforderte byyte wirklich
	//eingetroffen ist6
	//TODO: hier noch checks einbauen ob die communikation ohne fehler verlaufen ist.
	*readval = DL_I2C_receiveControllerData(I2C_INST);
	return 0; //TODO: vll hier noch entsprechenden opcode zurückgeben
}

volatile uint8_t ____i2c_imu_reg_single_read(uint8_t imu_addr, uint8_t reg_addr, uint8_t* readval) {
	
	imu_addr >>= 1;
	I2C_Regs * I2C_INST = (I2C_Regs *)I2C0_BASE;
	DL_I2C_flushControllerTXFIFO(I2C_INST);
	DL_I2C_fillControllerTXFIFO(I2C_INST, &reg_addr, 1);
	DL_I2C_enableControllerReadOnTXEmpty(I2C_INST);
	DL_I2C_startControllerTransferAdvanced(I2C_INST, imu_addr, I2C_MSA_DIR_RECEIVE, 3,\
	 DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_ENABLE, DL_I2C_CONTROLLER_ACK_DISABLE);
	uint16_t timeout, counter = 0; 
	timeout = 500;
	while(DL_I2C_getControllerRXFIFOCounter(I2C_INST) == 0) {
		if (++counter == timeout) return 2;
	};
	*readval = DL_I2C_receiveControllerData(I2C_INST);
	return 0; //TODO: vll hier noch entsprechenden opcode zurückgeben
}

uint8_t i2c_imu_reg_single_read(uint8_t imu_addr, uint8_t reg_addr,  uint8_t* readval) {

	//NVIC_DisableIRQ(15);
	I2C_Regs * i2c = (I2C_Regs *)I2C0_BASE;
	imu_addr >>= 1;

	DL_I2C_flushControllerTXFIFO(i2c);

	DL_I2C_fillControllerTXFIFO(i2c, &reg_addr, 1);

	uint32_t mask = 0x7FF;

	i2c->MASTER.MSA &= ~mask;

    i2c->MASTER.MSA |= (imu_addr << 1) | (uint32_t) 1;

	uint32_t rd_on_txempty, stop_en, start_en, ack_en, burst_en, mblen;
	mblen = rd_on_txempty = stop_en = start_en = burst_en = 1;
	ack_en = 0;

	i2c->MASTER.MCTR = (mblen << 16) | (rd_on_txempty<<5) | (stop_en << 2) \
	| (ack_en << 3) | (start_en << 1) | (burst_en);
	uint16_t timeout, counter = 0; 
	timeout = 500;
	while(DL_I2C_getControllerRXFIFOCounter(i2c) == 0) {
		if (++counter == timeout) {/*NVIC_EnableIRQ(15);*/ return 2;}
	};
	*readval = DL_I2C_receiveControllerData(i2c);
	return 2;
}

	//NVIC_EnableIRQ(15);
	/*uint32_t direction = auf read stellen;
    DL_Common_updateReg(&i2c->MASTER.MSA,
        ((targetAddr << I2C_MSA_SADDR_OFS) | (uint32_t) direction),
        (I2C_MSA_SADDR_MASK | I2C_MSA_DIR_MASK));

	uint32_t READ_ON_TX_EMPTY = 1;
	
    DL_Common_updateReg(&i2c->MASTER.MCTR,
        (((uint32_t) 1 << I2C_MCTR_MBLEN_OFS) | I2C_MCTR_BURSTRUN_ENABLE |
             I2C_MCTR_START_ENABLE | I2C_MCTR_STOP_ENABLE | I2C_MCTR_ACK_DISABLE),
        (I2C_MCTR_MBLEN_MASK | I2C_MCTR_BURSTRUN_MASK | I2C_MCTR_START_MASK |
            I2C_MCTR_STOP_MASK | I2C_MCTR_ACK_MASK));*/

uint8_t ___i2c_imu_reg_single_read(uint8_t imu_addr, uint8_t reg_addr, uint8_t* readval) {
	volatile uint8_t retval = 0;
	I2C_Regs * I2C_INST = (I2C_Regs *)I2C0_BASE;
	if ((retval = i2c_imu_reg_single_read(imu_addr, reg_addr, readval)) == 2) {
		DL_I2C_reset(I2C_INST);
		DL_I2C_enableController(I2C_INST);
		DL_I2C_resetControllerTransfer(I2C_INST);
	}
	/*if (retval == 2) {
		I2C_Regs * I2C_INST = (I2C_Regs *)I2C0_BASE;
		DL_I2C_resetControllerTransfer(I2C_INST);
	}*/
	return retval;
}

uint8_t i2c_imu_reg_single_write(uint8_t imu_addr, uint8_t reg_addr, uint16_t reg_val) {
	imu_addr >>= 1; //discard msb bit
	I2C_Regs * I2C_INST = (I2C_Regs *)I2C0_BASE;
	uint8_t array[] = {reg_addr, reg_val};
	//imu_addr &= ~(1<<0); //set write bit das write bit setzt du vermutlich mit der function DL_I2C_startControllerTransferAdvanced
	//TODO: vorsichtshalber vorher tx fifo flushen, bevor sie mit dem array füllst.
	//while (DL_I2C_getControllerStatus(I2C_INST) == (DL_I2C_CONTROLLER_STATUS_BUSY|DL_I2C_TARGET_STATUS_BUS_BUSY));
	/*volatile uint32_t status;
	do {status = DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE;}
	while (status == (uint32_t)0); //TODO: das hier vll. als nicht blockierend implementierenÜ*/
	while (!(DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
		/*counter++; 
		if (counter == timeout) return 2;*/
	};
	DL_I2C_flushControllerTXFIFO(I2C_INST);// TODO: das hier einfügen wenn das verhalten undeterministisch wird
	uint8_t writes = DL_I2C_fillControllerTXFIFO(I2C_INST, array, 2);//TODO: achtung wenn diese funktion zu oft hintereinander
	//aufgerufen wird, füllt sich die fifo und es werden nur noch die reg_addr in den 2 bytes übertragen, check machen der sicherstellt das alles richtig läuft
	if (writes != 2) return 1;
	DL_I2C_startControllerTransferAdvanced(I2C_INST, imu_addr, I2C_MSA_DIR_TRANSMIT, 2, \
	DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_ENABLE, DL_I2C_CONTROLLER_ACK_ENABLE);
	//while (DL_I2C_isControllerTXFIFOFull(I2C_INST)==1);
	while (DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE);
	while (DL_I2C_getControllerStatus(I2C_INST) & (DL_I2C_CONTROLLER_STATUS_BUSY));
	if (DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) return 1; //TODO: noch entsprechende opcodes zurückgeben
	//DL_I2C_transmitTargetDataBlocking(I2C_INST, 0);
	//TODO: 1 vll. auf 3 setzen ist die frage ob
	//die mpu address auch als byte mitzählt oder nur die registeraddresse
	return 0; //TODO: noch entsprechende opcodes zurückgeben
}

uint8_t _i2c_imu_reg_single_write(uint8_t imu_addr, uint8_t reg_addr, uint16_t reg_val) {
	uint8_t retval;// = _i2c_imu_reg_single_write(imu_addr, reg_addr, reg_val);
	I2C_Regs * I2C_INST = (I2C_Regs *)I2C0_BASE;
	while ((retval = i2c_imu_reg_single_write(imu_addr, reg_addr, reg_val)) == 2) {
		DL_I2C_reset(I2C_INST);
		DL_I2C_resetControllerTransfer(I2C_INST);
	}
	//DL_I2C_reset(I2C_INST);
	/*if (retval == 2) {
		I2C_Regs * I2C_INST = (I2C_Regs *)I2C0_BASE;
		DL_I2C_resetControllerTransfer(I2C_INST);
	}*/
	return retval;
}

#endif

#endif