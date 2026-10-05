/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "Core/inc/crsf_drv.h"
#include "Bindings/i2c_implements.h"
#include "Peripherals/MPU60x0.h"
#include "Core/inc/helper_functions.h"
#include "Core/inc/flight_controll.h"
#include "Core/inc/init.h"
#include <stdlib.h>
//#include "dl_gpio.h"


void USART_0_write_string(char* string, uint8_t len) {
    uint16_t i = 0;
	//DL_UART_transmitDataBlocking(UART_0_INST, (uint8_t)'b');
    while (i<len) {
		//DL_UART_transmitDataBlocking(UART_0_INST, (uint8_t)'a');
        DL_UART_transmitDataBlocking(UART_0_INST, (uint8_t)string[i]);
        ++i;
    }
}

void print_IMU_values() {
		sensordata acc, gyro;
		char string[37] =   "\n\racce data:x      ,y      ,z      ";
		char string_2[37] = "\n\rgyro data:x      ,y      ,z      ";
		char x_data[6], y_data[6], z_data[6];
		//x_data[5] = y_data[5] = z_data[5] = '\0';
		char x_sign, y_sign, z_sign;
			 mpu60x0_drv_get_sensor_data_v1(&acc, &gyro);
			 x_sign = y_sign = z_sign = '+';
			 if (num_to_char(abs((int)gyro.vel_x), x_data) == 0)
			 ;// {_delay_ms(10000);PORTB = 0b00000010;}
			 if (gyro.vel_x < 0) x_sign = '-';
			 if (num_to_char(abs((int)gyro.vel_y), y_data) == 0)
			 ;// {_delay_ms(10000);PORTB = 0b00000011;}
			 if (gyro.vel_y < 0) y_sign = '-';
			 if (num_to_char(abs((int)gyro.vel_z), z_data) == 0)
			 ;// {_delay_ms(10000);PORTB = 0b00000100;}
			 if (gyro.vel_z < 0) z_sign = '-';
			 string_2[13] = x_sign;//x_data[0];
			 string_2[14] = x_data[0];
			 string_2[15] = x_data[1];
			 string_2[16] = x_data[2];
			 string_2[17] = x_data[3];
			 string_2[17] = x_data[4];
			 string_2[20] = y_sign;//y_data[0];
			 string_2[21] = y_data[0];
			 string_2[22] = y_data[1];
			 string_2[23] = y_data[2];
			 string_2[24] = y_data[3];
			 string_2[25] = y_data[4];
			 string_2[27] = z_sign;//z_data[0];
			 string_2[28] = z_data[0];
			 string_2[29] = z_data[1];
			 string_2[30] = z_data[2];
			 string_2[31] = z_data[3];
			 string_2[32] = z_data[4];//z_data[0];
			 USART_0_write_string(string_2, 33);
			 x_sign = y_sign = z_sign = '+';
			 if (num_to_char(abs((int)acc.a_x), x_data) == 0)
			 ;// {_delay_ms(10000);PORTB = 0b00000010;}
			 if (acc.a_x < 0) x_sign = '-';
			 if (num_to_char(abs((int)acc.a_y), y_data) == 0)
			 ;// {_delay_ms(10000);PORTB = 0b00000011;}
			 if (acc.a_y < 0) y_sign = '-';
			 if (num_to_char(abs((int)acc.a_z), z_data) == 0)
			 ;// {_delay_ms(10000);PORTB = 0b00000100;}
			 if (acc.a_z < 0) z_sign = '-';
			 string[13] = x_sign;//x_data[0];
			 string[14] = x_data[0];
			 string[15] = x_data[1];
			 string[16] = x_data[2];
			 string[17] = x_data[3];
			 string[18] = x_data[4];
			 string[20] = y_sign;//y_data[0];
			 string[21] = y_data[0];
			 string[22] = y_data[1];
			 string[23] = y_data[2];
			 string[24] = y_data[3];
			 string[25] = y_data[4];
			 string[27] = z_sign;//z_data[0];
			 string[28] = z_data[0];
			 string[29] = z_data[1];
			 string[30] = z_data[2];
			 string[31] = z_data[3];
			 string[32] = z_data[4];//z_data[0];
			 USART_0_write_string(string, 33);
}


void print_IMU_values_f() {
		sensordata acc, gyro;
		char string[37] =   "\n\racce data:x      ,y      ,z      ";
		char string_2[37] = "\n\rgyro data:x      ,y      ,z      ";
		char x_data[6], y_data[6], z_data[6];
		//x_data[5] = y_data[5] = z_data[5] = '\0';
		char x_sign, y_sign, z_sign;
		mpu60x0_drv_get_sensor_data_v1(&acc, &gyro);
		acc.a_x *= 10;
		acc.a_z *= 10;
		acc.a_y *= 10;
		gyro.vel_x *= 10;
		gyro.vel_y *= 10;
		gyro.vel_z *= 10;
		x_sign = y_sign = z_sign = '+';
		if (num_to_char(abs((int)gyro.vel_x), x_data) == 0)
		;// {_delay_ms(10000);PORTB = 0b00000010;}
		if (gyro.vel_x < 0) x_sign = '-';
		if (num_to_char(abs((int)gyro.vel_y), y_data) == 0)
		;// {_delay_ms(10000);PORTB = 0b00000011;}
		if (gyro.vel_y < 0) y_sign = '-';
		if (num_to_char(abs((int)gyro.vel_z), z_data) == 0)
		;// {_delay_ms(10000);PORTB = 0b00000100;}
		if (gyro.vel_z < 0) z_sign = '-';
		string_2[13] = x_sign;//x_data[0];
		string_2[14] = x_data[0];
		string_2[15] = x_data[1];
		string_2[16] = x_data[2];
		string_2[17] = x_data[3];
		string_2[18] = '.';
		string_2[19] = x_data[4];
		string_2[20] = y_sign;//y_data[0];
		string_2[21] = y_data[0];
		string_2[22] = y_data[1];
		string_2[23] = y_data[2];
		string_2[24] = y_data[3];
		string_2[25] = '.';
		string_2[26] = y_data[4];
		string_2[27] = z_sign;//z_data[0];
		string_2[28] = z_data[0];
		string_2[29] = z_data[1];
		string_2[30] = z_data[2];
		string_2[31] = z_data[3];
		string_2[32] = '.';
		string_2[33] = z_data[4];//z_data[0];
		USART_0_write_string(string_2, 34);
		x_sign = y_sign = z_sign = '+';
		if (num_to_char(abs((int)acc.a_x), x_data) == 0)
		;// {_delay_ms(10000);PORTB = 0b00000010;}
		if (acc.a_x < 0) x_sign = '-';
		if (num_to_char(abs((int)acc.a_y), y_data) == 0)
		;// {_delay_ms(10000);PORTB = 0b00000011;}
		if (acc.a_y < 0) y_sign = '-';
		if (num_to_char(abs((int)acc.a_z), z_data) == 0)
		;// {_delay_ms(10000);PORTB = 0b00000100;}
		if (acc.a_z < 0) z_sign = '-';
		string[13] = x_sign;//x_data[0];
		string[14] = x_data[0];
		string[15] = x_data[1];
		string[16] = x_data[2];
		string[17] = x_data[3];
		string[18] = '.';
		string[19] = x_data[4];
		string[20] = y_sign;//y_data[0];
		string[21] = y_data[0];
		string[22] = y_data[1];
		string[23] = y_data[2];
		string[24] = y_data[3];
		string[25] = '.';
		string[26] = y_data[4];
		string[27] = z_sign;//z_data[0];
		string[28] = z_data[0];
		string[29] = z_data[1];
		string[30] = z_data[2];
		string[31] = z_data[3];
		string[32] = '.';
		string[33] = z_data[4];//z_data[0];
		USART_0_write_string(string, 34);
}

extern MPU_60x0_obj obj_instance;

#define DELAY (16000000)

volatile uint8_t global_var;

/*#pragma vector = 
void I2C0_IRQHandler() {
	global_var = 1;
}*/

/*
file:///C:/Users/slin9/Desktop/Projekte/Drohnen/Projekt%204%20DMF/Datenbl%C3%A4tter%20Komponenten%20Prototyp/MSPM0%20C-Series%2024-MHz%20Microcontrollers%20Technical%20Reference%20Manual.pdf
seite 1060 18.2.5 Output Generator ist die stelle für pwms im datenblatt
*/
// ich nehem an das die function DL_Timer_setCaptureCompareValue das register setzt das bei einem compare match den duty cycle setzt
// aber selbst nicht genau nachgeprüft.
void test_t0_pwm0(uint32_t val) {
	DL_Timer_setCaptureCompareValue(PWM_0_INST, val, DL_TIMER_CC_0_INDEX);
    //DL_TimerG_setCaptureCompareValue(PWM_0_INST, val, DL_TIMER_CC_0_INDEX);
}

void test_t0_pwm1(uint32_t val) {
	DL_Timer_setCaptureCompareValue(PWM_0_INST, val, DL_TIMER_CC_1_INDEX);
    //DL_TimerG_setCaptureCompareValue(PWM_0_INST, val, DL_TIMER_CC_1_INDEX);
}

void test_t1_pwm0(uint32_t val) {
    DL_Timer_setCaptureCompareValue(PWM_1_INST, val, DL_TIMER_CC_0_INDEX);
	//DL_TimerA_setCaptureCompareValue(PWM_1_INST, val, DL_TIMER_CC_0_INDEX);
}

void test_t1_pwm1(uint32_t val) {
    DL_Timer_setCaptureCompareValue(PWM_1_INST, val, DL_TIMER_CC_1_INDEX);
	//DL_TimerA_setCaptureCompareValue(PWM_1_INST, val, DL_TIMER_CC_1_INDEX);
}

void GPIOA_IRQHandler() {
	global_var = 1;
}

typedef enum RX_STATE {
	DETECT_SYNC_BYTE,
	DETECT_LEN_BYTE,
	DETECT_TYPE_BYTE,
	GATHER_PAYLOAD,
} RX_STATE;
//TODO: vll. noch auf 20MHz umsteigen, dann bist du doppelt so schnell und solltest immer noch die 416666 baudrate erreichen können.

//TODO: den code durch ifdefs undso so strukturieren, das wenn hardwaresupport für bestimmte operationen wie modulo besteht auch modulo verwendet wird, ansonsten algorithmen die
//modulo durchführen. also durch makro die hardwarefähigkeiten des uControllers angeben und den code durch direktiven so compilieren lassen, das er genau für diese hardware passt

volatile uint8_t rx_buffer_head = 0;
volatile uint8_t array_3[26];

volatile uint8_t data_ready = 0;
volatile uint8_t app_cmd_processing = 0;
uint8_t i = 0;

extern flight_controller_ifc fc;
volatile RX_STATE state = DETECT_SYNC_BYTE;
volatile uint8_t* next_write_buffer, *new_cmd_data_ptr;
uint8_t* buffer_list[2];

#define USE_ISR

int main(void)
{
    SYSCFG_DL_init(); //TODO: du musst die periode des pwms so stellen das sie kleiner ist als die periode der main loop 
	//(aus offensichtlichen gründen.)
	NVIC_EnableIRQ(UART0_INT_IRQn);
	for (volatile uint32_t i = 0; i<100000; ++i);//TODO: die schleife ist nötig sonst funktioniert der code irgenwie nicht
	//beim unpluggen/pluggen, d.h. entfernen und wieder einstecken der stromzufuhr, mit anderem worten einem reset des gesamt
	//systems durch abwürgen und wiederhertellen der stromzuführ. nur strom schon da ist und dann geflascht wird funktioniert
	//der code wie er das soll. irgendwas braucht beim systemstart mehr zeit als antizipiert, eleganterweise sollte das 
	//identifiziert werden und entsprechend im code abgefragt werden.
	I2C_Regs * I2C_INST = (I2C_Regs *)I2C0_BASE;
	DL_I2C_flushControllerTXFIFO(I2C_INST);
    //RC_Channels_Packed_Payload_frame frame;
    //uint8_t array[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14};
	core_init();
	/*I2C_Regs * I2C_INST = (I2C_Regs *)I2C0_BASE;
	uint8_t	reg_addr, reg_val;
	reg_addr = reg_val = 5;
	uint8_t array_1[] = {reg_addr, reg_val};
	//TODO: 
	//vorher tx fifo flushen, bevor sie mit dem array füllst.
	char gyro_string[] = "gyro_data";
	char accel_string[] = "accel_data";
	//volatile sensordata accel, gyro;*/
	DL_Timer_startCounter(PWM_0_INST);
	DL_Timer_startCounter(PWM_1_INST);
	//volatile uint8_t retval = 0;
	uint8_t array_1[22];
	uint8_t array_2[22];
	buffer_list[0] = array_1;
	buffer_list[1] = array_2;//TODO: das ganze hier vll. noch in eine offizielle initialisierungsfunktion
	uint8_t run;
	run = 1;
	DL_GPIO_initDigitalOutput(6);
	DL_GPIO_enableOutput(GPIO_GRP_0_PORT, 1<<6);
	DL_GPIO_initDigitalOutput(2);
	DL_GPIO_enableOutput(GPIO_GRP_0_PORT, 1<<2);
	DL_GPIO_initDigitalOutput(4);
	DL_GPIO_enableOutput(GPIO_GRP_0_PORT, 1<<4);
	//i2c_imu_reg_single_read(0xd0, 0x75, &retval);
	uint8_t crsf_fr[26];
	crsf_fr[0] = 0xC8;
	crsf_fr[1] = 0x18;
	crsf_fr[2] = 0x16;
	sensordata gyro_data, accel_data;
	attitude desired, actual;
	actual.roll = 0.0f; //TODO: die initalisierung des states sollte man vll. noch in eine offizielle initialisierungsfunktion
	//packen, wie core init
	actual.yaw = 0.0f;
	actual.pitch = 0.0f;
	//attitude new = {.pitch = 0, .roll = 0, .yaw = 0};
	//attitude *new_data  = &new;
	motor_cmds m_controll;
	//volatile uint32_t duty_cylce = 0;
	float thrust;
	RC_Channels_Packed_Payload_frame frame;
	//volatile uint8_t byte_stream[26]; 
	//char j[5];
	//volatile float ax, ay, az, vx, vy, vz;
    while (1) {
		/*duty_cylce += 10;
		if (duty_cylce>1000) duty_cylce = 0;*/
		//i2c_imu_reg_single_write(5, 5, 5);
		//print_IMU_values_f();
		/*if (imu_data_ready||pilot_cmds_update) {
			if (imu_data_ready) imu_data_ready = 0;
			if (pilot_cmds_update) pilot_cmds_update = 0;
		}*/
		DL_GPIO_togglePins(GPIO_GRP_0_PORT, 1<<4);
		if (run) {
			fc.get_sensor_data(&accel_data, &gyro_data);
			fc.calculate_attitude(&actual, gyro_data, accel_data);
#ifdef USE_ISR
			/*desired.pitch = new_data->pitch;
			desired.roll = new_data->roll;
			desired.yaw = new_data->yaw;
			desired.thrust = new_data->thrust;*/
			/*ax = accel_data.a_x;
			ay = accel_data.a_y;
			az = accel_data.a_z;
			vx = gyro_data.vel_x;
			vy = gyro_data.vel_y;
			vz = gyro_data.vel_z;*/
			/*DL_UART_transmitDataBlocking(UART_0_INST, 0xc8);
			DL_UART_transmitDataBlocking(UART_0_INST, 0x18);
			DL_UART_transmitDataBlocking(UART_0_INST, 0x16);
			for (uint8_t i = 0; i<26; ++i) DL_UART_transmitDataBlocking(UART_0_INST, array_3[i]);*/
			
			//for (uint8_t i = 5; i<14; ++i) ++byte_stream[i];
			//TODO: um 100% correct zu sein: CRIT_SECTION_ENTER
			app_cmd_processing = 1;
			//TODO: CRITICAL_SECTION_EXIT
			crsf_drv_deserialize_Channels_Packed_Payload_raw(&frame, new_cmd_data_ptr);
			app_cmd_processing = 0;
			/*frame.channel_01 = (uint16_t)1100;
			frame.channel_02 = 992;
			frame.channel_03 = (uint16_t)1100;
			frame.channel_04 = 992;
			uint16_t channel3 = (uint16_t)(frame.channel_01);
			num_to_char((int16_t)channel3, j);
			for (uint8_t i = 0; i<5; ++i) DL_UART_transmitDataBlocking(UART_0_INST, j[i]);*/
			/*if (finished) {
				DL_UART_transmitDataBlocking(UART_0_INST, 0xc8);
				DL_UART_transmitDataBlocking(UART_0_INST, 0x18);
				DL_UART_transmitDataBlocking(UART_0_INST, 0x16);
				for (uint8_t i = 0; i<26; ++i) DL_UART_transmitDataBlocking(UART_0_INST, array_3[i]);
				finished = 0;
			}*/
			crsf_drv_convert_channels_values_into_attitude(&desired, &frame);
			crsf_drv_convert_channels_values_into_thrust(&thrust, &frame);
			/*desired.pitch = 1;
			desired.roll = 1;
			desired.yaw = 1;
			thrust = 50;*/
#else
			fc->retrieve_commands(&desired);
#endif
			fc.pid_controller(&m_controll, desired, actual);
			fc.motor_controll(thrust, m_controll.roll_rate, m_controll.yaw_rate, m_controll.pitch_rate);
			//DL_GPIO_togglePins(GPIO_GRP_0_PORT, 1<<2);
			//fc_steer_drone();
		}
		//for (volatile uint32_t i = 0; i<100000;++i);
		//retval = i2c_imu_reg_single_write(0xd0, 11, 5);
		//if (retval == 0) DL_UART_transmitDataBlocking(UART_0_INST, 'a');
		//else DL_UART_transmitDataBlocking(UART_0_INST, 'b');
	}
}

void UART0_IRQHandler() {
	DL_GPIO_togglePins(GPIO_GRP_0_PORT, 1<<2);
	//TODO: achtung wenn die main schleife so langsam ist, das in der zwischenzeit hier mehrere pakete empfangen werden, ist die isr
	//zu den zeitpunkten in denen sie bytes der zwischenpakete empfägnt unnötiger overhead->herausfinden ob das der fall ist und dann bessere
	//lösung finden/
	uint8_t new_byte = DL_UART_receiveDataBlocking(UART_0_INST);
	switch (state) {
		case DETECT_SYNC_BYTE: if (new_byte == (uint8_t)0xC8) state = DETECT_LEN_BYTE;
		break;
		case DETECT_LEN_BYTE: if (new_byte == (uint8_t)0x18) state = DETECT_TYPE_BYTE;
							  else state = DETECT_SYNC_BYTE;
		break;
		case DETECT_TYPE_BYTE:
		if (new_byte == (uint8_t)0x16) {
			state = GATHER_PAYLOAD;
			next_write_buffer = buffer_list[i];
			i = (i + 1) % 2;
			rx_buffer_head = 0;
		} else state = DETECT_SYNC_BYTE;
		break;
		case GATHER_PAYLOAD:
		DL_GPIO_togglePins(GPIO_GRP_0_PORT, 1<<4);
		next_write_buffer[rx_buffer_head++] = new_byte; //auskommentierenfree_ptr->payload_ptr->byte[rx_buffer_head++] = new_byte;
		//++bytes_received;
		if (rx_buffer_head == 8) data_ready = 1;
		if (data_ready == 1) {//only need the first 4 channels, i.e. 8 bytes
			if (!app_cmd_processing) {
				new_cmd_data_ptr = next_write_buffer; 
				data_ready = 0;
			}
		}
		//TODO: hier noch code schreiben der markiert welcher channel gerade verfügbar geworden ist, mit anderen wortren nutze den channel_processing_status member
		if (rx_buffer_head == 22) {//TODO: hier wird die crc nicht mehr in die rohdaten mitaufgenommen: 100%korrekt wäre es natürlich wenn du sie auch mitaufnimmst und sie dann 
			state = DETECT_SYNC_BYTE;
			//finished = 1;
		}
		break;
		default: state = DETECT_SYNC_BYTE; break;
	}
	//DL_GPIO_togglePins(GPIO_GRP_0_PORT, 1<<4);
	//UART_0_INST->CPU_INT.ICLR |= (1<<10); 
}

			/*for (uint8_t i = 0; i<3; ++i) {
				if (frame_desr_list[i].free == true) {
					free_frame_descr = frame_descr_list[i];
					free_frame_ptr = free_frame_descr->addr;
					break;
				}
			}*/