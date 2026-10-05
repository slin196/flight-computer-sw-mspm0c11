/*
 * crsf_drv.c
 *
 * Created: 25/02/2026 16:42:14
 *  Author: slin9
 */ 

#include <stdint.h>
#include <stdbool.h>
#include "../inc/sensorfusion.h"
#include "../inc/sys_settings.h"
#include "../inc/crsf_drv.h"
#include <string.h>


#define MIN_FRAME_SIZE 4
#define RX_HEAD (*rx_head)
#define RX_TAIL (*rx_tail)

#define TICKS_TO_US(x)  ((((x - (uint16_t)992) * 5)/ 8) + 1500)
#define US_TO_TICKS(x)  (((x - 1500) * 8)/ 5 + 992) //die us eines channels geben die l�nge des duty cycles seines pwm signals an
#define CENTER 1500
#define MAX_CHANNEL_VAL 2000
#define MIN_CHANNEL_VAL 1000

#define MAX_ATTITUDE_ANGLE 50 //in �, TODO: 50 Weil 50/500 dann im nachfolgenden define kein subkomma steckt, ansonsten das makro 
//CONVERSION_RATE und die entsprechende funktion in der das makro converstion rate bentutzt wird auf festkommaoperation
//umstellen TODO: das hier ist nicht spezifisch f�r dieses modul, sondern eine allgemeine systemkonfiguration
// vll. noch in einen allgemeineres header file z.B. sys settings packen
#define US_TO_ANGLE_RATE ((MAX_CHANNEL_VAL - CENTER)/MAX_ATTITUDE_ANGLE) //provides a value to convert the us-value to attitude angle

#ifdef FIXED_POINT
#define US_TO_THRUST(x) ((x-1000)/4) //the max thrust value shall be 250 for ease of conversion
#else
#define US_TO_THRUST(x) (((float)(x+(int16_t)500))/3.9216f) //3.9216f = 1000/255, the max thrust value shall be 255
#endif

typedef struct crsf_drv_obj {
	int i;

} crsf_driver;

typedef enum op_res {
	RES_OK,
	RES_PENDING,
	RES_OP_ERROR
} OP_RES;

struct uart_interface {//das hier vll. noch als eigenes modul machen? sozusagen ein file f�r das com_if 
	OP_RES (*uart_write)(uint8_t*, uint8_t);
	OP_RES (*uart_receive)(uint8_t);
};

struct uart_interface uart_ifc;



uint8_t type_to_length_map[0xAC]; //maps the frame types to their lengths



//TODO: noch eine struct f�r einen extended header frame einf�gen.


uint8_t tx_buffer[256], rx_heap[3*64];
uint8_t tx_data_ptr, rx_next_frame_ptr, rx_last_frame_ptr;
volatile uint8_t *rx_tail, *rx_head, *bytes_in_rx_buffer_ptr; //a queue, assumes that rx_frames are taken and freed in fifo order
//TODO: das muss irgendwann wahrscheinlich mal ein proffesionller heap werden, sowie beim ethernettreiber
//wenn du wirklich den vollen umfang des crsf protokolls nutzen willst um alle pakete zu managen. nin��
//
extern volatile uint8_t rx_buffer_head;
extern volatile uint8_t rx_buffer_tail; // pointer the the last read byte in the rx_buffer TODO: opcodes mit enums?
extern volatile uint8_t time_to_read; 
extern volatile uint8_t rx_buffer_1[1000];
extern volatile uint8_t tx_status;
extern uint16_t	rx_buffer_head_1;
extern volatile uint8_t rx_buffer_space;

extern volatile uint8_t rx_buffer[256];
//TODO: ACHTUNG DIE L�NGE DES RX bUFFERs wird hier gar nicht festgelegt.

/*prototype functions of managing the simplest form of network "heap", a queue*/
uint8_t crsf_drv_malloc(crsf_frame * frame) {
	frame = (crsf_frame *)&rx_heap[rx_next_frame_ptr];
	if ((rx_next_frame_ptr + frame->hdr.len) < rx_last_frame_ptr) {
		rx_next_frame_ptr += frame->hdr.len;
		return 1;
	}
	frame = 0;
	return 0;
}

void crsf_drv_free(crsf_frame *frame) {
	rx_last_frame_ptr = rx_last_frame_ptr + (frame->hdr.len+2); //TODO: nimmt an das rx_last_frame_ptr == frame, relativ dreckige l�sung
}


typedef struct crsf_drv_init {
	volatile uint8_t* rx_buffer_addr;
	volatile uint8_t* rx_tail_addr; //pointer to the next byte to read in the buffer
	volatile uint8_t* rx_head_addr; //pointer to the latest received byte
	volatile uint8_t* rx_buffer_space_ptr;
	//TODO: das ganz hier noch generischer machen, indem du anstatt uint8_t* typ einen typedef machst, damit der 
	//head und tail der queue des uart moduls vom beliebigen typ sein k�nnen.
}crsf_drv_init;

void crsf_driver_init(init_data data) {
	crsf_drv_init* init = (crsf_drv_init*)data;
	tx_data_ptr = 0;
	//rx_buffer = init->rx_buffer_addr; 
	rx_tail = init->rx_tail_addr; 
	rx_head = init->rx_head_addr; 
	rx_next_frame_ptr = *rx_tail;
	bytes_in_rx_buffer_ptr = init->rx_buffer_space_ptr;
	type_to_length_map[GPS] = 15;
	type_to_length_map[GPS_Time] = 9;
	//TODO: map liste noch vervollst�ndigen
	type_to_length_map[RC_Channels_Packed_Payload] = 22;
	//TODO: map liste noch vervollst�ndigen
}

void serialize_32bit(uint8_t offset, uint32_t bytes) {
	tx_buffer[offset] = bytes>>24;
	tx_buffer[offset+1] = (bytes>>16) & 0xFF;
	tx_buffer[offset+2] = (bytes>>8) & 0xFF;
	tx_buffer[offset+3] = bytes & 0xFF;
}

//TODO: ob diese funktion wirklich notwendig ist? vll. bei einer version einer racing drone die auch looping machen kann
//Rate/Acro Mode: The channel value represents the desired angular velocity (degree per second). For example, a 100% roll stick deflection instructs the FC to make the drone rotate at the maximum configured roll rate.
void crsf_drv_convert_channels_values_into_angular_velocity(attitude* desired) {
	
}

#ifdef FIXED_POINT
int16_t map_duty_cycle_to_attitude(uint16_t us) {
#else
float map_duty_cycle_to_attitude(uint16_t us) {
#endif
	int16_t _us = (int16_t) us;
	if (us>MAX_CHANNEL_VAL) _us = MAX_CHANNEL_VAL;
	if (us<MIN_CHANNEL_VAL) _us = MIN_CHANNEL_VAL;
	_us -= CENTER;
#ifdef FIXED_POINT
	return _us / US_TO_ANGLE_RATE; //TODO: achtung hier wird die genauigkeit der gew�nschten position auf den vorkommabereich
	//beschr�nkt, bei einem maximalwert von 500 k�nntest du sieben leftshits von us machen. um auf subkommagenauigkeit zu kommen
	//noch entsprechend anpassen
#else
	return  (float)_us / (float)US_TO_ANGLE_RATE;
#endif
}

/*Angle/Stabilized Mode: The channel value represents a specific angle. The FC uses gyro and accelerometer data to match the roll/pitch angle to the stick position, aiming to return to level when the stick is released.*/
uint8_t crsf_drv_convert_channels_values_into_attitude_v1(attitude* attitude_desired, uint16_t* thrust) {
	RC_Channels_Packed_Payload_frame* payload = 0;
	crsf_frame * frame;
	uint8_t i = rx_last_frame_ptr;
	while (i <= rx_next_frame_ptr) {
		frame = (crsf_frame*)&rx_heap[rx_last_frame_ptr];
		if (frame->hdr.type == RC_Channels_Packed_Payload) {
			payload = (RC_Channels_Packed_Payload_frame*)frame->RC_Channels_Packed_data;
		}
		i += frame->hdr.len;
		crsf_drv_free(frame); //TODO: hier wird einfach alle pakete indiskriminant gefreet bis der heap frei ist, d.h. die aktuellesten befehle vom 
		//radio transmitter gefunden sind. f�r die jetzigen zwecke reicht das, sp�ter bei einem proffesionellem crsf treiber sollten nur die
		// RC_Channels_Packed_Payload frames dealloziert werden.
	}
	if (payload == 0) return 0;
	uint16_t duty_cycle_us = TICKS_TO_US(payload->channel_01);
	attitude_desired->roll = map_duty_cycle_to_attitude(duty_cycle_us);
	duty_cycle_us = TICKS_TO_US(payload->channel_02);
	attitude_desired->pitch = map_duty_cycle_to_attitude(duty_cycle_us);
	duty_cycle_us = TICKS_TO_US(payload->channel_03);
	attitude_desired->yaw = map_duty_cycle_to_attitude(duty_cycle_us);
	duty_cycle_us = TICKS_TO_US(payload->channel_04);
	*thrust = map_duty_cycle_to_attitude(duty_cycle_us);
	return 1;
}

void serialize_16bit(uint8_t offset, uint16_t bytes) {
	tx_buffer[offset] = bytes>>8;
	tx_buffer[offset++] = bytes & 0xFF;
}

void serialize_24bit(uint8_t offset, uint32_t bytes) {
	tx_buffer[offset] = (bytes>>16) & 0xFF;
	serialize_16bit(offset++, bytes & 0xFFFF);
}

void serialize_8bit(uint8_t offset, uint8_t byte) {
	tx_buffer[offset] = byte;
}

void crsf_drv_serialize_gps_frame(gps_frame* frame, uint8_t* _offset) {
	uint8_t offset = *_offset;
	serialize_32bit(offset, frame->latitude);
	offset += 4;
	serialize_32bit(offset, frame->longitude);
	offset += 4;
	serialize_16bit(offset, frame->groundspeed);
	offset += 2;
	serialize_16bit(offset, frame->heading);
	offset += 2;
	serialize_16bit(offset, frame->altitude);
	offset += 2;
	serialize_8bit(offset, frame->satellites);
	offset += 1;
	*_offset = offset;
}

void crsf_drv_serialize_frame(crsf_frame* frame, uint8_t offset) {
	
	tx_buffer[offset++] = frame->hdr.sync_byte;
	tx_buffer[offset++] = frame->hdr.len;
	tx_buffer[offset++] = frame->hdr.type;
	switch(frame->hdr.type) {
		case GPS: crsf_drv_serialize_gps_frame(frame->gps_data, &offset); break;
		case RC_Channels_Packed_Payload: crsf_drv_serialize_channels(frame->RC_Channels_Packed_data, &offset); break;
		//TODO: liste vervollst�ndigen
	}
	tx_buffer[offset++] = frame->crc;
}
 

uint8_t crsf_drv_send_frame(crsf_frame* frame) {
	if (frame->hdr.len > 64) return 0; //TODO: vll. hier noch OPCode returnen
	//TODO: noch einen check einf�gen ob der tx_buffer voll ist bevor du den frame da rein tust
	crsf_drv_serialize_frame(frame, tx_data_ptr);
	//TODO: CRC noch berechnen
	return uart_ifc.uart_write(tx_buffer + tx_data_ptr, frame->hdr.len); //TODO: entsprechende OPCodes returnen u.B: UART_BUSY etc
}

void crsf_drv_deserialize_gps_frame(gps_frame* gps_data, uint8_t* _offset)  {
	uint8_t offset = *_offset;
	uint32_t bit_32 = rx_buffer[offset++] << 3; //gps_data data pointer has same address as rx_buffer[offset]. in case system uses little endian
	//assigning rx_buffer values directly to gps_data member variables would reverse order of rx_buffer values and thus 
	//assign incorrect values to member variables. bit_32 as an intermediate illiviates this problem.
	bit_32 |= rx_buffer[offset++] << 2;
	bit_32 |= rx_buffer[offset++] << 1;
	bit_32 |= rx_buffer[offset++];
	gps_data->latitude = bit_32;
	bit_32 = rx_buffer[offset++] << 3;
	bit_32 |= rx_buffer[offset++] << 2;
	bit_32 |= rx_buffer[offset++] << 1;
	bit_32 |= rx_buffer[offset++];
	gps_data->longitude = bit_32;
	bit_32 = rx_buffer[offset++] << 1;
	bit_32 |= rx_buffer[offset++];
	gps_data->groundspeed = bit_32;
	bit_32 = rx_buffer[offset++] << 1;
	bit_32 |= rx_buffer[offset++];
	gps_data->altitude = bit_32;
	gps_data->satellites = rx_buffer[offset++];
	*_offset = offset;
}

void crsf_drv_deserialize_RC_Channels_Packed_Payload_frame(RC_Channels_Packed_Payload_frame* frame, uint8_t* _offset) {

	 	uint8_t offset = *_offset;
	    uint32_t bitBuffer = 0;
	    int bitCount = 0;
		uint16_t channels[16];

	    for (uint8_t ch = 0; ch < 16; ch++)
	    {
		    while (bitCount < 11)
		    {
			    bitBuffer |= ((uint32_t)rx_buffer[offset++]) << bitCount;
			    bitCount += 8;
		    }

		    channels[ch] = bitBuffer & 0x7FF; // 11 bit mask
		    bitBuffer >>= 11;
		    bitCount -= 11;
	    }
		frame->channel_01 = channels[0];
		frame->channel_02 = channels[1];
		frame->channel_03 = channels[2];
		frame->channel_04 = channels[3];
		frame->channel_05 = channels[4];
		frame->channel_06 = channels[5];
		frame->channel_07 = channels[6];
		frame->channel_08 = channels[7];
		frame->channel_09 = channels[8];
		frame->channel_10 = channels[9];
		frame->channel_11 = channels[10];
		frame->channel_12 = channels[11];
		frame->channel_13 = channels[12];
		frame->channel_14 = channels[13];
		frame->channel_15 = channels[14];
		frame->channel_16 = channels[15];
		*_offset = offset;
}

void crsf_drv_convert_channels_values_into_attitude(attitude* attitude_desired, RC_Channels_Packed_Payload_frame* payload) {
	/*TODO: hier wird angenommen, das die channels_1 == roll, channel_2 == pitch, channel_3 == thrust, channel_4 == yaw entsprechen, weil das die �bliche konfiguration ist, 
	ist allerdings nat�rlich auch eintellungssache, das vll. noch im code so einbauen, das die eine ge�nderte einstellung in der steuerung der drone tats�chlich umgesetzt wird*/
	uint16_t duty_cycle_us = TICKS_TO_US(payload->channel_01);
	attitude_desired->roll = map_duty_cycle_to_attitude(duty_cycle_us);
	duty_cycle_us = TICKS_TO_US(payload->channel_02);
	attitude_desired->pitch = map_duty_cycle_to_attitude(duty_cycle_us);
	duty_cycle_us = TICKS_TO_US(payload->channel_04);
	attitude_desired->yaw = map_duty_cycle_to_attitude(duty_cycle_us);
	//TODO: f�ge noch einen check ein ob die channel values innerhalb der g�ltigen werte liegen und lass, wenn das nicht der fall ist einen entsprechenden error code durch 
	// die funktion zur�ckgeben
}

float map_duty_cycle_to_thrust(uint16_t us) {
	int16_t _us = (int16_t) us;
	if (us>MAX_CHANNEL_VAL) _us = MAX_CHANNEL_VAL;
	if (us<MIN_CHANNEL_VAL) _us = MIN_CHANNEL_VAL;
	_us -= CENTER;
	return US_TO_THRUST(_us);
}

#ifdef FIXED_POINT
void crsf_drv_convert_channels_values_into_thrust(uint8_t* thrust, RC_Channels_Packed_Payload_frame* payload) {
#else
void crsf_drv_convert_channels_values_into_thrust(float* thrust, RC_Channels_Packed_Payload_frame* payload) {
#endif
	/*TODO: hier wird angenommen, das die channels_1 == roll, channel_2 == pitch, channel_3 == thrust, channel_4 == yaw entsprechen, weil das die �bliche konfiguration ist, 
	ist allerdings nat�rlich auch eintellungssache, das vll. noch im code so einbauen, das die eine ge�nderte einstellung in der steuerung der drone tats�chlich umgesetzt wird*/
	uint16_t duty_cycle_us = TICKS_TO_US(payload->channel_03);
	*thrust = map_duty_cycle_to_thrust(duty_cycle_us);
}

void crsf_drv_deserialize_Channels_Packed_Payload_raw(RC_Channels_Packed_Payload_frame* frame, volatile uint8_t* byte_stream) {

	uint32_t bitBuffer = 0;
	uint8_t bitCount = 0;
	uint8_t idx = 0;
	uint16_t channels[16];
	//TODO: effizienter w�re vermutlich ohne die schleife, wenn du die channel values manuell den channel_xx membern des structs zuwei�t weil dann nur ein kopiervorgang
	//herrscht.
	for (uint8_t ch = 0; ch < 16; ch++)
	{
		while (bitCount < 11)
		{
			bitBuffer |= ((uint32_t)byte_stream[idx++]) << bitCount;
			bitCount += 8;
		}

		channels[ch] = bitBuffer & 0x7FF; // 11 bit mask
		bitBuffer >>= 11;
		bitCount -= 11;
	}
	frame->channel_01 = channels[0];
	frame->channel_02 = channels[1];
	frame->channel_03 = channels[2];
	frame->channel_04 = channels[3];
	frame->channel_05 = channels[4];
	frame->channel_06 = channels[5];
	frame->channel_07 = channels[6];
	frame->channel_08 = channels[7];
	frame->channel_09 = channels[8];
	frame->channel_10 = channels[9];
	frame->channel_11 = channels[10];
	frame->channel_12 = channels[11];
	frame->channel_13 = channels[12];
	frame->channel_14 = channels[13];
	frame->channel_15 = channels[14];
	frame->channel_16 = channels[15];
}


void crsf_drv_serialize_channels(RC_Channels_Packed_Payload_frame *_channels, uint8_t* _offset)
{
	uint8_t offset = *_offset;
	if ((255-22) < offset) {
		memset(&tx_buffer[offset], 0, (255-offset)+1);
		memset(&tx_buffer[0], 0, (22-(255-offset))-1);
	} else {
		memset(&tx_buffer[offset], 0, 22);
	}
	
	uint16_t channels[] = {_channels->channel_01, _channels->channel_02, _channels->channel_03, _channels->channel_04, \
		_channels->channel_05, _channels->channel_06, _channels->channel_07, _channels->channel_07, _channels->channel_08,\
		_channels->channel_09, _channels->channel_10, _channels->channel_11, _channels->channel_12, _channels->channel_13, \
		_channels->channel_14, _channels->channel_15, _channels->channel_16};
		//{992, 992, 992, 992, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};
	uint16_t bitIndex = 0;
	uint8_t idx;
	
	for (uint8_t ch = 0; ch < 16; ++ch)
	{
		uint16_t value = channels[ch] & 0x07FF; // 11 Bit maskieren

		for (int bit = 0; bit < 11; bit++)
		{
			if (value & (1 << bit))
			{
				uint16_t byteIndex = bitIndex >> 3;
				uint8_t bitOffset = bitIndex & 0x07;
				idx = offset + byteIndex;
				tx_buffer[idx] |= (1 << bitOffset);
			}

			bitIndex++;
		}
	}
	*_offset += 22;
}

uint8_t crsf_drv_deserialize_frame(crsf_frame* frame, volatile uint8_t* _offset) {
	uint8_t offset = *_offset;
	frame->hdr.sync_byte = rx_buffer[offset++];
	frame->hdr.len = rx_buffer[offset++];
	frame->hdr.type = rx_buffer[offset++];
	switch(frame->hdr.type) {
		case GPS: crsf_drv_deserialize_gps_frame(frame->gps_data, &offset); break;
		//TODO: vervollst�ndige die liste
		case RC_Channels_Packed_Payload: crsf_drv_deserialize_RC_Channels_Packed_Payload_frame(frame->RC_Channels_Packed_data, &offset); break;
		
		default: return 0;
	}
	frame->crc = rx_buffer[offset++];
	*_offset = offset;
	return 1;
}

//TODO: berechne noch die CRC des frames bevor du ihn in den tx_buffer schreibst

typedef enum ERRORCODE {
	FRAME_NOT_FOUND,
	FRAME_HEAP_ALLOC_ERROR,
	FRAME_INCOMPLETE,
} ERRORCODE;

ERRORCODE crsf_drv_extract_frame(crsf_frame* frame) {
	
	bool frame_found = false;
	uint8_t frame_len, frame_type;
	while (*bytes_in_rx_buffer_ptr > MIN_FRAME_SIZE) {//TODO: aufpassen wenn die buffersize auch �ber 255 geht muss die i integer einen entsprechend gr��eren datentyp beinhalten

		switch (rx_buffer[RX_TAIL]) {//check sync byte
			case 0xC8: break; //Serial sync byte
			/*case Broadcast_address:; //Broadcast device address
			case Cloud:;
			case USB_Device:;
			case Bluetooth_Module_Wifi:;
			case Wifi_receiver:;
			case Video_Receiver:break;*/ //TODO: diese break an den letzten case, wenn das hier weiter vervollst�ndigt wird
			default: 
					--*bytes_in_rx_buffer_ptr;//TODO: diese dekrementierung wahrscheinlich atomar machen
					++RX_TAIL; continue; //TODO: dekrement von bytes_in_rx_buffer noch atomic machen
			//TODO: case abfragen vervollst�ndigen 
		}  //TODO: letztes case muss ein break haben, default ein continue
		frame_type = rx_buffer[(uint8_t)(RX_TAIL + 2)]; //TODO: das (uint8_t)(RX_TAIL + 2) machst damit du damit der wert nicht �ber 255 geht.
		//damit wird allerdings ein implementierungsdetail des UART als wissen hier vorrausgesetzt: die gr��e des UART-Buffers. d.h. uart und 
		//crsf driver sind nicht direkt entkoppelt. es w�re eleganter sowas durch (RX_TAIL + 2) % RX_BUFFER_SIZE zu l�sen, auch wenn das 
		//rechnerisch aufw�ndiger ist
		frame_len = rx_buffer[(uint8_t)(RX_TAIL + 1)]; //TODO: das gleiche wie in der zeile davor
		//TODO: hier noch den fall ber�cksichtigen, das ein frame eines typs auch eine variable l�nge haben kann
		if (frame_len == (type_to_length_map[frame_type]+2)) {//type and length have to fit 
			//now we should have the beginning of a frame
			frame_found = true;
			break;
		}
		//frame_found = true;
		--*bytes_in_rx_buffer_ptr;  //atomar machen??
		++RX_TAIL; //TODO: noch atomarer zugriff
	}
	
	if (!frame_found) return FRAME_NOT_FOUND;
	
	//if (crsf_drv_malloc(frame) == 0) return FRAME_HEAP_ALLOC_ERROR; //todo: wenn diese zeile auskommentiert ist irgendwann wieder inekommentieren
	
	if ((*bytes_in_rx_buffer_ptr) < (frame_len + 2)) return FRAME_INCOMPLETE;
	crsf_drv_deserialize_frame(frame, rx_tail);
	//TODO: eigentlich um diese codezeile das atomic makro, damit isr nicht zwischenfunkt: wie aber den atomic zugriff HW unabh�ngig machen?
	*bytes_in_rx_buffer_ptr -= (frame_len + 2); //Atomar machen
	return 5; //TODO:  noch richtigen opcode zur�ckgeben
}

/*void crsf_drv_read_buffer() {
	while(!time_to_read);
	while(USART_0_write_bytes(rx_buffer_1, 1000))
	while(tx_status);
	memset(rx_buffer_1, 0, 1000);
	time_to_read = 0;
	rx_buffer_head_1 = 0;
	UCSR0B |= (1<<RXCIE0);
	UCSR0B |= (1<<RXEN0);
}*/

//#define VERSION_1

#if defined(VERSION_1)
volatile uint8_t test_rx_channel_packet_readout(volatile uint8_t* buffer) {
	volatile uint8_t ptr;
	uint8_t i = 0;
	while (rx_buffer_space > 26) {
		if ((rx_buffer[rx_buffer_tail] == 0xC8) && (rx_buffer[(uint8_t)(rx_buffer_tail + 1)] == 24) && (rx_buffer[(uint8_t)(rx_buffer_tail + 2)] == RC_Channels_Packed_Payload))
		{
			for (uint8_t j = 0; j<26; ++j) {
				buffer[j] = rx_buffer[rx_buffer_tail++];
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE) //TODO: sicher das es auch das tut was chatgpt sagt?
				{
					--rx_buffer_space;
				}
			}
			return 1;
		}
		++rx_buffer_tail;
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) //TODO: sicher das es auch das tut was chatgpt sagt?
		{
			--rx_buffer_space;
		}
		++i;
	}
	return (volatile uint8_t)0;
}

#else

volatile uint8_t test_rx_channel_packet_readout(volatile uint8_t* buffer) {
	volatile uint8_t i = 0;
	if (rx_buffer_space<100) return 0;
	uint8_t rx_buffer_space_c = rx_buffer_space;
	while (rx_buffer_space_c) {
		buffer[i] = rx_buffer[rx_buffer_tail];
		++rx_buffer_tail;
		++i;
		--rx_buffer_space; //TODO: diese beiden folgenden statements atomar machen
		rx_buffer_space_c = rx_buffer_space;
		//ptr = &rx_buffer[RX_TAIL];
		return i;
	}
	return (volatile uint8_t)0;
}

#endif


volatile uint8_t test_rx_channel_packet_readout_runtime(volatile uint8_t* buffer) {
	uint8_t i = 0;
	//if (rx_buffer_space<100) return 0;
	for (uint8_t j = 0; j<110; ++j) {
		//buffer[i] = rx_buffer[rx_buffer_tail];
		rx_buffer_tail = (rx_buffer_tail + 1); //% BUFFER_0_SIZE;
		++i;
		//ATOMIC_BLOCK(ATOMIC_RESTORESTATE) //TODO: sicher das es auch das tut was chatgpt sagt?
		//{
		--rx_buffer_space;
		//}
		//ptr = &rx_buffer[RX_TAIL];
	}
	return i;
	//return (volatile uint8_t)0;
}