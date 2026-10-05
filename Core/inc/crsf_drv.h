/*
 * crs_drv.h
 *
 * Created: 26/02/2026 17:05:38
 *  Author: slin9
 */ 


#ifndef CRS_DRV_H_
#define CRS_DRV_H_

#include <stdint.h>
#include "../inc/sensorfusion.h"
#include "sys_settings.h"

typedef void* init_data; //TODO: schau das du des hier noch in einen struct mit module_data und sys_init member umwandelst, wie in der microchip
//andwendung, das der void pointer hier nicht direkt als funktionsargument steht

//TODO: liste der frame structs vervollst�ndigen, siehe https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md#0x02-gps
typedef struct crsf_frame_header {
	uint8_t sync_byte;
	uint8_t len;
	uint8_t type;
} crsf_frame_header;

typedef struct gps_frame{
	int32_t latitude;       // degree / 10`000`000
	int32_t longitude;      // degree / 10`000`000
	uint16_t groundspeed;   // km/h / 100
	uint16_t heading;       // degree / 100
	uint16_t altitude;      // meter - 1000m offset
	uint8_t satellites;     // # of sats in view
} gps_frame;

typedef struct gps_time_frame{
	int16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint16_t millisecond;
} gps_time_frame;

typedef struct RC_Channels_Packed_Payload_frame
{
	uint16_t channel_01: 11;
	uint16_t channel_02: 11;
	uint16_t channel_03: 11;
	uint16_t channel_04: 11;
	uint16_t channel_05: 11;
	uint16_t channel_06: 11;
	uint16_t channel_07: 11;
	uint16_t channel_08: 11;
	uint16_t channel_09: 11;
	uint16_t channel_10: 11;
	uint16_t channel_11: 11;
	uint16_t channel_12: 11;
	uint16_t channel_13: 11;
	uint16_t channel_14: 11;
	uint16_t channel_15: 11;
	uint16_t channel_16: 11;
} RC_Channels_Packed_Payload_frame;

enum frame_types {
	GPS = 0x02,
	GPS_Time,
	GPS_Extended = 0x06,
	Variometer_Sensor,
	Battery_Sensor,
	Barometric_Altidude_Vertical_Speed,
	Airspeed = 0x0A,
	Heartbeat,
	RPM,
	TEMP,
	Voltages,
	Discontinued,
	VTX_Telemetry,
	Barometer,
	Magnetometer,
	Accel_Gyro,
	Link_Statistics,
	RC_Channels_Packed_Payload = 0x16,
	Subset_RC_Channels_Packed,
	RC_Channels_Packed_11_bits,
	//TODO: enum liste vervollst�ndigen: https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md#0x16-rc-channels-packed-payload
};


typedef struct crsf_frame {
	crsf_frame_header hdr;
	union {
		gps_frame* gps_data;
		gps_time_frame* gps_time_data;
		RC_Channels_Packed_Payload_frame* RC_Channels_Packed_data;
		//TODO: das hier noch vervollst�ndigen
	};
	uint8_t crc;
} crsf_frame;





enum Device_Addresses {
	Broadcast_address = 0x0,
	Cloud = 0x0E,
	USB_Device = 0x10,
	Bluetooth_Module_Wifi = 0x12,
	Wifi_receiver,
	Video_Receiver,
	//TODO: hier noch irgendwie den Dynamic Address space schlau abbilden
};

uint8_t crsf_drv_malloc(crsf_frame *);

void crsf_driver_init(init_data data); 

void crsf_drv_free(crsf_frame *);

void crsf_drv_serialize_gps_frame(gps_frame*, uint8_t*);

uint8_t crsf_drv_extract_frame(crsf_frame*);

volatile uint8_t test_rx_channel_packet_readout(volatile uint8_t*);

void crsf_drv_serialize_channels(RC_Channels_Packed_Payload_frame *_channels, uint8_t payload[22]);

void crsf_drv_serialize_frame(crsf_frame*, uint8_t);

void USART0_read_buffer();

void crsf_drv_read_buffer();

volatile uint8_t test_rx_channel_packet_readout_runtime(volatile uint8_t* buffer);

void crsf_drv_deserialize_Channels_Packed_Payload_raw(RC_Channels_Packed_Payload_frame*, volatile uint8_t*);

void crsf_drv_deserialize_RC_Channels_Packed_Payload_frame(RC_Channels_Packed_Payload_frame*, uint8_t*);

void crsf_drv_convert_channels_values_into_attitude(attitude*, RC_Channels_Packed_Payload_frame*);

#ifdef FIXED_POINT
void crsf_drv_convert_channels_values_into_thrust(uint8_t* thrust, RC_Channels_Packed_Payload_frame* payload);
#else
void crsf_drv_convert_channels_values_into_thrust(float* thrust, RC_Channels_Packed_Payload_frame* payload);
#endif

#endif /* CRS_DRV_H_ */