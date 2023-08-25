#ifndef __XBEE_H
#define __XBEE_H

#include "stm32f10x.h"
#define true 1
#define false 0
	
#define NP 73

#define GROUND_STATION_MAC 	((uint64_t)0x0013A2004164909F)
#define BROADCAST_MAC				((uint64_t)0x000000000000FFFF)

#define MAX_FRAME_DATA_SIZE NP + 16
// api is always the third byte in packet
#define API_ID_INDEX 3
#define NO_ERROR 0
#define CHECKSUM_FAILURE 1
#define PACKET_EXCEEDS_BYTE_ARRAY_LENGTH 2
#define UNEXPECTED_START_BYTE 3

#define START_BYTE 0x7E
#define TX_REQUEST 0x10
#define DEFAULT_FRAME_ID 0x01
#define NO_RESPONSE_FRAME_ID 0x00
#define BROADCAST_RADIUS_MAX_HOPS 0x00
#define BROADCAST_ADDRESS16 0xFFFE
#define ACK_OPTION 0
#define TX_API_LENGTH 0x0E

void XBee_ReadPacket(uint8_t *frameDataPtr, uint8_t *flag);
void XBee_PreparePacket(uint8_t *TxBuffer, uint8_t *RFdata, uint16_t length);
#endif
