/**
  * Xbee data frame struct
  * 
  * Frame fields			Byte		Description
	*
  * Start delimiter		1				0x7E
  * Length						2 - 3		Most Significant Byte, Least Significant Byte
  * Frame data				4 - n		API-specific structure
  * Checksum					n + 1		1 byte
  * 
  * |---------------|---------------|-----------------------------------------------|-----------|
  * |								|								|					API-specific structure								|						|
  * |---------------|---------------|-------------------|---------------------------|-----------|
  * |Start delimiter|Length					|Frame type					|			Data									|Checksum		|
  * |---------------|-------|-------|-------------------|---|---|---|---|---|---|---|-----------|
  * |1							|2			|3			|4									|5	|6	|7	|8	|9	|...|n	|n+1        |
	* |---------------|-------|-------|-------------------|---|---|---|---|---|---|---|-----------|
	* |0x7E						|MSB		|LSB		|API frame type 		|Data												|Single byte|
	* |---------------|-------|-------|-------------------|---------------------------|-----------|
	*
	*	For broadcast transmissions, set the 64-bit destination address to 0x000000000000FFFF
	*
	*	For unicast transmissions, set the 64 bit address field to the address of the desired destination node.
	*	Set the reserved field to 0xFFFE.
  */
#include "xbee.h"
#include <string.h>

uint8_t _checksum;
uint8_t Complete;
uint8_t ErrorCode;
uint8_t Pos;

uint8_t read()
{
	return USART_ReceiveData(USART2);
}

void XBee_ReadPacket(uint8_t *RxBuffer, uint8_t *flag)
{
	// reset previous response
	if (Complete == true || ErrorCode > 0)
	{
		// discard previous packet and start over
		Complete = false;
		Pos = 0;
		_checksum = 0;
		ErrorCode = NO_ERROR;
		memset(RxBuffer, 0, 100);
	}
	
	uint8_t b = read();
	
	if (Pos > 0 && b == START_BYTE)
	{
		// new packet start before previous packeted completed -- discard previous packet and start over
		ErrorCode = UNEXPECTED_START_BYTE;
		return;
	}

	// checksum includes all bytes starting with api id
	if (Pos >= API_ID_INDEX)
	{
		_checksum+= b;
	}

	switch(Pos)
	{
		case 0:
			if (b == START_BYTE)
			{
				RxBuffer[Pos++] = b;
			}
		break;
			
		case 1:
			// length msb
			RxBuffer[Pos++] = b;
		break;

		case 2:
			// length lsb
			RxBuffer[Pos++] = b;
		break;

		case 3:
			RxBuffer[Pos++] = b;
		break;

		default:

			if (Pos > MAX_FRAME_DATA_SIZE)
			{
				// exceed max size.  should never occur
				ErrorCode = PACKET_EXCEEDS_BYTE_ARRAY_LENGTH;
				return;
			}

			// check if we're at the end of the packet
			// packet length does not include start, length, or checksum bytes, so add 3
			if (Pos == ((RxBuffer[1] << 8 & 0xFF) | (RxBuffer[2] & 0xFF)) + 3)
			{
				// verify checksum
				if (_checksum == 0xFF)
				{
					RxBuffer[Pos] = b;
					Complete = true;
					*flag = true;
					ErrorCode = NO_ERROR;
				} 
				else
				{
					ErrorCode = CHECKSUM_FAILURE;
				}
				// reset state vars
				Pos = 0;
				
				return;
			}
			else
			{
				// add to packet array, starting with the fourth byte of the apiFrame
				RxBuffer[Pos++] = b;
			}
	}
}

void XBee_PreparePacket(uint8_t *TxBuffer, uint8_t *RFdata, uint16_t length) {
	
	uint8_t checksum = 0;
	
	length += TX_API_LENGTH ; 											//Add (Offset 3-16)
							
	*TxBuffer++ = START_BYTE;												//Start Delimiter (Offset 0)
	*TxBuffer++ = length >> 8 & 0xFF;								//Length MSB (Offset 1)
	*TxBuffer++ = length & 0xFF;										//Length LSB (Offset 2)
	*TxBuffer++ = TX_REQUEST;    										//Frame Type (Offset 3) (Transmit Request frame - 0x10)
	*TxBuffer++ = NO_RESPONSE_FRAME_ID;							//Frame ID	 (Offset 4)	
	
	for(int8_t i = 7; i >= 0; i--)
	*TxBuffer++ = GROUND_STATION_MAC >> 8*i & 0xFF;	//64-bit destination address (Offset 5-12)
	
	*TxBuffer++ = BROADCAST_ADDRESS16 >> 8 & 0XFF;	//16-bit destination network address MSB (Offset 13)
  *TxBuffer++ = BROADCAST_ADDRESS16 & 0xFF;				//16-bit destination network address LSB (Offset 14)
	
  *TxBuffer++ = BROADCAST_RADIUS_MAX_HOPS;				//Broadcast Radius	(Offset 15)
  *TxBuffer++ = ACK_OPTION;												//Transmit Options (Offset 16)
	
	for(uint16_t i = 0; i < (length - 0x0E); i++)
	*TxBuffer++ = *RFdata++;												//RF data	(Offset 17-n)
				
	for(uint16_t i = 0; i <length; i++)
	checksum += *--TxBuffer;
	
	*(TxBuffer + length) = 0xFF - checksum;					//Checksum (Offset n+1)
}

