#include "string.h"
#include "ttalink_analysis.h"
#include "ttalink.h"

//#if TTALINK_CRC_EXTRA
//	static const uint8_t ttalink_extra_crcs[256] = TTALINK_MESSAGE_CRCS;
//#endif

unsigned char ttalink_parse_buf(unsigned char * buf_addr, ttalink_message_t* r_message, unsigned int *remain_length)
{
	uint16_t checksum;
	if(*(uint16_t *)buf_addr == 0xAB55)
	{
		memcpy(&r_message->magic, buf_addr, TTALINK_NUM_HEADER_BYTES);
		memcpy(&r_message->payload64, buf_addr + TTALINK_NUM_HEADER_BYTES, r_message->len);
		memcpy(&r_message->checksum, buf_addr + TTALINK_NUM_HEADER_BYTES + r_message->len, TTALINK_NUM_CHECKSUM_BYTES);
		if(*remain_length >= TTALINK_NUM_HEADER_BYTES + r_message->len + TTALINK_NUM_CHECKSUM_BYTES)
		{
			if(r_message->checksum == 0)
			{
				*remain_length -= TTALINK_NUM_HEADER_BYTES + r_message->len + TTALINK_NUM_CHECKSUM_BYTES;
				return 0;
			}
			else
				return 1;
		}
		else
			return 2;
	}
	else if(*(uint16_t *)buf_addr == 0xAA55)
	{
		memcpy(&r_message->magic, buf_addr, TTALINK_NUM_HEADER_BYTES);
		memcpy(&r_message->payload64, buf_addr + TTALINK_NUM_HEADER_BYTES, r_message->len);
		memcpy(&r_message->checksum, buf_addr + TTALINK_NUM_HEADER_BYTES + r_message->len, TTALINK_NUM_CHECKSUM_BYTES);
		if(*remain_length >= TTALINK_NUM_HEADER_BYTES + r_message->len + TTALINK_NUM_CHECKSUM_BYTES)
		{
			checksum = crc_calculate((const uint8_t*)&r_message->len, TTALINK_CORE_HEADER_LEN);
			crc_accumulate_buffer(&checksum, (const char*)r_message->payload64, r_message->len);
//#if TTALINK_CRC_EXTRA
//			crc_accumulate(ttalink_extra_crcs[r_message->msgid], &checksum);
//#endif
		if(checksum == r_message->checksum)
			{
				*remain_length -= TTALINK_NUM_HEADER_BYTES + r_message->len + TTALINK_NUM_CHECKSUM_BYTES;
				return 0;
			}
			else
				return 1;
		}
		else
			return 2;
	}
	else
		return 3;
}

