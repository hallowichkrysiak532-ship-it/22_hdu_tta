#include "ttalink_transmit.h"
#include "ttalink_rount.h"
#include "gcs.h"
#include "ARCL_OSAL.h"

//#if(DRIVER_VER_EXTERN < 30)	
//#include "commu_spi.h"
//#endif

extern unsigned char (*GcsSend)(unsigned char *send_data, unsigned int send_num);

void * gcsUartMutex;

//unsigned int spi_send_len = 0;
//unsigned char spi_send_mid_buf[530];

//unsigned int sync_send_len = 0;
//unsigned char sync_send_mid_buf[530];
//unsigned char uart_send_mid_buf[530];

void ttalink_start_send(ttalink_channel_t chan, uint16_t len)
{
	if(chan == TTALINK_CHANNEL_FC_SH)
	{
		ACRL_TakeSemaphoreBinary(gcsUartMutex,0xffffffff);
	}
}
void ttalink_end_send(ttalink_channel_t chan, uint16_t len)
{
	if(chan == TTALINK_CHANNEL_FC_SH)
	{
		ACRL_GiveSemaphoreBinary(gcsUartMutex);
	}
}

void ttalink_send_bytes(ttalink_channel_t chan, const char *buf, uint16_t len)
{
	if(chan == TTALINK_CHANNEL_FC_SH)
	{
		while(GcsSend((unsigned char *)buf,len) == 0);
	}
}
