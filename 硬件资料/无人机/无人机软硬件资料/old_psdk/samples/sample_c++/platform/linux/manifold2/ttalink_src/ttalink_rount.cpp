#include "ttalink_rount.h"

#define channel_max	3
ttalink_rount_list_t ttalink_rount_list[channel_max + 1];//foc���ռ�ö��ͨ��

ttalink_channel_t addr2chan(unsigned char addr)
{
	unsigned char index;
	for(index=1; index<=ttalink_rount_list[0].chan; index++)
	{
		if(ttalink_rount_list[index].addr == addr)
			return ttalink_rount_list[index].chan;
	}
	return TTALINK_CHANNEL_DEFAULT;
}

static void ttalink_rount_list_init(uint8_t local_addr)
{
	ttalink_rount_list[0].chan = TTALINK_CHANNEL_DEFAULT;
	ttalink_rount_list[0].addr = local_addr;
}

static void ttalink_rount_list_add(ttalink_channel_t chan, unsigned char addr)
{
	unsigned char channel;
	int temp;
	temp = (ttalink_rount_list[0].chan);
	if(temp++ < channel_max)		
	{
		ttalink_rount_list[0].chan = (ttalink_channel_t)temp;
		channel = ttalink_rount_list[0].chan;
		ttalink_rount_list[channel].chan = chan;
		ttalink_rount_list[channel].addr = addr;
	}
}


void ttalink_rount_system_init(void)
{
	unsigned char index;
	for(index=0; index<TTALINK_COMM_NUM_BUFFERS; index++)
	{
		ttalink_reset_channel_status(index);
	}
	ttalink_rount_list_init(TTALINK_SH_ADDRESS);
	
	ttalink_rount_list_add(TTALINK_CHANNEL_FC_SH,TTALINK_SV_ADDRESS);
	ttalink_rount_list_add(TTALINK_CHANNEL_FC_SH,TTALINK_EMBE_ADDRESS);
	ttalink_rount_list_add(TTALINK_CHANNEL_FC_SH,TTALINK_FC_ADDRESS);

//	ttalink_rount_list_add(TTALINK_CHANNEL_FC_EMBE,TTALINK_BASE_RTK_ADDRESS);
	
	
//	ttalink_rount_list_add(TTALINK_CHANNEL_RADAR_DEBUG,TTALINK_LB_RADAR_DEBUG_ADDRESS); //�����״�����
//  for(unsigned char i=TTALINK_GIMBAL_ADDRESS;i<TTALINK_BASE_RTK_ADDRESS;i++) //�����ַ��ע�ᵽTC
//    ttalink_rount_list_add(TTALINK_CHANNEL_FC_SH,i);
	
}

int is_own_addr(uint8_t addr) 
{
		if(ttalink_rount_list[0].addr == addr)
			return 1;
		else
			return 0;
}
