#include "gcs_interface.h"
#include "ARCL_OSAL.h"
#include "gcs.h"

void*  EmbeToFcQueue;
void* FcToEmbeQueue;
extern void* gcsUartMutex;



struct gcs_rw_interface_t
{
	unsigned char buf[GCS_COMM_MAX];
	unsigned int size;
};


void GcsInterfaceInit(void)
{
	ACRL_CreateQueue(&EmbeToFcQueue, 128,sizeof(struct gcs_rw_interface_t));
	ACRL_CreateQueue(&FcToEmbeQueue, 128,sizeof(struct gcs_rw_interface_t));
	GcsInit();
	//SetTaskTakeOffAltit(TAKE_OFF_ALTIT_NOR);
}


unsigned char GcsInterfaceSend(unsigned char *send_data, unsigned int send_num)
{
	unsigned int remain_size = send_num;
	struct gcs_rw_interface_t message_send;
	do
	{
		if(remain_size > GCS_COMM_MAX)
		{
			message_send.size = GCS_COMM_MAX;
			memcpy(message_send.buf , &send_data[send_num-remain_size], GCS_COMM_MAX);
			remain_size -= GCS_COMM_MAX;
			ACRL_QueueSend(FcToEmbeQueue, &message_send, 0);
		}
		else
		{
			message_send.size = remain_size;
			memcpy(message_send.buf , &send_data[send_num-remain_size], remain_size);
			remain_size = 0;
			ACRL_QueueSend(FcToEmbeQueue, &message_send, 0);
		}
	}while(remain_size > 0);
	return 1;
}






unsigned int GcsInterfaceReceive(unsigned char *receive_data, unsigned int max_len)
{
	struct gcs_rw_interface_t message_receive;
	if(ACRL_QueueReceive(EmbeToFcQueue,&message_receive, 0)==OSAL_TURE)
	{
		if(max_len < message_receive.size)
		{
			memcpy(receive_data, message_receive.buf, max_len);
			return max_len;
		}
		else
		{
			memcpy(receive_data, message_receive.buf, message_receive.size);
			return message_receive.size;
		}
	}
	else
		return 0;
}



unsigned char EmbeInterfaceSend(unsigned char *send_data, unsigned int send_num)
{
	unsigned int remain_size = send_num;
	struct gcs_rw_interface_t message_send;
	do
	{
		if(remain_size > GCS_COMM_MAX)
		{
			message_send.size = GCS_COMM_MAX;
			memcpy(message_send.buf , &send_data[send_num-remain_size], GCS_COMM_MAX);
			remain_size -= GCS_COMM_MAX;
			ACRL_QueueSend(EmbeToFcQueue, &message_send, 0);
		}
		else
		{
			message_send.size = remain_size;
			memcpy(message_send.buf , &send_data[send_num-remain_size], remain_size);
			remain_size = 0;
			ACRL_QueueSend(EmbeToFcQueue, &message_send, 0);
		}
	}while(remain_size > 0);
	return 1;
}





unsigned int EmbeInterfaceReceive(unsigned char *receive_data, unsigned int max_len)
{
	struct gcs_rw_interface_t message_receive;
	if(ACRL_QueueReceive(FcToEmbeQueue,&message_receive, 0)==OSAL_TURE)
	{
		if(max_len < message_receive.size)
		{
			memcpy(receive_data, message_receive.buf, max_len);
			return max_len;
		}
		else
		{
			memcpy(receive_data, message_receive.buf, message_receive.size);
			return message_receive.size;
		}
	}
	else
		return 0;

}











