#include "ARCL_OSAL.h"
#include "gcs.h"
#include <stdio.h>
#include <unistd.h>
#include "dji_logger.h"

#include "ttalink.h"

#include "gimbalControl.h"

ttalink_message_t  msg_sv;





struct taskGcsTime_t taskGlobalTime;
struct taskGcsTime_t taskSVTime;
void *server_communication(void *)
{
	unsigned int heart_time = 0;
	static unsigned int sysTime = 0;
	msg_sv.dst_addr = TTALINK_SV_ADDRESS;

	while(1)
	{
		sysTime = ACRL_GetTimeMs();

		UpdateGcs();

		if((sysTime-taskGlobalTime.version_data_count)>=1301)
		{
			taskGlobalTime.version_data_count = sysTime;
			// hand_request_device_version();
		}

		SendDataToFCTask(sysTime,&msg_sv,&taskSVTime);

/*
		if(ACRL_GetTimeMs()>heart_time)
		{
			heart_time = ACRL_GetTimeMs()+500;
  			update_heart_beat(0,&msg);
			printf("send heart beat %d \r\n",heart_time);
		}
*/
		usleep(20);
	}
}



