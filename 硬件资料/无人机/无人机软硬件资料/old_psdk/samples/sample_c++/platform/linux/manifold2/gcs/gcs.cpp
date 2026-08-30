#include "gcs.h"
#include "gcs_interface.h"
#include "mmap_normal.h"
#include "ARCL_OSAL.h"

#include "ttalink.h"
#include "gimbalControl.h"
#include "ttalink/common/ttalink_rosuav_imu_raw.h"

gcs_rc_input_t gcs_rc;

//osdk

///////////////////////////////////////////////

//////////////////////////////////////////////

unsigned char (*GcsSend)(unsigned char *send_data, unsigned int send_num);
unsigned int (*GcsReceive)(unsigned char *receive_data, unsigned int max_len);


ttalink_gps_sensors_data_t sensor_gps;
ttalink_rtk_sensors_data_t sensor_rtk;

int64_t GetCpuId()
{
    return 1234567890123llu;
}

struct gcs_location_param_t gcs_location_param;
ttalink_heartbeat_t heart_ack;
void hand_heart_beat_ack(ttalink_message_t *msg) {
	ttalink_heartbeat_decode(msg,&heart_ack);

	if(msg->src_addr == TTALINK_SV_ADDRESS)
	{
//		printf("receive heart beat from server \r\n");
		gcs_location_param.serverUpTime=0;
		gcs_location_param.serverLostFlag = 0;
		if(gcs_location_param.server_first_hb)
		{
			gcs_location_param.server_first_hb = 0;
			ttalink_general_request_version_t module;
			ttalink_general_request_version_send_struct(TTALINK_EMBE_ADDRESS, TTALINK_SH_ADDRESS, addr2chan(TTALINK_EMBE_ADDRESS),&module);
		}
	}
	else if(msg->src_addr == TTALINK_AGRICULTURE_APP_ADDRESS)
	{
		gcs_location_param.appUpTime=0;
		gcs_location_param.appLostFlag = 0;
		if(gcs_location_param.app_first_hb)
		{
			gcs_location_param.app_first_hb = 0;
//			AB_clear_point(0,1);
		}
	}
}

struct gcs_location_param_t *GetLocationParam(void)
{
	return &gcs_location_param;
}

uint8_t get_lost_heart_flag(void) {

	return gcs_location_param.serverLostFlag;
}

void handle_fc_message(ttalink_message_t *msg) {
	printf("handle fc message id is:%d \r\n", msg->msgid);
	switch(msg->msgid) {
		case TTALINK_MSG_ID_GENERAL_PARAM_SET:
		break;
		case TTALINK_MSG_ID_GENERAL_PARAM_VALUE:
		break;
		case TTALINK_MSG_ID_GENERAL_STATUS:
		break;
		case TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT:
		    hand_flight_data(msg);
		break;
		case TTALINK_MSG_ID_FUNCTION_MODE:
			hand_function_mode(msg);
			break;
		case TTALINK_MSG_ID_HEARTBEAT:
			hand_heart_beat_ack(msg);
			break;
        case TTALINK_MSG_ID_RC_INPUT:
		break;

		case TTALINK_MSG_ID_GENERAL_COMMAND:
			ttalink_general_command_t gc;
			ttalink_general_command_decode(msg, &gc);

			if(msg->dst_addr == TTALINK_PTZ_ADDRESS)
			{
				gimbal_general_command_handle(gc);
			}
		break;

		case TTALINK_MSG_ID_ROSUAV_IMU_RAW: {
            ttalink_rosuav_imu_raw_t imu_raw;
            ttalink_rosuav_imu_raw_decode(msg, &imu_raw);
            // 调试输出
            // printf("[GCS] ROSUAV_IMU_RAW acc=[%.6f %.6f %.6f] gyro=[%.6f %.6f %.6f]\n",
            //        imu_raw.acc[0], imu_raw.acc[1], imu_raw.acc[2],
            //        imu_raw.gyro[0], imu_raw.gyro[1], imu_raw.gyro[2]);
            // TODO: 传递给 uavData 接口或者发布到 ROS /imu/data_raw
            break;
        }

		default:
			break;
	}
}

unsigned char GcsNULLSend(unsigned char *send_data, unsigned int send_num)
{
	return 1;
}
unsigned int GcsNULLReceive(unsigned char *receive_data, unsigned int max_len)
{
	return 0;
}

void GcsInit(void) {

	printf("[GCS] *** patched GcsInit executed ***\n");

	GcsSend = GcsInterfaceSend;
	GcsReceive 	= GcsInterfaceReceive;

	ttalink_rount_system_init();

}

void gcs_log_data_free(void *p) {

	ACRL_Free(p);
}

uint8_t get_gcs_rc_input_copy() {

	return gcs_rc.copy_pub;
}

void gcs_rc_pub_reset() {
	gcs_rc.copy_pub = 0;
}

void hand_ttalink_rc_input(ttalink_message_t *msg) {

	ttalink_rc_input_t rc;

	ttalink_rc_input_decode(msg,&rc);
	memcpy(gcs_rc.rc_input,rc.rc_input,40);
	gcs_rc.copy_pub = 1;
}

void get_gcs_rc_input(signed short *rc)
{
	uint16_t rc_data[20];
	uint8_t i;
	gcs_rc.copy_pub = 0;
	memcpy(rc_data,gcs_rc.rc_input,40);

	for (i = 0; i < 8; i++)
	{
		if(i == 2)
		{
			if(rc_data[i] >= 1950)
			{
				rc[i] = 512;
			}
			else if(rc_data[i] <= 550)
			{
				rc[i] = -512;
			}
			else
			{
				rc[i] = (512*(rc_data[i]-1500))/450;
			}
		}
		else
		{
			if(rc_data[i] >= 2000)
			{
				rc[i] = 512;
			}
			else if(rc_data[i] <= 500)
			{
				rc[i] = -512;
			}
			else
			{
				rc[i] = (512*(rc_data[i]-1500))/500;
			}
		}
	}
}

void send_ack_message() {

	//comAckQueue
}

void hand_general_request_module_version(ttalink_message_t * msg)
{
	char* versionOut = NULL;
	ttalink_general_request_version_t version;
	ttalink_general_request_version_decode(msg,&version);
	ttalink_general_request_version_ack_t module;
	unsigned char tempHead[64];
	unsigned char tempHeadLength = 0;

	memset(tempHead, 0, 64);
	tempHeadLength = sprintf((char *)(tempHead),"FC_M300_");

	tempHeadLength += sprintf((char *)(tempHead + tempHeadLength),"OSDK_NX_");

	tempHeadLength += sprintf((char *)(tempHead + tempHeadLength),"V%0.2f_",DRIVER_VER);
	tempHeadLength += sprintf((char *)(tempHead + tempHeadLength),"%d.%d.%d.%d_Beta",SOFT_A,SOFT_B,SOFT_C,SOFT_SVN);

    std::string vsersionString = "";//m_mmap_normal.mmap_read("version");
	versionOut = (char*)vsersionString.data();

	if(version.device_type == TTA_DEVICE_TYPE_GENERIC||(version.device_type == TTA_DEVICE_TYPE_FC))
	{
		memset(module.version,0,64);
		module.device_type = TTA_DEVICE_TYPE_FC;
		module.sn = GetCpuId();
		module.ack = 0;
		memcpy(module.version,versionOut,vsersionString.size());

		printf("version--------------->>>>:%s, size----->>>>%zu", module.version,vsersionString.size());
		ttalink_general_request_version_ack_send_struct(msg->src_addr, TTALINK_SH_ADDRESS, addr2chan(msg->src_addr),&module);
	}
}

void handle_uart_channel_message(ttalink_message_t *msg) {

	handle_fc_message(msg);

	return;
}


void UpdateGcs(void) {
	unsigned char buf[GCS_COMM_MAX];
	int gcs_size = 0;
	int index = 0;

	ttalink_message_t uart_msg;
	ttalink_status_t status;

	memset(buf,0,GCS_COMM_MAX);

	gcs_size = GcsReceive(buf,GCS_COMM_MAX);

	if(gcs_size > 0)
	{
		// printf("[GCS] receive data size is %d\n", gcs_size);
		printf("receive data size is --------------------------->%d \r\n", gcs_size);
		for(index = 0; index < gcs_size; index++)
		{
			if(ttalink_parse_char(TTALINK_CHANNEL_FC_SH, buf[index], &uart_msg, &status) == TTALINK_FRAMING_OK)
			{
				// printf("[GCS] ttalink parse OK, msgid=%d\n", uart_msg.msgid);
				handle_uart_channel_message(&uart_msg);
			}
		}
	}
}

