#include "ARCL_OSAL.h"
#include "stdio.h"
#include "gimbalControl.h"
#include "UdpDataPort.h"
#include "public_math.h"
#include <string>
#include <sys/time.h>
#include "dji_logger.h"

#include "dji_gimbal_manager.h"
#include "dji_fc_subscription.h"
#include "tta_camera_manager.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "ttalink.h"
#include "ttalink_rount.h"
#ifdef __cplusplus
}
#endif

#include <unistd.h>

#define HEARTBEAT_INTERVAL  1000
#define DEFAULT_CONTINUOUS_SHOOT_INTERVAL    1000
#define GIMBAL_MOVE_STEP    5
#define GIMBAL_ZOOM_FACTOR_STEP  1
#define LOCK_INTERVAL  100

#define LIMIT_LOCK_WIDTH_MIN  10
#define LIMIT_LOCK_IN  500
#define LIMIT_LOCK_KP_MAX 1800
#define LIMIT_LOCK_KI_MAX 500
#define LIMIT_LOCK_KD_MAX 500
#define LIMIT_LOCK_OUT_MAX 1800


int64_t gimbalSn=11223344;

void sendHeartBeat(DataPort* port, uint8_t dst_addr, uint8_t src_addr, int64_t sn, uint8_t type)
{
	if(port){
		uint8_t buffer[TTALINK_MAX_PACKET_LEN];
		ttalink_message_t msg;
		ttalink_heartbeat_t hb;
		int len=0;

		bzero(&hb, sizeof(hb));
		hb.device_type = type;
		hb.sn = sn;
		ttalink_heartbeat_encode(dst_addr, src_addr, &msg, &hb);
		len = ttalink_to_send_buffer(buffer, &msg);
		BufferItem* item = new BufferItem((char *)buffer, len);
		port->send(item);
	}
}
void sendGeneralCommand(DataPort* port, uint8_t dst_addr, uint8_t src_addr, GeneralCmd_t cmd_data, ACK_TYPE ack, int64_t sn, uint8_t type){
	if(port){
		uint8_t buffer[TTALINK_MAX_PACKET_LEN];
		ttalink_message_t msg;
		ttalink_general_command_t gca;
		int len=0;
		memset(&gca, 0, sizeof(gca));
		gca.sn = sn;
		gca.device_type = type;
        gca.cmd = cmd_data.cmd;
        gca.param1 = cmd_data.param1;
        gca.param2 = cmd_data.param2;
        gca.param3 = cmd_data.param3;
        gca.param4 = cmd_data.param4;
        gca.param5 = cmd_data.param5;
        gca.param6 = cmd_data.param6;
        gca.param7 = cmd_data.param7;
		ttalink_general_command_encode(dst_addr, src_addr, &msg, &gca);
		len = ttalink_to_send_buffer(buffer, &msg);
		BufferItem* ack_item = new BufferItem((char*)buffer, len);
		port->send(ack_item);
	}
}
void sendGeneralCommandAck(DataPort* port, uint8_t dst_addr, uint8_t src_addr, uint16_t cmd, ACK_TYPE ack, int64_t sn, uint8_t type){
	if(port){
		uint8_t buffer[TTALINK_MAX_PACKET_LEN];
		ttalink_message_t msg;
		ttalink_general_command_ack_t gca;
		int len=0;

		memset(&gca, 0, sizeof(gca));
		gca.sn = sn;
		gca.device_type = type;
		gca.cmd = cmd;
		gca.result = 0;
		ttalink_general_command_ack_encode(dst_addr, src_addr, &msg, &gca);
		len = ttalink_to_send_buffer(buffer, &msg);
		BufferItem* ack_item = new BufferItem((char*)buffer, len);
		port->send(ack_item);
	}
}

void SendGeneralStatus(DataPort* port, uint8_t dst_addr, uint8_t src_addr, std::string name, double value, int64_t sn, uint8_t type)
{
	if(port){
		uint8_t buffer[TTALINK_MAX_PACKET_LEN];
		ttalink_message_t msg;
		ttalink_general_status_t gs;
		int len=0;

		gs.device_type = type;
		gs.sn = sn;
		memset(gs.status_id, 0, sizeof(gs.status_id));
		memcpy(gs.status_id, name.c_str(), name.size());
		gs.status_value = value;
		ttalink_general_status_encode(dst_addr, src_addr, &msg, &gs);
		len = ttalink_to_send_buffer(buffer, &msg);
		BufferItem* item = new BufferItem((char *)buffer, len);
		port->send(item);
	}
}

int WriteFileDataAppended(const std::string& path, const char* buffer, int size){
	FILE* fp = fopen(path.c_str(), "a");
	if(fp){
		int w = fwrite(buffer, 1, size, fp);
		fclose(fp);
		return w;
	}else{
		return -1;
	}
}

std::string GetDateTimeStringShort() {
    struct timeval tv;
    struct tm *t;
	char line[32];

	gettimeofday(&tv, NULL);

    time_t rawtime = tv.tv_sec;
	int mills = tv.tv_usec / 1000;

	t = localtime(&rawtime);
	if (t!=NULL) {
		sprintf(line, "%04d-%02d-%02d_%02d:%02d:%02d#%03d", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec, mills);
		return line;
	}else {
		return "0000-00-00_00:00:00-000";
	}
}

std::string getPushUrl(int64_t productSn, int64_t cameraSn){

}

void sendRepackAddress(DataPort* port, int dstAddr, ttalink_message_t &msg){
	if(port){
        int len=0;
        uint8_t buffer[TTALINK_MAX_PACKET_LEN];
		ttalink_general_command_t gc;
        ttalink_general_command_decode(&msg, &gc);
		ttalink_general_command_encode(dstAddr, TTALINK_PTZ_ADDRESS, &msg, &gc);
		len = ttalink_to_send_buffer(buffer, &msg);
		BufferItem* item = new BufferItem((char *)buffer, len);
		port->send(item);
	}
}

E_DjiMountPosition g_mount_pos_payload = DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1;

void gimbalInit()
{
    T_DjiReturnCode returnCode = DjiGimbalManager_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init gimbal manager failed, error code: 0x%08X", returnCode);
    }

    returnCode = DjiGimbalManager_SetMode(g_mount_pos_payload, DJI_GIMBAL_MODE_YAW_FOLLOW);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Set gimbal mode failed, error code: 0x%08X", returnCode);
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic flight status failed, error code:0x%08llX", returnCode);
    }
}


int Dji_FlightControlGetValueOfGimbalAngles(T_DjiFcSubscriptionGimbalAngles &angles)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionGimbalAngles gimbleAngle;
    T_DjiDataTimestamp gimbalAngleTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
                                                      (uint8_t *) &gimbleAngle,
                                                      sizeof(T_DjiFcSubscriptionGimbalAngles),
                                                      &gimbalAngleTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic gimbal angles error, error code: 0x%08X", djiStat);
        return -1;
    }

    angles = gimbleAngle;

    return 1;
}

void gimbal_general_command_handle(ttalink_general_command_t &gc)
{
    T_DjiReturnCode returnCode;
    T_DjiGimbalManagerRotation rotation;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    switch(gc.cmd){
        case 103:
            DjiTta_CameraManagerStartShootSinglePhoto(g_mount_pos_payload);
            break;
        case 104:
            if(gc.param1 == 1){
                DjiTta_CameraManagerStartRecordVideo(g_mount_pos_payload);
            }else if(gc.param1 == 2){
                DjiTta_CameraManagerStopRecordVideo(g_mount_pos_payload);
            }
            break;
        case 105:


            break;
        case 106:
            if(gc.param1 == 1){
                DjiTta_CameraManagerStartContinuousZoom(g_mount_pos_payload
                    ,DJI_CAMERA_ZOOM_DIRECTION_IN, DJI_CAMERA_ZOOM_SPEED_NORMAL);
            }else if(gc.param1 == 2){
                DjiTta_CameraManagerStartContinuousZoom(g_mount_pos_payload
                    ,DJI_CAMERA_ZOOM_DIRECTION_OUT, DJI_CAMERA_ZOOM_SPEED_NORMAL);
            }
            else if(gc.param1 == 3)
            {
                DjiTta_CameraManagerStopContinuousZoom(g_mount_pos_payload);
            }
            break;
        case 109:
            {
                T_DjiFcSubscriptionGimbalAngles angles;

                if(Dji_FlightControlGetValueOfGimbalAngles(angles))
                {
                    rotation.pitch = angles.x;
                    rotation.roll = angles.y;
                    rotation.yaw = angles.z;
                    rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE;

                    USER_LOG_ERROR("current gimbal angle ================= (%.1f, %.1f, %.1f) ",
                                    rotation.pitch, rotation.roll,rotation.yaw);

                    rotation.pitch = gc.param1;
                    rotation.roll = gc.param2;
                    rotation.yaw = gc.param3;

                    rotation.time = 5;

                    returnCode = DjiGimbalManager_Rotate(g_mount_pos_payload, rotation);
                    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        USER_LOG_ERROR("Target gimbal pry = (%.1f, %.1f, %.1f) failed, error code: 0x%08X",
                                    rotation.pitch, rotation.roll,rotation.yaw,returnCode);
                    }
                    osalHandler->TaskSleepMs(1000);

                }
            }

            break;
        case 110:

            returnCode = DjiGimbalManager_Reset(g_mount_pos_payload);
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                USER_LOG_ERROR("Reset gimbal failed, error code: 0x%08X", returnCode);
            }
            osalHandler->TaskSleepMs(2000);

            break;
        case 111://point
            {
                float x = 0.0;
                float y = 0.0;

                x = (gc.param1 + 10000)*0.00005;
                y = (gc.param2*(-1) + 10000)*0.00005;

                USER_LOG_INFO("point input data x:%f, y:%f, out put x:%f, y:%f \r\n"
                ,gc.param1, gc.param2, x, y);

                // p->setTapZoomPointSyncSample(PAYLOAD_INDEX_0, 0, x, y);
            }
            break;
        case 112://point zoom
            {
                float x = 0.0;
                float y = 0.0;

                x = (gc.param1 + 10000)*0.00005;
                y = (gc.param2*(-1) + 10000)*0.00005;

                USER_LOG_INFO("zoom point input data x:%f, y:%f, out put x:%f, y:%f \r\n"
                ,gc.param1, gc.param2, x, y);

                // p->setTapZoomPointSyncSample(PAYLOAD_INDEX_0, 2, x, y);
            }
            break;
        case 113:
            if(1){
                // GimbalModule::Rotation rotation;
                // rotation.roll = 0;
                // rotation.pitch = -90;
                // rotation.yaw = GetFeedbackAddr()->nav_yaw;
                // rotation.rotationMode = 0;
                // rotation.time = 1;
                // g->rotateSyncSample(PAYLOAD_INDEX_0, rotation);
            }
            break;
        case 115:
            if(1){
                // GimbalModule::Rotation rotation;
                // rotation.roll = gc.param2;
                // rotation.pitch = gc.param3;
                // rotation.yaw = gc.param4;
                // rotation.rotationMode = gc.param1;
                // rotation.time = gc.param5;
                // g->rotateSyncSample(PAYLOAD_INDEX_0, rotation);
            }
            break;
        case 130:
        case 131:
        case 132:
        case 134:
        case 135:
            // printf("send ptz tracker\n");
            // sendRepackAddress(gimbalConnection, TTALINK_PTZ_TRACKER_ADDRESS, msg);
            break;
        case 200:
            break;
        default:
            break;
    }
}

//test-gimbal-flag

void *gimbalControl(void *)
{



}





