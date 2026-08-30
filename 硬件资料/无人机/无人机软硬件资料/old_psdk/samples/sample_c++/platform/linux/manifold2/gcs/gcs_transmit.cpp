 /***********************����***********************/
#include "gcs.h"
#include "gcs_transmit.h"
#include "gcs_receive.h"
#include "ttalink_rount.h"
#include "ttalink.h"
#include "ttalink/common/ttalink_rosuav_imu_raw.h"
#include "dji_logger.h"

#include "sensor.h"

#include "tta_fc_subscription.h"
#include "tta_flight_control.h"

#include "public_math.h"

static unsigned int rosuav_imu_raw_count = 0;
SemaphoreHandle_t loopInputMutex;

//extern int (*GcsSend)(unsigned char *send_data, unsigned int send_num);
extern struct gcs_heart_t gcs_heart;
/***********************����***********************/
ttalink_heartbeat_t heart_beat;
/**********************************************/

void update_heart_beat(uint8_t state,unsigned char index) //, ttalink_message_t *msg
{
	heart_beat.simple_time++;
	heart_beat.device_type = TTA_DEVICE_TYPE_SH;
	heart_beat.sn = 1400220992315llu;
	heart_beat.upload_state = 0;

	// ttalink_heartbeat_send_struct(TTALINK_FC_ADDRESS, TTALINK_SH_ADDRESS, addr2chan(TTALINK_FC_ADDRESS),&heart_beat);
	// ttalink_heartbeat_send_struct(TTALINK_SV_ADDRESS, TTALINK_SH_ADDRESS, addr2chan(TTALINK_FC_ADDRESS),&heart_beat);
}

void update_rosuav_baro_raw_data(sensor_t *sensor)
{
	ttalink_rosuav_baro_raw_t sd_msg;
	sd_msg.pressure[0] = 0;
	sd_msg.attitude[0] = sensor->altitudeBarometer;

	ttalink_rosuav_baro_raw_send_struct(TTALINK_FC_ADDRESS, TTALINK_SH_ADDRESS, addr2chan(TTALINK_FC_ADDRESS),&sd_msg);
}

void update_rosuav_mag_raw_data(sensor_t *sensor)
{
	static unsigned int count=0;

	ttalink_mag_sensors_data_t sd_msg;
	memset(&sd_msg,0,sizeof(ttalink_mag_sensors_data_t));
	count++;
	sd_msg.sensor_index = 11; //通过 该值判断 原始值与校准后的值，11或者12 原始值

	sd_msg.update_time = ACRL_GetTimeMs();
	sd_msg.mag_raw[0] =  sensor->compass.x;
	sd_msg.mag_raw[1] =  sensor->compass.y;
	sd_msg.mag_raw[2] =  sensor->compass.z;

	ttalink_mag_sensors_data_send_struct(TTALINK_FC_ADDRESS, TTALINK_SH_ADDRESS, addr2chan(TTALINK_FC_ADDRESS),&sd_msg);
}

void update_rosuav_imu_raw_data(sensor_t *sensor)
{
	// printf("[FC] update_rosuav_imu_raw_data called\n");

    ttalink_rosuav_imu_raw_t sd_msg;
    // 填充 raw IMU，使用 accRaw 和 fuse 之前的 gyro（如果有原始 gyro 字段优先用它）
    sd_msg.acc[0] = sensor->accRaw.x;
    sd_msg.acc[1] = sensor->accRaw.y;
    sd_msg.acc[2] = sensor->accRaw.z;

    // 3. 最原始的陀螺：angularRateRaw（rad/s）
    sd_msg.gyro[0] = sensor->angularRateRaw.x;
    sd_msg.gyro[1] = sensor->angularRateRaw.y;
    sd_msg.gyro[2] = sensor->angularRateRaw.z;

    // reserve 置零
    for (int i = 0; i < 4; i++) {
        sd_msg.reserve[i] = 0.0f;
    }

	// printf("[FC] sending ROSUAV_IMU_RAW acc=[%.6f %.6f %.6f] gyro=[%.6f %.6f %.6f]\n",
    //        sd_msg.acc[0], sd_msg.acc[1], sd_msg.acc[2],
    //        sd_msg.gyro[0], sd_msg.gyro[1], sd_msg.gyro[2]);

    // 发送：仿照其他 send_struct 调用
    ttalink_rosuav_imu_raw_send_struct(TTALINK_FC_ADDRESS, TTALINK_SH_ADDRESS, addr2chan(TTALINK_FC_ADDRESS), &sd_msg);
}

//////////////////////////////////////////////////////////////////

void update_rosuav_switch_baro_data(sensor_t *sensor)
{
	ttalink_baro_sensors_data_t sd_msg;
	memset(&sd_msg,0,sizeof(ttalink_baro_sensors_data_t));

	// sd_msg.update_time = sensor->baro.baro_raw1.sampling_time; /*< This is update time of baro.*/
	// sd_msg.sensor_index = 1;
	// sd_msg.temperature = (signed int)(100*sensor->baro.baro_raw1.Actual_Temperature); /*< This is baro sensor temperature.*/
	// sd_msg.pressure = sensor->baro.baro_raw1.Actual_Pressure; /*< This is air pressure.*/
	// sd_msg.baro_e = 0.0f;
	// sd_msg.baro_var = 0.0f;

	ttalink_baro_sensors_data_send_struct(TTALINK_FC_ADDRESS, TTALINK_SH_ADDRESS, addr2chan(TTALINK_FC_ADDRESS),&sd_msg);
}

void update_rosuav_switch_mag_data(sensor_t *sensor)
{
	static unsigned int count=0;

	ttalink_mag_sensors_data_t sd_msg;
	memset(&sd_msg,0,sizeof(ttalink_mag_sensors_data_t));
	count++;
	sd_msg.sensor_index = 1; //通过 该值判断 原始值与校准后的值，1或者2 校准后值

	sd_msg.update_time = ACRL_GetTimeMs();
	sd_msg.mag_raw[0] = sensor->compass.x * 1000;
	sd_msg.mag_raw[1] = sensor->compass.y * 1000;
	sd_msg.mag_raw[2] = sensor->compass.z * 1000;

	ttalink_mag_sensors_data_send_struct(TTALINK_FC_ADDRESS, TTALINK_SH_ADDRESS, addr2chan(TTALINK_FC_ADDRESS),&sd_msg);
}

void update_rosuav_switch_gps_data(sensor_t *sensor)
{
	ttalink_gps_sensors_data_t sd_msg;
	memset(&sd_msg,0,sizeof(ttalink_gps_sensors_data_t));

	sd_msg.sensor_index = 1;
	sd_msg.gpsFix = sensor->gpsDetails.fixState;
	sd_msg.carrSoln = 0;
	sd_msg.hp_longitude = (double)sensor->gpsPosition.x / 10000000;
	sd_msg.hp_latitude = (double)sensor->gpsPosition.y / 10000000;
	sd_msg.hp_hMSL = sensor->gpsPosition.z / 100;
	sd_msg.temperature = 0;
	sd_msg.longitude = sensor->gpsPosition.x ;
	sd_msg.latitude = sensor->gpsPosition.y;
	sd_msg.altitude = sensor->gpsPosition.z / 100;
	sd_msg.vel_n = sensor->gpsVelocity.x;
	sd_msg.vel_e = sensor->gpsVelocity.y;
	sd_msg.vel_d = sensor->gpsVelocity.z;
	//sd_msg.diff_vel[3] =
	sd_msg.ground_vel = sqrt(sd_msg.vel_n*sd_msg.vel_n + sd_msg.vel_e*sd_msg.vel_e);
	sd_msg.heading = 0;
	sd_msg.update_time = 0;
	sd_msg.pos_acc = sensor->gpsDetails.pdop;
	sd_msg.speed_acc = sensor->gpsDetails.sacc;
	sd_msg.hor_acc = sensor->gpsDetails.hacc;
	sd_msg.ver_acc = sensor->gpsDetails.vacc;
	sd_msg.course_acc = 0;
	sd_msg.sate_num = sensor->gpsDetails.totalSatelliteNumberUsed;

	sd_msg.year = (int)(sensor->gpsData/10000);
	sd_msg.month = (int)((sensor->gpsData - sd_msg.year * 10000)*0.01);
	sd_msg.date = sensor->gpsData - (sd_msg.year * 10000 + sd_msg.month*100);
	sd_msg.hour = (int)(sensor->gpsTime/10000);
	sd_msg.minuter = (int)((sensor->gpsTime - sd_msg.hour * 10000)*0.01);
	sd_msg.second = sensor->gpsTime - (sd_msg.hour * 10000 + sd_msg.minuter*100);

	ttalink_gps_sensors_data_send_struct(TTALINK_FC_ADDRESS, TTALINK_SH_ADDRESS, addr2chan(TTALINK_FC_ADDRESS),&sd_msg);
}

void update_rosuav_switch_rtk_data(sensor_t *sensor)
{
	ttalink_rtk_sensors_data_t sd_msg;
	memset(&sd_msg,0,sizeof(ttalink_rtk_sensors_data_t));

	sd_msg.stas_number = 0;
	sd_msg.stas_number_tracked = 0;
	sd_msg.stas_number_usedinsolution = 0;
	sd_msg.gps_time_week = 0;
	sd_msg.gps_time_ms = 0;
	sd_msg.sol_status = 0;
	sd_msg.pos_type = sensor->rtkPosInfo;
	sd_msg.rtk_lat = sensor->rtkPos.latitude;
	sd_msg.rtk_lon = sensor->rtkPos.longitude;
	sd_msg.rtk_alt = sensor->rtkPos.hfsl;
	sd_msg.undulation = 0;
	sd_msg.lat_std_deviation = 0;
	sd_msg.lon_std_deviation = 0;
	sd_msg.alt_std_deviation =0;

	sd_msg.hor_speed = sensor->rtkVelocity.x;
	sd_msg.trk_gnd = 0;
	sd_msg.vert_speed = sensor->rtkVelocity.y;
	sd_msg.length = 0;
	sd_msg.heading = sensor->rtkYaw;
	sd_msg.pitch = 0;
	sd_msg.hdg_std_deviation = 0;
	sd_msg.ptch_std_deviation = 0;
	sd_msg.GDOP = 0;
	sd_msg.PDOP = 0;
	sd_msg.HDOP = 0;

	ttalink_rtk_sensors_data_send_struct(TTALINK_FC_ADDRESS, TTALINK_SH_ADDRESS, addr2chan(TTALINK_FC_ADDRESS),&sd_msg);
}
//////////////////////////////////////////////////////////////////

void update_rc_input_data(signed short *rc,unsigned char max_ch)
{
	ttalink_stream_rc_t sd_msg;

	for(unsigned char i=0;i<max_ch;i++)
	{
		sd_msg.rc_input[i] = (unsigned short)(rc[i]*0.48828125f+1500); //范围 ±1024 --> 1000~2000
	}

	ttalink_stream_rc_send_struct(TTALINK_FC_ADDRESS, TTALINK_SH_ADDRESS, addr2chan(TTALINK_FC_ADDRESS),&sd_msg);
}

void update_ctrl_target_data(void)
{
	ttalink_rosuav_ctrl_target_data_t sd_msg;
	memset(&sd_msg,0,sizeof(ttalink_rosuav_ctrl_target_data_t));

	xSemaphoreTake( loopInputMutex, portMAX_DELAY );
	sd_msg.motor_flag = GetLoopInput()->motor_flag;
	sd_msg.flight_flag = GetLoopInput()->flight_flag;

	sd_msg.longi = GetLoopInput()->longitude * M_RAD_TO_DEG_F;
	sd_msg.latit = GetLoopInput()->latitude * M_RAD_TO_DEG_F;
	sd_msg.altit = GetLoopInput()->altitude;
	sd_msg.velN = GetLoopInput()->vel_nav.x;
	sd_msg.velE = GetLoopInput()->vel_nav.y;
	sd_msg.velD = GetLoopInput()->vel_nav.z;
	sd_msg.atti_pitch = GetLoopInput()->euler.pitch;
	sd_msg.atti_roll = GetLoopInput()->euler.roll;
	sd_msg.atti_yaw = GetLoopInput()->euler.yaw;
	sd_msg.accX = GetLoopInput()->acc_body.x;
	sd_msg.accY = GetLoopInput()->acc_body.y;
	sd_msg.accZ = GetLoopInput()->acc_body.z;
	sd_msg.gyro_pitch = GetLoopInput()->euler_rate.pitch;
	sd_msg.gyro_roll = GetLoopInput()->euler_rate.roll;
	sd_msg.gyro_yaw = GetLoopInput()->euler_rate.yaw;
	sd_msg.torque_x = GetLoopInput()->control_out.torque.x;
	sd_msg.torque_y = GetLoopInput()->control_out.torque.y;
	sd_msg.torque_z = GetLoopInput()->control_out.torque.z;
	sd_msg.thrust = GetLoopInput()->control_out.thrust;
	sd_msg.matrix_step = GetLoopInput()->control_out.motor_matrix_status;
	sd_msg.accN = GetLoopInput()->acc_nav.x;
	sd_msg.accE = GetLoopInput()->acc_nav.y;
	sd_msg.accD = GetLoopInput()->acc_nav.z;

	for(unsigned char i=0;i<8;i++)
	{
		sd_msg.motor_pwm[i] = GetLoopInput()->ctrl_motor_pwm[i];
	}
	xSemaphoreGive( loopInputMutex );

	ttalink_rosuav_ctrl_target_data_send_struct(TTALINK_FC_ADDRESS, TTALINK_SH_ADDRESS, addr2chan(TTALINK_FC_ADDRESS),&sd_msg);
}

void update_ctrl_feed_back(sensor_t *sensor)
{
	ttalink_rosuav_ctrl_feed_back_t sd_msg;

	memset(&sd_msg,0,sizeof(ttalink_rosuav_ctrl_feed_back_t));

	sd_msg.navi_mode = 1;
	sd_msg.navi_atti_mode = 1;

	sd_msg.longi = sensor->positionFused.longitude * M_RAD_TO_DEG_F;
	sd_msg.latit = sensor->positionFused.latitude * M_RAD_TO_DEG_F;
	sd_msg.altit = sensor->positionFused.altitude;

	sd_msg.velN = sensor->velocity.data.x;
	sd_msg.velE = sensor->velocity.data.y;
	sd_msg.velD = sensor->velocity.data.z;

	sd_msg.atti_pitch = sensor->angleEuler.pitch;
	sd_msg.atti_roll = sensor->angleEuler.roll;
	sd_msg.atti_yaw = sensor->angleEuler.yaw;

	sd_msg.gyro_pitch = sensor->angularRateFusioned.x;
	sd_msg.gyro_roll = sensor->angularRateFusioned.y;
	sd_msg.gyro_yaw = sensor->angularRateFusioned.z;

	sd_msg.quat[0] = sensor->quaternion.q0;
	sd_msg.quat[1] = sensor->quaternion.q1;
	sd_msg.quat[2] = sensor->quaternion.q2;
	sd_msg.quat[3] = sensor->quaternion.q3;

	sd_msg.accN = sensor->accRaw.x;
	sd_msg.accE = sensor->accRaw.y;
	sd_msg.accD = sensor->accRaw.z;

	// USER_LOG_INFO("update_ctrl_feed_back -- atti pitch	------------->>>>%f ", sd_msg.atti_pitch );

	ttalink_rosuav_ctrl_feed_back_send_struct(TTALINK_FC_ADDRESS, TTALINK_SH_ADDRESS, addr2chan(TTALINK_FC_ADDRESS),&sd_msg);
}

void SendDataToFCTask(unsigned int systime, ttalink_message_t *msg,struct taskGcsTime_t *taskGcsTime) //,unsigned char lost_flag
{
	unsigned char send_flag = 0;
	unsigned int tempTime =  0;
	static unsigned int heart_beat_count=0,rosuav_baro_raw_count=0,rosuav_sim_mag = 0;
	static unsigned char index = 0;
	static unsigned int rosuav_baro_count=0;

	sensor_t *pSensorData = tta_getSensorData();
	if((systime - heart_beat_count)>=(501))
	{
		heart_beat_count = systime;
		update_heart_beat(1,index);
	}
	if((systime - rosuav_baro_raw_count)>=(202))
	{
		rosuav_baro_raw_count = systime;
		update_rosuav_baro_raw_data(pSensorData);
	}

	if((systime - rosuav_baro_count)>=(199))
	{
		rosuav_baro_count = systime;
		update_rosuav_switch_baro_data(pSensorData);
	}

	if((systime - rosuav_sim_mag) >= 21)
	{
		rosuav_sim_mag = systime;

		update_ctrl_feed_back(pSensorData);
	}

	if ((systime - rosuav_imu_raw_count) >= 5) { // 200Hz 例如
    	
		rosuav_imu_raw_count = systime;
    	
		update_rosuav_imu_raw_data(pSensorData);
	}
}






