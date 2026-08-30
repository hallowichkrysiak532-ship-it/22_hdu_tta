#pragma once


#include "ttalink.h"
#include "sensor.h"

#define portMAX_DELAY 0xffffffffUL

typedef struct tag_SemaphoreHandle_t
{;}SemaphoreHandle_t;

inline void  xSemaphoreGive(SemaphoreHandle_t Mutex)
{;}

inline void xSemaphoreTake(SemaphoreHandle_t Mutex, unsigned int delay)
{;}

struct taskGcsTime_t
{
	uint32_t global_count;
	uint32_t heart_beat_count ;
	uint32_t gps_location_count;
	uint32_t gps_ahrs_count;
	uint32_t device_data_count;
	uint32_t uav_status_data_count;
	uint32_t wp_point_count;
	uint32_t normal_count;
	uint32_t rc_count ;
	uint32_t exam_data_count;
	uint32_t version_data_count;
};


struct motor_ctrl_data_t
{
	unsigned char test_mode; //1 = test
	unsigned char roll_mode;
	unsigned char pitch_mode;
	unsigned char yaw_mode;
	float roll_ang;
	float pitch_ang;
	float yaw_ang;
	float roll_vel;
	float pitch_vel;
	float yaw_vel;
};

#ifdef __cplusplus
extern "C" {
#endif

struct motor_ctrl_data_t *GetMotionCtrlAddr(void);

void update_heart_beat(uint8_t state,unsigned char index);
void SendDataToFCTask(unsigned int systime, ttalink_message_t *msg,struct taskGcsTime_t *taskGcsTime);
void SetMotorTestMode(unsigned char mode);

void update_control_data(struct motor_ctrl_data_t ctrl);

void SendCaliEncoderCmd(unsigned index);


void send_log_string(char *str, unsigned int str_length);


void update_ctrl_feed_back(sensor_t *sensor);
void update_ctrl_target_data(void);

void update_mav_hil_motor(controlLoopInput_t *loopInput) ;
void update_rosuav_switch_imu_data(sensor_t *sensor);
void update_rosuav_imu_raw_data(sensor_t *sensor); 
void update_rosuav_switch_gps_data(sensor_t *sensor);
void update_rosuav_switch_mag_data(sensor_t *sensor);
void update_rosuav_switch_rtk_data(sensor_t *sensor);


#ifdef __cplusplus
}
#endif

