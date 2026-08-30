#ifndef __SENSOR_H
#define __SENSOR_H

#include "dji_fc_subscription.h"

typedef struct tagAngle_euler_t
{
	dji_f64_t roll;
	dji_f64_t pitch;
	dji_f64_t yaw;
}angle_euler_t;

typedef struct tag_sensor_t
{
	angle_euler_t angleEuler;
	T_DjiFcSubscriptionQuaternion quaternion;
	T_DjiFcSubscriptionAccelerationRaw accRaw;
	T_DjiFcSubscriptionAccelerationBody accBody;
    T_DjiFcSubscriptionAngularRateRaw angularRateRaw;
	T_DjiFcSubscriptionAngularRateFusioned angularRateFusioned;
    T_DjiFcSubscriptionAltitudeFused altitudeFused;
    T_DjiFcSubscriptionAltitudeBarometer altitudeBarometer;
    T_DjiFcSubscriptionAltitudeOfHomePoint altitudeHomePoint;
    T_DjiFcSubscriptionHeightRelative heightRelative;
    T_DjiFcSubscriptionPositionFused positionFused;
    T_DjiFcSubscriptionRC rc;
    T_DjiFcSubscriptionControlDevice controlDevice;
    T_DjiFcSubscriptionWholeBatteryInfo  batteryInfo;
    T_DjiFcSubscriptionVelocity velocity;
	T_DjiFcSubscriptionGpsVelocity gpsVelocity;
    T_DjiFcSubscriptionGpsPosition gpsPosition;
    T_DjiFcSubscriptionGpsDetails gpsDetails;
	T_DjiFcSubscriptionGpsDate gpsData;
    T_DjiFcSubscriptionGpsTime gpsTime;
    T_DjiFcSubscriptionRtkPosition rtkPos;
	T_DjiFcSubscriptionRtkYaw rtkYaw;
    T_DjiFcSubscriptionRtkYawInfo rtkYawInfo;
    T_DjiFcSubscriptionRtkPositionInfo rtkPosInfo;
    T_DjiFcSubscriptionRtkVelocity rtkVelocity;
	T_DjiFcSubscriptionCompass compass;
}sensor_t;

typedef struct tag_control_out_t
{
	unsigned int motor_matrix_status;
	float thrust;		//推力
	T_DjiVector3f torque;	//扭矩
}control_out_t;

typedef struct tag_loopinput_t
{
	unsigned char motor_flag;
	unsigned char flight_flag;

	//目标位置  WGS84坐标系
	double latitude;
	double longitude;
	float altitude;
	//目标速度 导航坐标系 NED
	T_DjiVector3f 		vel_nav;
	//前馈速度 导航坐标系 NED
	T_DjiVector3f 		est_vel_nav;
	//目标加速度 导航坐标系
	T_DjiVector3f		acc_nav;
	//前馈加速度 导航坐标系
	T_DjiVector3f 		est_acc_nav;
	//目标加速度  体坐标系
	T_DjiVector3f  	acc_body;
	//前馈加速度  体坐标系
	T_DjiVector3f  	est_acc_body;
	//目标欧拉角
	angle_euler_t euler;
	//前馈欧拉角
	angle_euler_t est_euler;
	//目标欧拉角速度
	angle_euler_t euler_rate;
	//前馈欧拉角速度
	angle_euler_t est_euler_rate;
	//目标体坐标系角速度
	T_DjiVector3f  gyro;
	//前馈体坐标系角速度
	T_DjiVector3f  est_gyro;
	//控制输出
	control_out_t control_out;
	//马达输出
	int ctrl_motor_pwm[16];

	float offset[3];  //

	float targetYaw;

	float yawThreshold;
	float posThreshold;
	float fly_time;
}controlLoopInput_t;



#endif


