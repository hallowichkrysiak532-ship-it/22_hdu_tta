#ifndef __FLIGHT_LOGIC_H
#define __FLIGHT_LOGIC_H

#include "ctrl_pid.h"
#ifdef MCU_PLATFORM
#include "ttalink.h"
#include "common.h"
#include "public_math.h"
#include "flight_ctrl_param.h"
#else
#pragma once
#include "public_math.h"
#include "ttalink.h"
#include "sensor.h"
#endif

#include "common/ttalink.h"

//typedef enum tagFlightCtrlStatus
//{
//	F_BASE_NC						 = 0,
//	F_BASE_GYRO_CTRL				,	
//	F_BASE_TAKE_OFF 			 	,
//	F_BASE_ATTI_AUTO_ALTI			,
//	F_BASE_ATTI_AUTO_ALTI_VEL		,
//	F_BASE_ATTI_WITHOUT_AUTOGAS		,
//	F_GPS_POS_TAKE_OFF 				,   	//  控制中没用到
//	F_GPS_POS_VEL					,       // 8
//	F_GPS_POS_VEL_ALTI_VEL			, 		// 9
//	F_GPS_HOVER_ACC					,
//	F_GPS_HOVER						,       // 11
//	F_GPS_HOVER_ALTI_VEL			,   	// 12
//	F_GPS_NAVGATION					,     	// 13
//	F_GPS_NAVGATION_ALTI_VEL		, 		// 14
//	F_GPS_AUTO_TAKE_OFF				,   	// 15
//}flightCtrlStatus_e;


#define	flightCtrlStatus_e FLIGHT_CTRL_STATUS_E


#define	LIMIT_ALTI_HIGH_P			3 	 			// 3
#define	LIMIT_GPS_POS_P_NAV		120*4

#define	LIMIT_ATTI_YAW_MAX_GYRO	 30.0f
#define	LIMIT_ATTI_YAW_OUT			 120


#ifdef __cplusplus
extern "C" {
#endif

FLIGHT_CTRL_STATUS_E GetFlightCtrlSta(void);
void SetFlightCtrlSta(FLIGHT_CTRL_STATUS_E sta);
void Flight_Ctrl(controlLoopInput_t *loopInput,sensor_t*  sensor_data_feedback, float dt);
void F_BaseStabilityControl(controlLoopInput_t *loopInput,sensor_t*  sensor_data_feedback, float dt);
void Landing_judgment(controlLoopInput_t *loopInput,sensor_t*  sensor_data_feedback, float dt);

//向量转换 导航坐标系->体坐标系
T_DjiVector3f vector_nav2body(T_DjiFcSubscriptionQuaternion q, _vector3 nav);

#ifdef __cplusplus
}
#endif


#endif
