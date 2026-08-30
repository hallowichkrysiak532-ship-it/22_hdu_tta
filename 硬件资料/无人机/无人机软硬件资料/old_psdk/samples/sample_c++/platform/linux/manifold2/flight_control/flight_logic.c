#include "ctrl_pid.h"
#include "flight_logic.h"

#ifdef MCU_PLATFORM
#include "public_math.h"
#include "system.h"
#include "FreeRTOS.h"
#include "semphr.h"
#else
#include "ARCL_OSAL.h"
#include "dji_platform.h"
#include "dji_flight_controller.h"
#endif

#include "dji_logger.h"

static const double s_earthCenter = 6378137.0;
static const double s_degToRad = 0.01745329252;

FLIGHT_CTRL_STATUS_E flightCtrlStatus = ROS_F_BASE_NC;

#define	THRUST_SLOPE	3.5f		//推力斜率，表示起飞时推力从0增加到最大的时间
extern int ctrl_motor_pwm[16];
//获取控制状态
FLIGHT_CTRL_STATUS_E GetFlightCtrlSta(void)
{
	return flightCtrlStatus;
}

void SetFlightCtrlSta(FLIGHT_CTRL_STATUS_E sta)
{
	flightCtrlStatus = sta;
}

void vector_rate_Euler2Body(const double vector_enler_rate[3], double roll,
  double pitch, double vector_body_rate[3])
{
  double X2;
  double Y2;
  double dv4[9];
  int i15;
  int i16;
  X2 = roll * 3.1415926535897931 / 180.0;
  Y2 = pitch * 3.1415926535897931 / 180.0;
  dv4[0] = 1.0;
  dv4[3] = 0.0;
  dv4[6] = -sin(Y2);
  dv4[1] = 0.0;
  dv4[4] = cos(X2);
  dv4[7] = sin(X2) * cos(Y2);
  dv4[2] = 0.0;
  dv4[5] = -sin(X2);
  dv4[8] = cos(X2) * cos(Y2);
  for (i15 = 0; i15 < 3; i15++) {
    vector_body_rate[i15] = 0.0;
    for (i16 = 0; i16 < 3; i16++) {
      vector_body_rate[i15] += dv4[i15 + 3 * i16] * vector_enler_rate[i16];
    }
  }
}

//向量转换 欧拉角速度->体坐标角速度
_vector3 vector_rate_nav2body(float roll, float pitch, _vector3 nav_rate)
{
	_vector3 result;

	double ned_double[3], xyz_double[3];

	result.x = 0;
	result.y = 0;
	result.z = 0;

	ned_double[0] = nav_rate.x;
	ned_double[1] = nav_rate.y;
	ned_double[2] = nav_rate.z;

	vector_rate_Euler2Body(ned_double, (double)roll, (double)pitch, xyz_double);

	result.x = (float)xyz_double[0];
	result.y = (float)xyz_double[1];
	result.z = (float)xyz_double[2];

	return result;
}

void Quaternion2Cnb_NED(const double q[4], double cnb[9])
{
  cnb[0] = ((q[0] * q[0] + q[1] * q[1]) - q[2] * q[2]) - q[3] * q[3];
  cnb[1] = 2.0 * (q[1] * q[2] - q[0] * q[3]);
  cnb[2] = 2.0 * (q[1] * q[3] + q[0] * q[2]);
  cnb[3] = 2.0 * (q[1] * q[2] + q[0] * q[3]);
  cnb[4] = ((q[0] * q[0] - q[1] * q[1]) + q[2] * q[2]) - q[3] * q[3];
  cnb[5] = 2.0 * (q[2] * q[3] - q[0] * q[1]);
  cnb[6] = 2.0 * (q[1] * q[3] - q[0] * q[2]);
  cnb[7] = 2.0 * (q[2] * q[3] + q[0] * q[1]);
  cnb[8] = ((q[0] * q[0] - q[1] * q[1]) - q[2] * q[2]) + q[3] * q[3];
}

void vector_NED2XYZ_Quaternion(const double vector_NED[3], const double q[4],
  double vector_XYZ[3])
{
  double cnb[9];
  int i13;
  int i14;
  Quaternion2Cnb_NED(q, cnb);
  for (i13 = 0; i13 < 3; i13++) {
    vector_XYZ[i13] = 0.0;
    for (i14 = 0; i14 < 3; i14++) {
      vector_XYZ[i13] += cnb[i13 + 3 * i14] * vector_NED[i14];
    }
  }
}

//向量转换 导航坐标系->体坐标系
T_DjiVector3f vector_nav2body(T_DjiFcSubscriptionQuaternion q, _vector3 nav)
{
	T_DjiVector3f result;

	double q_double[4], ned_double[3], xyz_double[3];

	result.x = 0;
	result.y = 0;
	result.z = 0;

	q_double[0] = q.q0;
	q_double[1] = q.q1;
	q_double[2] = q.q2;
	q_double[3] = q.q3;

	ned_double[0] = nav.x;
	ned_double[1] = nav.y;
	ned_double[2] = nav.z;

	vector_NED2XYZ_Quaternion(ned_double, q_double, xyz_double);

	result.x = xyz_double[0];
	result.y = xyz_double[1];
	result.z = xyz_double[2];

	return result;
}

void vector_XYZ2NED_Quaternion(const float vector_XYZ[3], const float q[4],
  float vector_NED[3])
{
  float b_q[9];
  int i0;
  int i1;
  USER_LOG_INFO("vel flight control --------------->q[0]: %.4f, q[1]: %.4f, q[2]: %.4f, q[3]: %.4f !",q[0], q[1], q[2], q[3]);
  /* UNTITLED3 此处显示有关此函数的摘要 */
  /*    此处显示详细说明 */
  b_q[0] = ((q[0] * q[0] + q[1] * q[1]) - q[2] * q[2]) - q[3] * q[3];
  b_q[3] = 2.0F * (q[1] * q[2] - q[0] * q[3]);
  b_q[6] = 2.0F * (q[1] * q[3] + q[0] * q[2]);
  b_q[1] = 2.0F * (q[1] * q[2] + q[0] * q[3]);
  b_q[4] = ((q[0] * q[0] - q[1] * q[1]) + q[2] * q[2]) - q[3] * q[3];
  b_q[7] = 2.0F * (q[2] * q[3] - q[0] * q[1]);
  b_q[2] = 2.0F * (q[1] * q[3] - q[0] * q[2]);
  b_q[5] = 2.0F * (q[2] * q[3] + q[0] * q[1]);
  b_q[8] = ((q[0] * q[0] - q[1] * q[1]) - q[2] * q[2]) + q[3] * q[3];
  for (i0 = 0; i0 < 3; i0++) {
    vector_NED[i0] = 0.0F;
    for (i1 = 0; i1 < 3; i1++) {
      vector_NED[i0] += b_q[i0 + 3 * i1] * vector_XYZ[i1];
    }
  }
}


//判断起飞条件
unsigned char TakeOffSuccess(float acc_z, float motor_thrust)		//Z轴加速度   马达推力
{
	unsigned char result = 0;
	// float Forced_takeoff_thrust = 0.5f;

	// //计算强制起飞推力位置
	// Forced_takeoff_thrust = 1.05f*Get_Drone_Default_Mass()*9.81f;

	// if(acc_z < -0.03*9.81f || motor_thrust > Forced_takeoff_thrust)
	// {
	// 	result = 1;
	// }
	return result;
}

void F_AttiCtrlTakeOff(controlLoopInput_t *loopInput,sensor_t*  sensor_data_feedback, float dt)
{
	float thrust;
	//BaseAttitudeCtrl(sensor_data_feedback, loopInput, dt);

	if(loopInput->flight_flag == 0)
	{
		if(Dji_FlightControlMonitoredTakeoff())
		{
			loopInput->flight_flag = 1;
		}
	}
}

void F_AttiCtrlLanding(controlLoopInput_t *loopInput, sensor_t* snesor_data_feedback, float dt)
{
    float thrust;

	if(Dji_FlightControlMonitoredLanding())
	{
        loopInput->motor_flag = 0;
		loopInput->flight_flag = 0;
	}
}


T_DjiFcSubscriptionPositionFused Dji_FlightControlGetValueOfPositionFused(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionPositionFused positionFused = {0};
    T_DjiDataTimestamp positionFusedTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
                                                      (uint8_t *) &positionFused,
                                                      sizeof(T_DjiFcSubscriptionPositionFused),
                                                      &positionFusedTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic position fused error, error code: 0x%08X", djiStat);
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", positionFusedTimestamp.millisecond,
                       positionFusedTimestamp.microsecond);
        USER_LOG_DEBUG("PositionFused: %f, %f,%f,%d.", positionFused.latitude, positionFused.longitude,
                       positionFused.altitude, positionFused.visibleSatelliteNumber);
    }

    return positionFused;
}


dji_f32_t Dji_FlightControlGetValueOfRelativeHeight(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionAltitudeFused altitudeFused = 0;
    T_DjiFcSubscriptionAltitudeOfHomePoint homePointAltitude = 0;
    dji_f32_t relativeHeight = 0;
    T_DjiDataTimestamp relativeHeightTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
                                                      (uint8_t *) &homePointAltitude,
                                                      sizeof(T_DjiFcSubscriptionAltitudeOfHomePoint),
                                                      &relativeHeightTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic altitude of home point error, error code: 0x%08X", djiStat);
        return -1;
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", relativeHeightTimestamp.millisecond,
                       relativeHeightTimestamp.microsecond);
    }

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
                                                      (uint8_t *) &altitudeFused,
                                                      sizeof(T_DjiFcSubscriptionAltitudeFused),
                                                      &relativeHeightTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic altitude fused error, error code: 0x%08X", djiStat);
        return -1;
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", relativeHeightTimestamp.millisecond,
                       relativeHeightTimestamp.microsecond);
    }

    relativeHeight = altitudeFused - homePointAltitude;

    return relativeHeight;
}


T_DjiFcSubscriptionQuaternion Dji_FlightControlGetValueOfQuaternion(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionQuaternion quaternion = {0};
    T_DjiDataTimestamp quaternionTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                                      (uint8_t *) &quaternion,
                                                      sizeof(T_DjiFcSubscriptionQuaternion),
                                                      &quaternionTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic quaternion error, error code: 0x%08X", djiStat);
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", quaternionTimestamp.millisecond,
                       quaternionTimestamp.microsecond);
        USER_LOG_DEBUG("Quaternion: %f %f %f %f.", quaternion.q0, quaternion.q1, quaternion.q2, quaternion.q3);
    }

    return quaternion;
}


T_DjiVector3f Dji_FlightControlQuaternionToEulerAngle(const T_DjiFcSubscriptionQuaternion quat)
{
    T_DjiVector3f eulerAngle;
    double q2sqr = quat.q2 * quat.q2;
    double t0 = -2.0 * (q2sqr + quat.q3 * quat.q3) + 1.0;
    double t1 = (dji_f64_t) 2.0 * (quat.q1 * quat.q2 + quat.q0 * quat.q3);
    double t2 = -2.0 * (quat.q1 * quat.q3 - quat.q0 * quat.q2);
    double t3 = (dji_f64_t) 2.0 * (quat.q2 * quat.q3 + quat.q0 * quat.q1);
    double t4 = -2.0 * (quat.q1 * quat.q1 + q2sqr) + 1.0;
    t2 = (t2 > 1.0) ? 1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;
    eulerAngle.x = asin(t2);
    eulerAngle.y = atan2(t3, t4);
    eulerAngle.z = atan2(t1, t0);
    return eulerAngle;
}


T_DjiVector3f
Dji_FlightControlLocalOffsetFromGpsAndFusedHeightOffset(const T_DjiFcSubscriptionPositionFused target,
                                                            const T_DjiFcSubscriptionPositionFused origin,
                                                            const dji_f32_t targetHeight,
                                                            const dji_f32_t originHeight)
{
    T_DjiVector3f deltaNed;
    double deltaLon = target.longitude - origin.longitude;
    double deltaLat = target.latitude - origin.latitude;
    deltaNed.x = deltaLat * s_earthCenter;
    deltaNed.y = deltaLon * s_earthCenter * cos(target.latitude);
    deltaNed.z = targetHeight - originHeight;

    return deltaNed;
}


T_DjiVector3f
Dji_FlightControlVector3FSub(const T_DjiVector3f vectorA,
                                 const T_DjiVector3f vectorB)
{
    T_DjiVector3f result;
    result.x = vectorA.x - vectorB.x;
    result.y = vectorA.y - vectorB.y;
    result.z = vectorA.z - vectorB.z;
    return result;
}

dji_f32_t Dji_FlightControlVectorNorm(T_DjiVector3f v)
{
    return sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
}

int Dji_FlightControlSignOfData(dji_f32_t data)
{
    return data < 0 ? -1 : 1;
}

void Dji_FlightControlHorizCommandLimit(dji_f32_t speedFactor, dji_f32_t *commandX, dji_f32_t *commandY)
{
    if (fabs(*commandX) > speedFactor)
        *commandX = speedFactor * Dji_FlightControlSignOfData(*commandX);
    if (fabs(*commandY) > speedFactor)
        *commandY = speedFactor * Dji_FlightControlSignOfData(*commandY);
}

bool
Dji_FlightControlMoveByPositionOffset(const T_DjiVector3f offsetDesired, float yawDesiredInDeg,
                                          float posThresholdInM, float yawThresholdInDeg)
{
    int timeoutInMilSec = 2000;
    int controlFreqInHz = 50;  // Hz
    int cycleTimeInMs = 1000 / controlFreqInHz;
    int outOfControlBoundsTimeLimit = 10 * cycleTimeInMs;    // 10 cycles
    int withinControlBoundsTimeReqmt = 100 * cycleTimeInMs;  // 100 cycles
    int elapsedTimeInMs = 0;
    int withinBoundsCounter = 0;
    int outOfBounds = 0;
    int brakeCounter = 0;
    int speedFactor = 2;

	T_DjiOsalHandler *s_osalHandler = NULL;

	s_osalHandler = DjiPlatform_GetOsalHandler();
    if (!s_osalHandler) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

    //! get origin position and relative height(from home point)of aircraft.
    T_DjiFcSubscriptionPositionFused originGPSPosition = Dji_FlightControlGetValueOfPositionFused();
    dji_f32_t originHeightBaseHomePoint = Dji_FlightControlGetValueOfRelativeHeight();
    if (originHeightBaseHomePoint == -1) {
        USER_LOG_ERROR("Relative height is invalid!");
        return false;
    }

    T_DjiFlightControllerJoystickMode joystickMode = {
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_POSITION_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_VERTICAL_POSITION_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_YAW_ANGLE_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE,
        DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
    };
    DjiFlightController_SetJoystickMode(joystickMode);

    while (elapsedTimeInMs < timeoutInMilSec) {
        T_DjiFcSubscriptionPositionFused currentGPSPosition = Dji_FlightControlGetValueOfPositionFused();
        T_DjiFcSubscriptionQuaternion currentQuaternion = Dji_FlightControlGetValueOfQuaternion();
        dji_f32_t currentHeight = Dji_FlightControlGetValueOfRelativeHeight();
        if (originHeightBaseHomePoint == -1) {
            USER_LOG_ERROR("Relative height is invalid!");
            return false;
        }

        float yawInRad = Dji_FlightControlQuaternionToEulerAngle(currentQuaternion).z;
        //! get the vector between aircraft and origin point.

        T_DjiVector3f localOffset = Dji_FlightControlLocalOffsetFromGpsAndFusedHeightOffset(
            currentGPSPosition,
            originGPSPosition,
            currentHeight,
            originHeightBaseHomePoint);
        //! get the vector between aircraft and target point.
        T_DjiVector3f offsetRemaining = Dji_FlightControlVector3FSub(offsetDesired, localOffset);

        T_DjiVector3f positionCommand = offsetRemaining;
        Dji_FlightControlHorizCommandLimit(speedFactor, &positionCommand.x, &positionCommand.y);

        T_DjiFlightControllerJoystickCommand joystickCommand = {positionCommand.x, positionCommand.y,
                                                                offsetDesired.z + originHeightBaseHomePoint,
                                                                yawDesiredInDeg};
        DjiFlightController_ExecuteJoystickAction(joystickCommand);

        if (Dji_FlightControlVectorNorm(offsetRemaining) < posThresholdInM &&
            fabs(yawInRad / s_degToRad - yawDesiredInDeg) < yawThresholdInDeg) {
            //! 1. We are within bounds; start incrementing our in-bound counter
            withinBoundsCounter += cycleTimeInMs;
        } else {
            if (withinBoundsCounter != 0) {
                //! 2. Start incrementing an out-of-bounds counter
                outOfBounds += cycleTimeInMs;
            }
        }
        //! 3. Reset withinBoundsCounter if necessary
        if (outOfBounds > outOfControlBoundsTimeLimit) {
            withinBoundsCounter = 0;
            outOfBounds = 0;
        }
        //! 4. If within bounds, set flag and break
        if (withinBoundsCounter >= withinControlBoundsTimeReqmt) {
            break;
        }
        s_osalHandler->TaskSleepMs(cycleTimeInMs);
        elapsedTimeInMs += cycleTimeInMs;
    }

    while (brakeCounter < withinControlBoundsTimeReqmt) {
        s_osalHandler->TaskSleepMs(cycleTimeInMs);
        brakeCounter += cycleTimeInMs;
    }

    if (elapsedTimeInMs >= timeoutInMilSec) {
        USER_LOG_ERROR("Task timeout!");
        return false;
    }

    return true;
}




void Dji_FlightControlVelocityAndYawRateCtrl( T_DjiVector3f offsetDesired, float yawRate,
                                                 uint32_t timeMs)
{
    uint32_t originTime = 0;
    uint32_t currentTime = 0;
    uint32_t elapsedTimeInMs = 0;
    T_DjiReturnCode returnCode;
	T_DjiOsalHandler *s_osalHandler = NULL;

	s_osalHandler = DjiPlatform_GetOsalHandler();
    if (!s_osalHandler) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

    s_osalHandler->GetTimeMs(&originTime);
    s_osalHandler->GetTimeMs(&currentTime);
    elapsedTimeInMs = currentTime - originTime;
    T_DjiFlightControllerJoystickMode joystickMode = {
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE,
        DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
    };

    returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
        return returnCode;
    }
    // s_osalHandler->TaskSleepMs(500);

	// yawRate = 10;

	USER_LOG_INFO("Dji_FlightControlVelocityAndYawRateCtrl:%f, %f, %f , yaw :%f ", offsetDesired.x, offsetDesired.y,
	offsetDesired.z, yawRate);

    DjiFlightController_SetJoystickMode(joystickMode);
    T_DjiFlightControllerJoystickCommand joystickCommand = {offsetDesired.x, offsetDesired.y, offsetDesired.z,
                                                            yawRate};

	USER_LOG_INFO("Flight control elapsed time is:%d, timems is :%d ", elapsedTimeInMs, timeMs);

    while (elapsedTimeInMs <= timeMs) {
        DjiFlightController_ExecuteJoystickAction(joystickCommand);
        s_osalHandler->TaskSleepMs(2);
        s_osalHandler->GetTimeMs(&currentTime);
        elapsedTimeInMs = currentTime - originTime;
    }
}

void TransformationXYtoNE(float dataX, float dataY, float *dataN, float *dataE, float sin_yaw, float cos_yaw)
{
	*dataN = dataX*cos_yaw - dataY*sin_yaw;
	*dataE = dataX*sin_yaw + dataY*cos_yaw;
}

void vel_body2nav(float q[4], float vel_body[3], float vel_nav[3])
{
		float c11 = q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3];
		float c12 = 2*(q[1]*q[2]+q[0]*q[3]);
		float c13 = 2*(q[1]*q[3]-q[0]*q[2]);
		float c21 = 2*(q[1]*q[2]-q[0]*q[3]);
		float c22 = q[0]*q[0]-q[1]*q[1]+q[2]*q[2]-q[3]*q[3];
		float c23 = 2*(q[2]*q[3]+q[0]*q[1]);
		float c31 = 2*(q[1]*q[3]+q[0]*q[2]);
		float c32 = 2*(q[2]*q[3]-q[0]*q[1]);
		float c33 = q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3];

		vel_nav[0] = c11*vel_body[0]+c12*vel_body[1]+c13*vel_body[2];
		vel_nav[1] = c21*vel_body[0]+c22*vel_body[1]+c23*vel_body[2];
		vel_nav[2] = c31*vel_body[0]+c32*vel_body[1]+c33*vel_body[2];

}

extern sensor_t g_sensor_data;
//"with north:0(m/s), earth:0(m/s), up:5(m/s), yaw:0(deg/s) from current point for 2s"
void F_velCtrlFlight(controlLoopInput_t *loopInput,sensor_t*  sensor_data_feedback, float dt)
{
	T_DjiVector3f offsetDesired = {0};
	float desiredX = 0.0;
	float desiredY = 0.0;
    float q[4] = {0};
    float vel_body[3] = {0};
    float vel_nav[3] = {0};

    _vector3 vel_out = {0};
    _vector3 nav_rate = {0};


    q[0] = g_sensor_data.quaternion.q0;
    q[1] = g_sensor_data.quaternion.q1;
    q[2] = g_sensor_data.quaternion.q2;
    q[3] = g_sensor_data.quaternion.q3;

    vel_body[0] = loopInput->vel_nav.x;
    vel_body[1] = loopInput->vel_nav.y;
    vel_body[2] = -loopInput->vel_nav.z;

    USER_LOG_INFO("vel flight control --------------->angle yaw :%f!",g_sensor_data.angleEuler.yaw);
    vector_XYZ2NED_Quaternion(vel_body, q, vel_nav);
    offsetDesired.x = vel_nav[0];
    offsetDesired.y = vel_nav[1];
    offsetDesired.z = -vel_nav[2];
    USER_LOG_INFO("User send velocity ----------------->x: %.2f, y: %.2f, z: %.2f", loopInput->vel_nav.x, loopInput->vel_nav.y, loopInput->vel_nav.z);
    USER_LOG_INFO("vel flight control ----------------->x:%f, y:%f, z:%f, yaw:%f, time:%f!",
    offsetDesired.x,offsetDesired.y,offsetDesired.z, loopInput->targetYaw, loopInput->fly_time);


	//BaseAttitudeCtrl(sensor_data_feedback, loopInput, dt);
	Dji_FlightControlVelocityAndYawRateCtrl( offsetDesired, loopInput->targetYaw,
                                                 loopInput->fly_time);

}

void F_posCtrlFlight(controlLoopInput_t *loopInput,sensor_t*  sensor_data_feedback, float dt)
{
	T_DjiVector3f offsetDesired = {0};

    offsetDesired.x = loopInput->offset[0];
    offsetDesired.y = loopInput->offset[1];
    offsetDesired.z = loopInput->offset[2];

    USER_LOG_INFO("vel flight control ----------------->x:%f, y:%f, z:%f, yaw:%f!",
    offsetDesired.x,offsetDesired.y,offsetDesired.z, loopInput->targetYaw);


	//BaseAttitudeCtrl(sensor_data_feedback, loopInput, dt);
	Dji_FlightControlMoveByPositionOffset(offsetDesired, loopInput->targetYaw,
                                          loopInput->posThreshold, loopInput->yawThreshold);

}

//落地判断
void Landing_judgment(controlLoopInput_t *loopInput,sensor_t*  sensor_data_feedback, float dt)
{
	;
}

float pwm_output[4];
void Flight_Ctrl(controlLoopInput_t *loopInput,sensor_t*  sensor_data_feedback, float dt)
{
	switch(GetFlightCtrlSta())
	{
        // 起飞
		case ROS_F_GPS_AUTO_TAKE_OFF:					//GPS模式自动起飞
			F_AttiCtrlTakeOff(loopInput, sensor_data_feedback, dt);

            SetFlightCtrlSta(ROS_F_BASE_NC);
			break;

        case ROS_F_GPS_POS_VEL:
            F_posCtrlFlight(loopInput,sensor_data_feedback, dt);

            SetFlightCtrlSta(ROS_F_BASE_NC);
            break;

		case ROS_F_GPS_POS_VEL_ALTI_VEL:
			F_velCtrlFlight(loopInput,sensor_data_feedback, dt);

            SetFlightCtrlSta(ROS_F_BASE_NC);
		    break;

        // 降落
		case ROS_F_GPS_NAVGATION_ALTI_VEL:				//GPS模式航线控制，垂向速度控制（自动降落）
			F_AttiCtrlLanding(loopInput, sensor_data_feedback, dt);

            SetFlightCtrlSta(ROS_F_BASE_NC);
			break;

		default:
			break;
	}

}

