#include "gcs.h"
#include "gcs_receive.h"
#include "gcs_transmit.h"
#include "string.h"
#include "sensor.h"
#include "tta_flight_control.h"
#include "dji_logger.h"
#include "flight_logic.h"

//extern sim_sensor_t sim_sensor;

uint32_t receive_fc_data_count = 0;
void hand_flight_data(ttalink_message_t *msg)
{
	controlLoopInput_t *re_loopint;
	ttalink_rosuav_ctrl_loop_input_t re_msg;
	ttalink_rosuav_ctrl_loop_input_decode(msg,&re_msg);

	// USER_LOG_INFO("recevie fc msg flight ctrl state------------->>>>%d ", GetFlightCtrlSta());

	re_loopint = GetLoopInput();
#if 1
	SetFlightCtrlSta((FLIGHT_CTRL_STATUS_E)re_msg.flight_ctrl_status);
	// re_loopint->flight_ctrl_state = re_msg.flight_ctrl_status;

	switch(GetFlightCtrlSta())
	{
//		case F_BASE_GYRO_CTRL:						//角速率控制模式
//			F_BodyGyroCtrl(loopInput, feedback, dt);
//			break;
		case ROS_F_BASE_ATTI_WITHOUT_AUTOGAS:			//姿态控制，手动油门
			re_loopint->euler.pitch = re_msg.atti_pitch;
			re_loopint->euler.roll = re_msg.atti_roll;
			re_loopint->euler.yaw = re_msg.atti_yaw;
			re_loopint->est_euler_rate.pitch = re_msg.gyro_pitch;
			re_loopint->est_euler_rate.roll = re_msg.gyro_roll;
			re_loopint->euler_rate.yaw = re_msg.gyro_yaw;
			re_loopint->control_out.thrust = re_msg.thro_thrust;
			break;
		case ROS_F_BASE_TAKE_OFF:						//起飞模式
		case ROS_F_GPS_AUTO_TAKE_OFF:					//GPS模式自动起飞
		case ROS_F_GPS_POS_TAKE_OFF:					//起飞模式

			re_loopint->longitude = re_msg.longi;
			re_loopint->latitude = re_msg.latit;
			re_loopint->altitude = re_msg.altit;
			re_loopint->vel_nav.x = re_msg.velN;
			re_loopint->vel_nav.y = re_msg.velE;
			re_loopint->vel_nav.z = re_msg.velD;
			re_loopint->acc_nav.x = re_msg.accN;
			re_loopint->acc_nav.y = re_msg.accE;
			re_loopint->acc_nav.z = re_msg.accD;
			re_loopint->euler.pitch = re_msg.atti_pitch;
			re_loopint->euler.roll = re_msg.atti_roll;
			re_loopint->euler.yaw = re_msg.atti_yaw;
			re_loopint->euler_rate.yaw = re_msg.gyro_yaw;

			break;
		case ROS_F_BASE_ATTI_AUTO_ALTI:					//姿态模式垂向高度控制
			re_loopint->euler.pitch = re_msg.atti_pitch;
			re_loopint->euler.roll = re_msg.atti_roll;
			re_loopint->euler.yaw = re_msg.atti_yaw;
			re_loopint->est_euler_rate.roll = re_msg.gyro_roll;
			re_loopint->est_euler_rate.pitch = re_msg.gyro_pitch;
			re_loopint->euler_rate.yaw = re_msg.gyro_yaw;
			re_loopint->altitude = re_msg.altit;
			re_loopint->est_vel_nav.z = re_msg.velD;
			break;

		case ROS_F_BASE_ATTI_AUTO_ALTI_VEL:				//姿态模式垂向速度控制
			re_loopint->euler.pitch = re_msg.atti_pitch;
			re_loopint->euler.roll = re_msg.atti_roll;
			re_loopint->euler.yaw = re_msg.atti_yaw;
			re_loopint->est_euler_rate.roll = re_msg.gyro_roll;
			re_loopint->est_euler_rate.pitch = re_msg.gyro_pitch;
			re_loopint->euler_rate.yaw = re_msg.gyro_yaw;
			re_loopint->vel_nav.z = re_msg.velD;


			break;
		case ROS_F_GPS_POS_VEL:							//GPS模式垂向高度控制，水平速度控制
			re_loopint->offset[0] = re_msg.velN;  //
			re_loopint->offset[1] = re_msg.velE;
			re_loopint->offset[2] = re_msg.velD;

			re_loopint->targetYaw  = re_msg.atti_yaw;

			re_loopint->yawThreshold = re_msg.param[0];
			re_loopint->posThreshold = re_msg.param[1];
			break;
		case ROS_F_GPS_POS_VEL_ALTI_VEL:				//GPS模式垂向速度控制，水平速度控制

			re_loopint->vel_nav.x = re_msg.velN;  //
			re_loopint->vel_nav.y = re_msg.velE;
			re_loopint->vel_nav.z = re_msg.velD;

			re_loopint->targetYaw  = re_msg.atti_yaw;

			re_loopint->fly_time = re_msg.param[0];

			break;
		case ROS_F_GPS_HOVER_ACC:						//10 位置锁定，高度锁定
		case ROS_F_GPS_HOVER:							//10 位置锁定，高度锁定

			re_loopint->longitude = re_msg.longi;
			re_loopint->latitude = re_msg.latit;
			re_loopint->altitude = re_msg.altit;
			re_loopint->est_vel_nav.x = re_msg.velN;
			re_loopint->est_vel_nav.y = re_msg.velE;
			re_loopint->est_vel_nav.z = re_msg.velD;
			re_loopint->est_acc_nav.x = re_msg.accN;
			re_loopint->est_acc_nav.y = re_msg.accE;
			re_loopint->est_acc_nav.z = re_msg.accD;
			re_loopint->altitude = re_msg.altit;
			re_loopint->euler.yaw = re_msg.atti_yaw;
			re_loopint->est_euler_rate.yaw = re_msg.gyro_yaw;
//			re_loopint->euler_rate.yaw = re_msg.gyro_yaw;

			break;

		case ROS_F_GPS_HOVER_ALTI_VEL:	 				//12 GPS模式水平位置锁定，垂向速度控制

			re_loopint->longitude = re_msg.longi;
			re_loopint->latitude = re_msg.latit;
			re_loopint->altitude = re_msg.altit;
			re_loopint->vel_nav.z = re_msg.velD;
			re_loopint->est_vel_nav.x = re_msg.velN;
			re_loopint->est_vel_nav.y = re_msg.velE;
			re_loopint->est_acc_nav.x = re_msg.accN;
			re_loopint->est_acc_nav.y = re_msg.accE;
			re_loopint->est_acc_nav.z = re_msg.accD;
			re_loopint->euler.yaw = re_msg.atti_yaw;
			re_loopint->est_euler_rate.yaw = re_msg.gyro_yaw;
//			re_loopint->euler_rate.yaw = re_msg.gyro_yaw;
			break;
		case ROS_F_GPS_NAVGATION:						//GPS模式航线控制
		case ROS_F_GPS_NAVGATION_ALTI_VEL:				//GPS模式航线控制，垂向速度控制（自动降落）

			re_loopint->longitude = re_msg.longi;
			re_loopint->latitude = re_msg.latit;
			re_loopint->altitude = re_msg.altit;
			re_loopint->est_vel_nav.x = re_msg.velN;
			re_loopint->est_vel_nav.y = re_msg.velE;
			re_loopint->est_vel_nav.z = re_msg.velD;
			re_loopint->est_acc_nav.x = re_msg.accN;
			re_loopint->est_acc_nav.y = re_msg.accE;
			re_loopint->est_acc_nav.z = re_msg.accD;
			re_loopint->est_euler.pitch = re_msg.atti_pitch;
			re_loopint->est_euler.roll = re_msg.atti_roll;
			re_loopint->euler.yaw = re_msg.atti_yaw;
			re_loopint->est_euler_rate.yaw = re_msg.gyro_yaw;
//			re_loopint->est_euler_rate.yaw = re_msg.gyro_yaw;
			break;
		default:
			break;
	}

	if (receive_fc_data_count++ % 50 == 0) {
		USER_LOG_INFO("re_loopint pos latitude:%f, longitude :%f, altitude:%f------------->>>> ",
		re_loopint->latitude,re_loopint->longitude, re_loopint->altitude);

		USER_LOG_INFO("re_loopint vel nav x:%f, y :%f, z:%f------yaw:%f------->>>> yaw_gyro:%f------->>>> ", re_loopint->est_vel_nav.x,
		re_loopint->est_vel_nav.y, re_loopint->est_vel_nav.z, re_loopint->euler.yaw, re_loopint->est_euler_rate.yaw);
	}


#endif
}

void hand_function_mode(ttalink_message_t *msg)
{
	ttalink_function_mode_t re_msg;
	controlLoopInput_t *re_loopint;

	re_loopint = GetLoopInput();
	ttalink_function_mode_decode(msg,&re_msg);

	// USER_LOG_INFO("recevie fc function cmd------------->>>>%d ", re_msg.function_cmd);

	switch(re_msg.function_cmd)
	{
		case TTALINK_UNLOCK_MOTOR:
		    if(re_loopint->motor_flag == 0)
			{
				Dji_FlightControlMotorsTurnOn();
			}
			re_loopint->motor_flag = 1;

		break;
		case TTALINK_LOCK_MOTOR:
		    if(re_loopint->motor_flag == 1)
			{
				Dji_FlightControlMotorsTurnOff();
			}
			re_loopint->motor_flag = 0;
			re_loopint->flight_flag = 0;


		break;

		default:

		break;
	}
}

ttalink_rc_input_t rc_msg;
signed short thro_rc = 0;
signed short rudd_rc = 0;
void hand_rc_input(ttalink_message_t *msg)
{
	ttalink_stream_rc_t sd_msg;

	ttalink_rc_input_decode(msg,&rc_msg);

	for(unsigned char i=0;i<16;i++)
	{
		sd_msg.rc_input[i] = (unsigned short)(rc_msg.rc_input[i]); //范围 ±1024 --> 1000~2000
//		sd_msg.rc_input[i] = (unsigned short)(rc_msg.rc_input[i])*2.048f - 3072; //范围 ±1024 --> 1000~2000
		//p1 =      0.4883 p2 =        1500
	}
	thro_rc = sd_msg.rc_input[2];
	rudd_rc = sd_msg.rc_input[3];
	ttalink_stream_rc_send_struct(TTALINK_FC_ADDRESS, TTALINK_SH_ADDRESS, addr2chan(TTALINK_FC_ADDRESS),&sd_msg);
}



