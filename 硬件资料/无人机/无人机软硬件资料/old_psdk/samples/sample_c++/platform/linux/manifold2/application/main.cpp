/**
 ********************************************************************
 * @file    main.cpp
 * @brief
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "application.hpp"
#include "tta_fc_subscription.h"
#include "ttalink.h"

#include "server_communication.h"
#include "ARCL_OSAL.h"

#include "stream_pusher.h"

#include "gcs_interface.h"
#include "Udp_data_port .h"
#include "tta_flight_control.h"

#include "gimbalControl.h"

#include "UdpDataPort.h"

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private values -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/

void *pth_flight_control;
void *pth_server_communication;
void *pth_main_pilot;
void *pth_fly_application;
void *pth_file_operation;
void *pth_gimbal_control;
void *pth_stream_pusher;
void *pth_psdk_action;
void *pth_psdk_interface;


/* Exported functions definition ---------------------------------------------*/
int main(int argc, char **argv)
{

    Application application(argc, argv);
    char inputChar;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiReturnCode returnCode;

    GcsInterfaceInit();

    //subscript task
    Dji_FcSubscriptionStartService();

    gimbalInit();

    // DataPort * udpport = new UdpDataPort("fc", 10086, "127.0.0.1", 47140, DataPort::MODE_OUTCOME | DataPort::MODE_INCOME , 0xffffff);

	// udpport->start();

    // 飞机控制线程：起飞、降落、飞行
    ACRL_CreatPthread("flight_control", &pth_flight_control, flight_control_task,0,99,(void *)NULL);
	ACRL_CreatPthread("server communication", &pth_server_communication, server_communication,0,97,NULL);
	ACRL_CreatPthread("stream pusher", &pth_stream_pusher, stream_pusher,0,80,(void *)NULL);
	start_udp();


    while(1);
   {
       sleep(1);
   }

    return 0;
}

/* Private functions definition-----------------------------------------------*/


/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
