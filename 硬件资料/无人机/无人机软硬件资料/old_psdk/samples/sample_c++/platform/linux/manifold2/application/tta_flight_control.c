/**
 ********************************************************************
 * @file    tta_flight_control.c
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
#include "dji_flight_controller.h"
#include "tta_flight_control.h"
#include "dji_fc_subscription.h"
#include "dji_platform.h"
#include "dji_logger.h"
#include <math.h>
#include "ARCL_OSAL.h"

#include "sensor.h"
#include "flight_logic.h"
#include "gcs_transmit.h"

#include <dji_aircraft_info.h>
/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
#pragma pack(1)
typedef struct {
    dji_f32_t x;
    dji_f32_t y;
    dji_f32_t z;
} T_DjiFlightControlVector3f; // pack(1)
#pragma pack()

typedef struct {
    E_DjiFcSubscriptionDisplayMode displayMode;
    char *displayModeStr;
} T_DjiFlightControlDisplayModeStr;

static const T_DjiFlightControlDisplayModeStr s_flightControlDisplayModeStr[] = {
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE, .displayModeStr = "attitude mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS, .displayModeStr = "p_gps mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ASSISTED_TAKEOFF, .displayModeStr = "assisted takeoff mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_TAKEOFF, .displayModeStr = "auto takeoff mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING, .displayModeStr = "auto landing mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME, .displayModeStr = "go home mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING, .displayModeStr = "force landing mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ENGINE_START, .displayModeStr = "engine start mode"},
    {.displayMode = 0xFF, .displayModeStr = "unknown mode"}
};


/* Private values -------------------------------------------------------------*/
static T_DjiOsalHandler *s_osalHandler = NULL;
static const double s_earthCenter = 6378137.0;
static const double s_degToRad = 0.01745329252;

controlLoopInput_t g_loop_input = {0};

/* Private functions declaration ---------------------------------------------*/

/* Exported functions definition ---------------------------------------------*/
T_DjiReturnCode tta_Dji_FlightControlRun()
{
    T_DjiReturnCode returnCode;

    USER_LOG_DEBUG("Init flight Control ");

    memset(&g_loop_input, 0, sizeof(controlLoopInput_t));

    returnCode = tta_Dji_FlightControlInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init flight Control sample failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    //Dji_FlightControlSample(flightCtrlSampleSelect);

    USER_LOG_DEBUG("Deinit Flight Control Sample");

    // returnCode = tta_Dji_FlightControlDeInit();
    // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     USER_LOG_ERROR("Deinit Flight Control sample failed,error code:0x%08llX", returnCode);
    //     return returnCode;
    // }

    return returnCode;
}


T_DjiReturnCode
Dji_FlightControlJoystickCtrlAuthSwitchEventCallback(T_DjiFlightControllerJoystickCtrlAuthorityEventInfo eventData)
{
    switch (eventData.joystickCtrlAuthoritySwitchEvent) {
        case DJI_FLIGHT_CONTROLLER_MSDK_GET_JOYSTICK_CTRL_AUTH_EVENT: {
            if (eventData.curJoystickCtrlAuthority == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_MSDK) {
                USER_LOG_INFO("[Event]Msdk request to obtain joystick ctrl authority\r\n");
            } else {
                USER_LOG_INFO("[Event]Msdk request to release joystick ctrl authority\r\n");
            }
            break;
        }
        case DJI_FLIGHT_CONTROLLER_INTERNAL_GET_JOYSTICK_CTRL_AUTH_EVENT: {
            if (eventData.curJoystickCtrlAuthority == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_INTERNAL) {
                USER_LOG_INFO("[Event]Internal request to obtain joystick ctrl authority\r\n");
            } else {
                USER_LOG_INFO("[Event]Internal request to release joystick ctrl authority\r\n");
            }
            break;
        }
        case DJI_FLIGHT_CONTROLLER_OSDK_GET_JOYSTICK_CTRL_AUTH_EVENT: {
            if (eventData.curJoystickCtrlAuthority == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_OSDK) {
                USER_LOG_INFO("[Event] Request to obtain joystick ctrl authority\r\n");
            } else {
                USER_LOG_INFO("[Event] Request to release joystick ctrl authority\r\n");
            }
            break;
        }
        case DJI_FLIGHT_CONTROLLER_RC_LOST_GET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc due to rc lost\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_RC_NOT_P_MODE_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc for rc is not in P mode\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_RC_SWITCH_MODE_GET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc due to rc switching mode\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_RC_PAUSE_GET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc due to rc pausing\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_RC_REQUEST_GO_HOME_GET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc due to rc request for return\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_LOW_BATTERY_GO_HOME_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc for low battery return\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_LOW_BATTERY_LANDING_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc for low battery land\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_OSDK_LOST_GET_JOYSTICK_CTRL_AUTH_EVENT:
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc due to sdk lost\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_NERA_FLIGHT_BOUNDARY_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc due to near boundary\r\n");
            break;
        default:
            USER_LOG_INFO("[Event]Unknown joystick ctrl authority event\r\n");
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/* Private functions definition-----------------------------------------------*/
T_DjiReturnCode tta_Dji_FlightControlInit(void)
{
    T_DjiReturnCode returnCode;

    s_osalHandler = DjiPlatform_GetOsalHandler();
    if (!s_osalHandler) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

    returnCode = DjiFlightController_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init flight controller module failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
        return returnCode;
    }
    s_osalHandler->TaskSleepMs(1000);

    // returnCode = DjiFcSubscription_Init();
    // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     USER_LOG_ERROR("Init data subscription module failed, error code:0x%08llX", returnCode);
    //     return returnCode;
    // }

    /*! subscribe fc data */
    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic flight status failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic display mode failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    // returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_AVOID_DATA,
    //                                               DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
    //                                               NULL);

    // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     USER_LOG_ERROR("Subscribe topic avoid data failed,error code:0x%08llX", returnCode);
    //     return returnCode;
    // }

    // returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
    //                                               DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
    //                                               NULL);

    // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     USER_LOG_ERROR("Subscribe topic quaternion failed,error code:0x%08llX", returnCode);
    //     return returnCode;
    // }

    // returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
    //                                               DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
    //                                               NULL);

    // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     USER_LOG_ERROR("Subscribe topic position fused failed,error code:0x%08llX", returnCode);
    //     return returnCode;
    // }

    // returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
    //                                               DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
    //                                               NULL);

    // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     USER_LOG_ERROR("Subscribe topic altitude fused failed,error code:0x%08llX", returnCode);
    //     return returnCode;
    // }

    // returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
    //                                               DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
    //                                               NULL);

    // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     USER_LOG_ERROR("Subscribe topic altitude of home point failed,error code:0x%08llX", returnCode);
    //     return returnCode;
    // }

    returnCode = DjiFlightController_RegJoystickCtrlAuthorityEventCallback(
        Dji_FlightControlJoystickCtrlAuthSwitchEventCallback);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT) {
        USER_LOG_ERROR("Register joystick control authority event callback failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode tta_Dji_FlightControlDeInit(void)
{
    T_DjiReturnCode returnCode;

    returnCode = DjiFlightController_Deinit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit flight controller module failed, error code:0x%08llX",
                       returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_DeInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit data subscription module failed, error code:0x%08llX",
                       returnCode);
        return returnCode;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}


T_DjiFcSubscriptionFlightStatus Dji_FlightControlGetValueOfFlightStatus(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionFlightStatus flightStatus;
    T_DjiDataTimestamp flightStatusTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                                                      (uint8_t *) &flightStatus,
                                                      sizeof(T_DjiFcSubscriptionFlightStatus),
                                                      &flightStatusTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic flight status error, error code: 0x%08X", djiStat);
        flightStatus = 0;
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", flightStatusTimestamp.millisecond,
                       flightStatusTimestamp.microsecond);
        USER_LOG_DEBUG("Flight status: %d.", flightStatus);
    }

    return flightStatus;
}


T_DjiFcSubscriptionDisplaymode Dji_FlightControlGetValueOfDisplayMode(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionDisplaymode displayMode;
    T_DjiDataTimestamp displayModeTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
                                                      (uint8_t *) &displayMode,
                                                      sizeof(T_DjiFcSubscriptionDisplaymode),
                                                      &displayModeTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic display mode error, error code: 0x%08X", djiStat);
        displayMode = 0;
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", displayModeTimestamp.millisecond,
                       displayModeTimestamp.microsecond);
        USER_LOG_DEBUG("Display mode : %d.", displayMode);
    }

    return displayMode;
}

bool Dji_FlightControlMotorStartedCheck(void)
{
    int motorsNotStarted = 0;
    int timeoutCycles = 20;

    while (Dji_FlightControlGetValueOfFlightStatus() != DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_ON_GROUND &&
           Dji_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ENGINE_START &&
           motorsNotStarted < timeoutCycles) {
        motorsNotStarted++;
        s_osalHandler->TaskSleepMs(100);
    }
    return motorsNotStarted != timeoutCycles ? true : false;
}

bool Dji_FlightControlTakeOffInAirCheck(void)
{
    int stillOnGround = 0;
    int timeoutCycles = 110;

    while (Dji_FlightControlGetValueOfFlightStatus() != DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR &&
           (Dji_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ASSISTED_TAKEOFF ||
            Dji_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_TAKEOFF) &&
           stillOnGround < timeoutCycles) {
        stillOnGround++;
        s_osalHandler->TaskSleepMs(100);
    }

    return stillOnGround != timeoutCycles ? true : false;
}

bool takeoffFinishedCheck(void)
{
    while (Dji_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_TAKEOFF ||
           Dji_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ASSISTED_TAKEOFF) {
        s_osalHandler->TaskSleepMs(1000);
    }

    return (Dji_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS ||
            Dji_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE) ? true : false;
}

bool Dji_FlightControlLandFinishedCheck(void)
{
    while (Dji_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING ||
           Dji_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR) {
        s_osalHandler->TaskSleepMs(1000);
    }

    return (Dji_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS ||
            Dji_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE) ? true : false;
}

uint8_t Dji_FlightControlGetDisplayModeIndex(E_DjiFcSubscriptionDisplayMode displayMode)
{
    uint8_t i;

    for (i = 0; i < sizeof(s_flightControlDisplayModeStr) / sizeof(T_DjiFlightControlDisplayModeStr); i++) {
        if (s_flightControlDisplayModeStr[i].displayMode == displayMode) {
            return i;
        }
    }

    return i;
}

bool Dji_FlightControlCheckActionStarted(E_DjiFcSubscriptionDisplayMode mode)
{
    int actionNotStarted = 0;
    int timeoutCycles = 20;

    while (Dji_FlightControlGetValueOfDisplayMode() != mode && actionNotStarted < timeoutCycles) {
        actionNotStarted++;
        s_osalHandler->TaskSleepMs(100);
    }

    if (actionNotStarted == timeoutCycles) {
        USER_LOG_ERROR("%s start failed, now flight is in %s.",
                       s_flightControlDisplayModeStr[Dji_FlightControlGetDisplayModeIndex(mode)].displayModeStr,
                       s_flightControlDisplayModeStr[Dji_FlightControlGetDisplayModeIndex(
                           Dji_FlightControlGetValueOfDisplayMode())].displayModeStr);
        return false;
    } else {
        USER_LOG_INFO("Now flight is in %s.",
                      s_flightControlDisplayModeStr[Dji_FlightControlGetDisplayModeIndex(mode)].displayModeStr);
        return true;
    }
}


bool Dji_FlightControlMonitoredTakeoff(void)
{
    T_DjiReturnCode djiStat;

    //! Start takeoff
    djiStat = DjiFlightController_StartTakeoff();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Request to take off failed, error code: 0x%08X", djiStat);
        return false;
    }

    //! Motors start check
    if (!Dji_FlightControlMotorStartedCheck()) {
        USER_LOG_ERROR("Takeoff failed. Motors are not spinning.");
        return false;
    } else {
        USER_LOG_INFO("Motors spinning...");
    }
    //! In air check
    if (!Dji_FlightControlTakeOffInAirCheck()) {
        USER_LOG_ERROR("Takeoff failed. Aircraft is still on the ground, but the "
                       "motors are spinning");
        return false;
    } else {
        USER_LOG_INFO("Ascending...");
    }
    //! Finished takeoff check
    if (!takeoffFinishedCheck()) {
        USER_LOG_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. "
                       "Please connect DJI GO.");
        return false;
    }

    return true;
}

bool Dji_FlightControlMonitoredLanding(void)
{
    T_DjiReturnCode djiStat;
    /*! Step 1: Start landing */
    djiStat = DjiFlightController_StartLanding();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Start landing failed, error code: 0x%08X", djiStat);
        return false;
    }

    /*! Step 2: check Landing start*/
    if (!Dji_FlightControlCheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING)) {
        USER_LOG_ERROR("Fail to execute Landing action!");
        return false;
    } else {
        /*! Step 3: check Landing finished*/
        if (!Dji_FlightControlLandFinishedCheck()) {
            USER_LOG_ERROR("Landing finished, but the aircraft is in an unexpected mode. "
                           "Please connect DJI Assistant.");
            return false;
        }
    }

    return true;
}


bool Dji_FlightControlMotorsTurnOn(void)
{
    T_DjiReturnCode returnCode;

    // returnCode = DjiFlightController_TurnOnMotors();
    // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     USER_LOG_ERROR("Turn on motors failed, error code: 0x%08X", returnCode);

    //     return false;
    // }

    return true;
}

bool Dji_FlightControlMotorsTurnOff(void)
{
    T_DjiReturnCode returnCode;

    returnCode = DjiFlightController_TurnOffMotors();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Turn off motors failed, error code: 0x%08X", returnCode);

        return false;
    }

    return true;
}


void DjiTest_FlightControlVelocityControlSample()
{
    T_DjiReturnCode returnCode;

    // USER_LOG_INFO("Flight control move-by-velocity sample start");
    // DjiTest_WidgetLogAppend("Flight control move-by-velocity sample start");

    // USER_LOG_INFO("--> Step 1: Obtain joystick control authority");
    // DjiTest_WidgetLogAppend("--> Step 1: Obtain joystick control authority");
    // returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
    //     goto out;
    // }
    // s_osalHandler->TaskSleepMs(1000);

    // USER_LOG_INFO("--> Step 2: Take off\r\n");
    // DjiTest_WidgetLogAppend("--> Step 2: Take off\r\n");
    // if (!DjiTest_FlightControlMonitoredTakeoff()) {
    //     USER_LOG_ERROR("Take off failed");
    //     goto out;
    // }
    // USER_LOG_INFO("Successful take off\r\n");
    // DjiTest_WidgetLogAppend("Successful take off\r\n");

    // USER_LOG_INFO(
    //     "--> Step 3: Move with north:0(m/s), earth:0(m/s), up:5(m/s), yaw:0(deg/s) from current point for 2s!");
    // DjiTest_WidgetLogAppend(
    //     "--> Step 3: Move with north:0(m/s), earth:0(m/s), up:5(m/s), yaw:0(deg/s) from current point for 2s!");
    // DjiTest_FlightControlVelocityAndYawRateCtrl((T_DjiTestFlightControlVector3f) {0, 0, 5.0}, 0, 2000);

    // USER_LOG_INFO("--> Step 4: Emergency brake for 2s");
    // DjiTest_WidgetLogAppend("--> Step 4: Emergency brake for 2s");
    // returnCode = DjiFlightController_ExecuteEmergencyBrakeAction();
    // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     USER_LOG_ERROR("Emergency brake failed, error code: 0x%08X", returnCode);
    //     goto out;
    // }
    // s_osalHandler->TaskSleepMs(2000);
    // returnCode = DjiFlightController_CancelEmergencyBrakeAction();
    // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     USER_LOG_ERROR("Cancel emergency brake action failed, error code: 0x%08X", returnCode);
    //     goto out;
    // }

    // USER_LOG_INFO(
    //     "--> Step 5: Move with north:-1.5(m/s), earth:2(m/s), up:0(m/s), yaw:20(deg/s) from current point for 2s!");
    // DjiTest_WidgetLogAppend(
    //     "--> Step 5: Move with north:-1.5(m/s), earth:2(m/s), up:0(m/s), yaw:20(deg/s) from current point for 2s!");
    // DjiTest_FlightControlVelocityAndYawRateCtrl((T_DjiTestFlightControlVector3f) {-1.5, 2, 0}, 20, 2000);

    // USER_LOG_INFO("--> Step 6: Emergency brake for 2s");
    // DjiTest_WidgetLogAppend("--> Step 6: Emergency brake for 2s");
    // returnCode = DjiFlightController_ExecuteEmergencyBrakeAction();
    // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     USER_LOG_ERROR("Emergency brake failed, error code: 0x%08X", returnCode);
    //     goto out;
    // }
    // s_osalHandler->TaskSleepMs(2000);
    // returnCode = DjiFlightController_CancelEmergencyBrakeAction();
    // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     USER_LOG_ERROR("Cancel emergency brake action failed, error code: 0x%08X", returnCode);
    //     goto out;
    // }

    // USER_LOG_INFO(
    //     "--> Step 7: Move with north:3(m/s), earth:0(m/s), up:0(m/s), yaw:0(deg/s) from current point for 2.5s!");
    // DjiTest_WidgetLogAppend(
    //     "--> Step 7: Move with north:3(m/s), earth:0(m/s), up:0(m/s), yaw:0(deg/s) from current point for 2.5s!");
    // DjiTest_FlightControlVelocityAndYawRateCtrl((T_DjiTestFlightControlVector3f) {3, 0, 0}, 0, 2500);

    // USER_LOG_INFO("--> Step 8: Emergency brake for 2s");
    // DjiTest_WidgetLogAppend("--> Step 8: Emergency brake for 2s");
    // returnCode = DjiFlightController_ExecuteEmergencyBrakeAction();
    // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     USER_LOG_ERROR("Emergency brake failed, error code: 0x%08X", returnCode);
    //     goto out;
    // }
    // s_osalHandler->TaskSleepMs(2000);
    // returnCode = DjiFlightController_CancelEmergencyBrakeAction();
    // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     USER_LOG_ERROR("Cancel emergency brake action failed, error code: 0x%08X", returnCode);
    //     goto out;
    // }

    // USER_LOG_INFO(
    //     "--> Step 9: Move with north:-1.6(m/s), earth:-2(m/s), up:0(m/s), yaw:0(deg/s) from current point for 2.2s!");
    // DjiTest_WidgetLogAppend(
    //     "--> Step 9: Move with north:-1.6(m/s), earth:-2(m/s), up:0(m/s), yaw:0(deg/s) from current point for 2.2s!");
    // DjiTest_FlightControlVelocityAndYawRateCtrl((T_DjiTestFlightControlVector3f) {-1.6, -2, 0}, 0, 2200);

    // USER_LOG_INFO("--> Step 10: Emergency brake for 2s");
    // DjiTest_WidgetLogAppend("--> Step 10: Emergency brake for 2s");
    // returnCode = DjiFlightController_ExecuteEmergencyBrakeAction();
    // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     USER_LOG_ERROR("Emergency brake failed, error code: 0x%08X", returnCode);
    //     goto out;
    // }
    // s_osalHandler->TaskSleepMs(2000);
    // returnCode = DjiFlightController_CancelEmergencyBrakeAction();
    // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     USER_LOG_ERROR("Cancel emergency brake action failed, error code: 0x%08X", returnCode);
    //     goto out;
    // }

    // USER_LOG_INFO("--> Step 11: Landing\r\n");
    // DjiTest_WidgetLogAppend("--> Step 11: Landing\r\n");
    // if (!DjiTest_FlightControlMonitoredLanding()) {
    //     USER_LOG_ERROR("Landing failed");
    //     goto out;
    // }
    // USER_LOG_INFO("Successful landing\r\n");
    // DjiTest_WidgetLogAppend("Successful landing\r\n");

    // USER_LOG_INFO("--> Step 12: Release joystick authority");
    // DjiTest_WidgetLogAppend("--> Step 12: Release joystick authority");
    // returnCode = DjiFlightController_ReleaseJoystickCtrlAuthority();
    // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     USER_LOG_ERROR("Release joystick authority failed, error code: 0x%08X", returnCode);
    //     goto out;
    // }

// out:
//     USER_LOG_INFO("Flight control move-by-velocity sample end");
//     DjiTest_WidgetLogAppend("Flight control move-by-velocity sample end");
}

// 获取到的tcp数据
controlLoopInput_t * GetLoopInput(void)
{
    return &g_loop_input;
}
extern sensor_t  g_sensor_data;
#define SYS_FLIGHT_TIME 10
void *flight_control_task(void *p)
{
    float tar_yaw = 0;
    float tar_bat = 0;
	unsigned int print_time = 0;
    sensor_t *temp_sensor;
    // 控制器初始化
    tta_Dji_FlightControlRun();

    while(1)
	{
        // if(ACRL_GetTimeMs() >= print_time)
        // {
        //         print_time = ACRL_GetTimeMs()+1000;
        //        //printf("Motor=%d ，Flight=%d\r\n",DroneSta.motor_flag,DroneSta.flight_flag);

        //        //printf("Longi=%lf，Latit=%lf, Altit=%0.2f\r\n",feedback.longi,feedback.pos_y,feedback.pos_z);
        //     // std::cout << "Longi，Latit, Altit= "
        //     // << feedback.pos_x<< ","
        //     // << feedback.pos_y<< ","
        //     // << feedback.pos_z<<","
        //     // "\n";

        // }
        Flight_Ctrl( GetLoopInput(), tta_getSensorData(), SYS_FLIGHT_TIME);

        // temp_sensor = tta_getSensorData();
        // tar_yaw = g_sensor_data.angleEuler.yaw;
        // tar_yaw = temp_sensor->angleEuler.yaw;
        // tar_bat = temp_sensor->batteryInfo.voltage;

        // printf("Flight_Ctrl in task.feedback_yaw:%0.3f-------->>\r\n",tar_yaw);
		update_ctrl_target_data();
		// update_ctrl_feed_back(tta_getSensorData());


        usleep(SYS_FLIGHT_TIME*1000);
    }
}


/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
