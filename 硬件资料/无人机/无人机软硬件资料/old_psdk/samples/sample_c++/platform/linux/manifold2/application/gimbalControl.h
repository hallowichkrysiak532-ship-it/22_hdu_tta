#ifndef __GIMBALCONTROL_H__
#define __GIMBALCONTROL_H__

// #include "UdpDataPort.h"
#include <string>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif
#include "ttalink.h"
#ifdef __cplusplus
}
#endif

#include <unistd.h> 

struct Lock_param_t
{
	float kp;
	float ki;
	float kd;
	float p_out;
	float d_out;
	float integral;
	float old_gyro[4];
	float last_derivative;
	float dt;
	unsigned char integral_flag;
};

struct Lock_focus_t
{
    unsigned char cmd;
	int main_width;
	int main_height;
    int main_center_x;
    int main_center_y;
	int x;
	int y;
	int width;
	int height;
    int core_x;
    int core_y;
	int dx;
	int dy;	
	float zoom_param;
};

typedef struct tagFind_Target_t
{
   unsigned int time;
   unsigned int temp_time;
   double longi;
   double latit;
   float altit;
   unsigned char find_flag;
   unsigned short index;
   unsigned short num;
   unsigned char full_flag;
   unsigned int count;
   unsigned char send_flag;
}Find_Target_t;

typedef struct tagGeneralCmd_t
{
    int cmd;
    double param1;
    double param2;
    double param3;
    double param4;
    double param5;
    double param6;
    double param7;
}GeneralCmd_t;

void *gimbalControl(void *);

void gimbalInit();

void gimbal_general_command_handle(ttalink_general_command_t &gc);


#endif
