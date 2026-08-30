#ifndef __GCS_H
#define __GCS_H

#include "ARCL_OSAL.h"

#include "ttalink.h"
#include "ttalink_rount.h"
#include "ttalink_transmit.h"

#include "gcs_receive.h"
#include "gcs_transmit.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>



#define	 TTALINK_VERSION_INT	 (TTALINK_VERSION_MAJOR * 256)

#define FRAME_FC  1
#define FRAME_SH  2
#define FRAME_TC  3
#define FRAME_DEVICE  10

struct UpdateFrame_t
{
	unsigned char fc_img_flag;
	unsigned char sh_img_flag;
	unsigned char tc_img_flag;
	unsigned char um482_img_flag;
	unsigned char device_img_flag;

};


typedef struct log_crtl_stru {
		uint8_t ctrl_cmd;
		int  rount;
		uint8_t src_addr;
		uint8_t des_addr;
}log_ctrl_t;

//typedef struct {
//		PARAM_ID_TYPE index;
//		const char * name;
//		uint8_t name_size;
//}param_index_t;


typedef struct img_struct {
	uint16_t block;
	uint8_t type;
	uint8_t state;
	uint32_t count;
//	FIL	img_fp;
}img_struct_t;

#define PASS_START_MASK		(1 << 15)
#define PASS_RUN_MASK			(1 << 14)
#define PASS_END_MASK			(1 << 13)
#define PASS_INDEX_MASK		(0x1FFF)

typedef struct passthrought_struct {
	uint8_t data_buf[1024];
	int recv_index;
	int recv_offset;
//	int recv_flage;
} passthrought_struct_t;


typedef struct passthrough_header {
	uint8_t size;
	uint8_t type;
	uint16_t index;	 // :1 start,:2 runing 3: end :[4 ---> 16] index
	uint8_t data[250];  // max  length  251
}passthrough_header_t;

typedef struct gcs_comshell_struct {
	uint16_t buf_size;
	uint8_t *data_buf;
}gcs_comshell_data_t;

typedef struct gcs_vision_struct {
	uint16_t buf_size;
	uint8_t *data_buf;
}usart_vision_data_t;

typedef struct gcs_land_location {

	uint8_t copy_pub;
	uint8_t land_state;
	double land_latit;
	double land_longi;
	float alt;
	float yaw;
	int delay_time; 
	float param1;
	float param2;
	float param3;
	float param4;
}gcs_land_location_t;

struct gcs_location_param_t
{
	float stationBasicAltit;	
	float landRelativeAltit;	
	unsigned int serverUpTime;
	unsigned int appUpTime;
	unsigned char serverLostFlag;
	unsigned char appLostFlag;
	int server_first_hb;
	int app_first_hb;
	float gcs_wp_width;
	float gcs_wp_area;
	float gcs_wp_all_distance;
	float gcs_wp_remain_time;

};

typedef struct gcs_rc_input {
	uint8_t copy_pub;	
	uint8_t rc_input[8];
}gcs_rc_input_t;


#pragma pack()

#ifdef __cplusplus
extern "C" {
#endif

void UpdateGcs(void);

int64_t GetCpuId();

uint8_t get_lost_heart_flag(void);

void handle_fc_message(ttalink_message_t *msg);

unsigned char GcsNULLSend(unsigned char *send_data, unsigned int send_num);
unsigned int GcsNULLReceive(unsigned char *receive_data, unsigned int max_len);
void GcsInit(void);

void gcs_log_data_free(void *p);

uint8_t get_gcs_rc_input_copy();
void gcs_rc_pub_reset();

void hand_ttalink_rc_input(ttalink_message_t *msg);

void get_gcs_rc_input(signed short *rc);

void hand_general_request_module_version(ttalink_message_t * msg);


#ifdef __cplusplus
}
#endif

#endif
