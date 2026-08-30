#ifndef __GCS_INTERFACE_H
#define __GCS_INTERFACE_H


#include "ttalink.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>



#ifdef __cplusplus
extern "C"{
#endif // __cplusplus


#define GCS_COMM_MAX  1024

#define GET_ARRAY_LEN(array,len){len = (sizeof(array) / sizeof(array[0]));}

void GcsInterfaceInit(void);
unsigned char GcsInterfaceSend(unsigned char *send_data, unsigned int send_num);
unsigned int GcsInterfaceReceive(unsigned char *receive_data, unsigned int max_len);
unsigned char EmbeInterfaceSend(unsigned char *send_data, unsigned int send_num);
unsigned int EmbeInterfaceReceive(unsigned char *receive_data, unsigned int max_len);


#ifdef __cplusplus
}
#endif

#endif
