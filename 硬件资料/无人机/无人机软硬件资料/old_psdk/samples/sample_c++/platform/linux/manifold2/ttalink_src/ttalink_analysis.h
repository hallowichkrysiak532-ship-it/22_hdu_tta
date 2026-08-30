#ifndef ___TTALINK_ANALYSIS_H__
#define ___TTALINK_ANALYSIS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ttalink.h"

unsigned char ttalink_parse_buf(unsigned char * buf_addr, ttalink_message_t* r_message, unsigned int *remain_length);

#ifdef __cplusplus
}
#endif

#endif
