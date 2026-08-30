#ifndef ___GCS_RECEIVE_H__
#define ___GCS_RECEIVE_H__

#include "ttalink.h"

#ifdef __cplusplus
extern "C" {
#endif

void hand_flight_data(ttalink_message_t *msg);
void hand_function_mode(ttalink_message_t *msg);
void hand_rc_input(ttalink_message_t *msg);

#ifdef __cplusplus
}
#endif

#endif
