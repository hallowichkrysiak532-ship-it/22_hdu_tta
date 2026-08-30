#ifndef __TTALINK_ROUT_H__
#define __TTALINK_ROUT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "../../../../../ttalink/common/ttalink.h"

#define	ROUNT_MAX	10

#define ROUNT_ENABLE		(1)
#define ROUNT_DISABLE		(0)

typedef struct __ttalink_rount_list
{
	ttalink_channel_t chan;
	unsigned char addr;
}ttalink_rount_list_t;


ttalink_channel_t addr2chan(unsigned char addr);

void ttalink_rount_system_init(void);

int is_own_addr(uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif
