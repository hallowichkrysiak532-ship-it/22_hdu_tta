#ifndef __ARCL_OSAL_H__
#define __ARCL_OSAL_H__

#define OSAL_TURE 1
#define OSAL_FALSE 0

#include "stddef.h"


enum ARCL_STATURES_e
{
    ARCL_PTHREAD_RUNING = 0,
    ARCL_PTHREAD_READY,
    ARCL_PTHREAD_BLOCKED,
    ARCL_PTHREAD_SUSPENDED,
    ARCL_PTHREAD_DELETED,
    ARCL_PTHREAD_INVALID,
};

#ifdef __cplusplus
extern "C" {
#endif

unsigned char ACRL_CreatPthread(const char *pthreadName, void **pthread, void *(*pthreadFunc)(void *), unsigned int stackSize, unsigned int priority, void *arg);
void ACRL_PthreadSuspend(void *pthread);
void ACRL_PthreadResume(void *pthread);
unsigned char ACRL_DestoryPthread(void **pthread);
unsigned char ARCL_ResetQueue(void *queue);

void ACRL_PthreadSleepMs(unsigned int time_ms);
enum ARCL_STATURES_e  ACRL_GetPthreadStatus(void *pthread);

unsigned char ACRL_CreateQueue(void **queue, unsigned int QueueLength, unsigned int ItemSize);
unsigned char ACRL_QueueSend(void *queue, const void *data, unsigned int xTicksToWait);
unsigned char ACRL_QueueReceive(void *queue, void *data, unsigned int xTicksToWait);
unsigned char ACRL_DestoryQueue(void **queue);
unsigned char ARCL_ResetQueue(void *queue);
unsigned char ACRL_QueuePeek(void *queue, void *data, unsigned int xTicksToWait);
unsigned char ACRL_QueueOverWrite(void *queue, const void *data, unsigned int xTicksToWait);

unsigned char ACRL_CreatSemaphoreBinary(void **semaphore);
unsigned char ACRL_TakeSemaphoreBinary(void *semaphore, unsigned int xTicksToWait);
unsigned char ACRL_GiveSemaphoreBinary(void *semaphore);
unsigned char ACRL_DestorySemaphoreBinary(void **semaphore);

unsigned int ACRL_GetTimeMs(void);
void ACRL_PrintfPlanMessage(const char* fmt, ...);
void ACRL_PrintfWayError(const char* fmt, ...);
void  ACRL_PrintfEventMessage(const char* fmt, ...);

void *ACRL_Malloc(unsigned int size);
void ACRL_Free(void *ptr);

int ACRL_fopen(void ** fp, const char *filename, const char *mode);
int ACRL_fclose(void * fp);
int ACRL_fread(void *ptr, size_t size, size_t nmemb, void *fp);
int ACRL_fwrite(void *ptr, size_t size, size_t nmemb, void *fp);
int ACRL_fseek(void * fp, long offset, int whence);
int ACRL_unlink(const char *filename);
long int ARCL_ftell(void *fp);

#ifdef __cplusplus
}
#endif

#endif
