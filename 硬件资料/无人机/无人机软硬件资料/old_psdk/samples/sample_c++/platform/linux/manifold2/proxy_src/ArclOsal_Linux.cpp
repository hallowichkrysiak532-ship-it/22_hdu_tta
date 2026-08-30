#include "stdlib.h"
#include <stdio.h>
#include "stdarg.h"
#include "ARCL_OSAL.h"

#include <sys/time.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/prctl.h>
#include <unordered_map>
#include <list>
#include <string.h>

typedef struct _my_thread_t
{
	const char* name;
	void *(*func)(void *);
	void* data;
	pthread_mutex_t mutex;
	pthread_cond_t cond;
	pthread_t thread;
	enum ARCL_STATURES_e status;
} my_thread_t;

class ThreadMap {
public:
	ThreadMap();
	virtual ~ThreadMap();
public:
	void put(pthread_t t, my_thread_t* ptr);
	void remove(pthread_t t);
	my_thread_t* get(pthread_t t);
private:
	std::unordered_map<pthread_t, my_thread_t*> map;
	pthread_mutex_t mutex;
};
ThreadMap::ThreadMap()
{
	pthread_mutex_init(&mutex, NULL);
}
ThreadMap::~ThreadMap()
{
	pthread_mutex_destroy(&mutex);
}
void ThreadMap::put(pthread_t t, my_thread_t* ptr)
{
	pthread_mutex_lock(&mutex);
	map[t] = ptr;
	pthread_mutex_unlock(&mutex);
}
void ThreadMap::remove(pthread_t t)
{
	pthread_mutex_lock(&mutex);
	map.erase(t);
	pthread_mutex_unlock(&mutex);
}
my_thread_t* ThreadMap::get(pthread_t t)
{
	my_thread_t* ptr = NULL;
	pthread_mutex_lock(&mutex);
	std::unordered_map<pthread_t, my_thread_t*>::const_iterator got = map.find(t);
	if(got != map.end()) {
		ptr = got->second;
	}
	pthread_mutex_unlock(&mutex);
	return ptr;
}

static ThreadMap threadMap;
void suspend_pthread(my_thread_t* item)
{
	if(item) {
		pthread_mutex_lock(&item->mutex);
		item->status = ARCL_PTHREAD_SUSPENDED;
		pthread_cond_wait(&item->cond, &item->mutex);
		item->status = ARCL_PTHREAD_RUNING;
		pthread_mutex_unlock(&item->mutex);
	}
}
void suspend(int s)
{
	pthread_t self = pthread_self();
	my_thread_t* got = threadMap.get(self);
	if(got) {
		my_thread_t* item = got;
		suspend_pthread(item);
	}
}

static void* my_thread_func(void* data)
{
	signal(SIGUSR1, SIG_IGN);
	my_thread_t * item = (my_thread_t*)data;
	prctl(PR_SET_NAME, item->name);
	pthread_mutex_init(&item->mutex, NULL);
	pthread_cond_init(&item->cond, NULL);
	threadMap.put(item->thread, item);
	signal(SIGUSR1, suspend);
	pthread_mutex_lock(&item->mutex);
	item->status = ARCL_PTHREAD_RUNING;
	pthread_mutex_unlock(&item->mutex);
	item->func(item->data);
	pthread_mutex_lock(&item->mutex);
	item->status = ARCL_PTHREAD_BLOCKED;
	pthread_mutex_unlock(&item->mutex);
	signal(SIGUSR1, SIG_IGN);
	threadMap.remove(item->thread);
	pthread_mutex_lock(&item->mutex);
	pthread_cond_broadcast(&item->cond);
	pthread_mutex_unlock(&item->mutex);
	pthread_cond_destroy(&item->cond);
	pthread_mutex_destroy(&item->mutex);
}

// create new pthread
unsigned char ACRL_CreatPthread(const char *pthreadName, void **pthread, void *(*pthreadFunc)(void *), unsigned int stackSize, unsigned int priority, void *arg)
{
	size_t ss;
	struct sched_param sp;
	
	my_thread_t* t = (my_thread_t*)ACRL_Malloc(sizeof(my_thread_t));

	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_attr_getstacksize(&attr, &ss);
	if(stackSize > ss) {
		ss = stackSize;
		pthread_attr_setstacksize(&attr, ss);
	}
	pthread_attr_getschedparam(&attr, &sp);
	sp.sched_priority = priority;
	pthread_attr_setschedparam(&attr, &sp);

	t->name = pthreadName;
	t->func = pthreadFunc;
	t->data = arg;
	t->status = ARCL_PTHREAD_READY;

	pthread_create(&(t->thread), &attr, my_thread_func, t);
	pthread_attr_destroy(&attr);
	
	*pthread = t;
	return OSAL_TURE;
}

void ACRL_PthreadSuspend(void *pthread)
{
	//printf("bunchen: %lu want suspend %lu@%p\n", pthread_self(), pthread?((my_thread_t*)pthread)->thread:0, pthread);
	if(pthread) {
		my_thread_t* t = (my_thread_t*)pthread;
		pthread_t self = pthread_self();
		if(self == t->thread) {
			suspend_pthread(t);
		}else{
			pthread_mutex_lock(&t->mutex);
			if(t->status == ARCL_PTHREAD_RUNING) {
				pthread_kill(t->thread, SIGUSR1);
			}
			pthread_mutex_unlock(&t->mutex);
		}
	}
}


void ACRL_PthreadResume(void *pthread)
{
	//printf("bunchen: %lu want resume %lu@%p\n", pthread_self(), pthread?((my_thread_t*)pthread)->thread:0, pthread);
	if(pthread) {
		my_thread_t* t = (my_thread_t*)pthread;
		pthread_mutex_lock(&t->mutex);
		pthread_cond_broadcast(&t->cond);
		pthread_mutex_unlock(&t->mutex);
	}
}

unsigned char ACRL_DestoryPthread(void **pthread)
{
	if(pthread && *pthread) {
		my_thread_t* t = (my_thread_t*)(*pthread);
		pthread_join(t->thread, NULL);
		t->status = ARCL_PTHREAD_DELETED;
		ACRL_Free(*pthread);
		*pthread = NULL;
		return OSAL_TURE;
	}else{
		return OSAL_FALSE;
	}
}


void ACRL_PthreadSleepMs(unsigned int time_ms)
{
	usleep(time_ms * 1000);
}



enum ARCL_STATURES_e  ACRL_GetPthreadStatus(void *pthread)
{

	if(pthread) {
		my_thread_t* t = (my_thread_t*)pthread;
		return t->status;
	}else{
		return ARCL_PTHREAD_INVALID;
	}
}



typedef struct _my_queue_t
{
	unsigned int bufSize;
	unsigned int queueLen;
	std::list<unsigned char *> *data;
	pthread_mutex_t mutex;
} my_queue_t;

////////////////////////////queue////////////////////////////////
unsigned char ACRL_CreateQueue(void **queue, unsigned int QueueLength, unsigned int ItemSize)
{
	my_queue_t* q = (my_queue_t*)ACRL_Malloc(sizeof(my_queue_t));
	q->bufSize = ItemSize;
	q->queueLen = QueueLength;
	q->data = new std::list<unsigned char *>();
	pthread_mutex_init(&q->mutex, NULL);
	*queue = q;
	return OSAL_TURE;
}


unsigned char ACRL_QueueSend(void *queue, const void *data, unsigned int xTicksToWait)
{
	if(queue) {
		unsigned char ret = OSAL_FALSE;
		my_queue_t* q = (my_queue_t*)queue;
		unsigned int now = ACRL_GetTimeMs();
		do {
			pthread_mutex_lock(&q->mutex);
			if(q->data->size() < q->queueLen) {
				unsigned char * d = (unsigned char *)ACRL_Malloc(q->bufSize);
				memcpy(d, data, q->bufSize);
				q->data->push_back(d);
				ret = OSAL_TURE;
			}
			pthread_mutex_unlock(&q->mutex);
			if(ret==OSAL_FALSE) {
				usleep(10000);
			}
		}while(ret==OSAL_FALSE && ACRL_GetTimeMs() < xTicksToWait + now);
		if(ret==OSAL_FALSE) {
			pthread_mutex_lock(&q->mutex);
			if((q->data->size() > 0)) {
				unsigned char * d = q->data->front();
				q->data->pop_front();
				ACRL_Free(d);
			}
			unsigned char * d = (unsigned char *)ACRL_Malloc(q->bufSize);
			memcpy(d, data, q->bufSize);
			q->data->push_back(d);
			ret = OSAL_TURE;
			pthread_mutex_unlock(&q->mutex);
		}
		return ret;
	}else{
		return OSAL_FALSE;
	}
}


unsigned char ACRL_QueueReceive(void *queue, void *data, unsigned int xTicksToWait)
{
	if(queue) {
		unsigned char ret = OSAL_FALSE;
		my_queue_t* q = (my_queue_t*)queue;
		unsigned int now = ACRL_GetTimeMs();
		do {
			pthread_mutex_lock(&q->mutex);
			if(q->data->size() > 0) {
				unsigned char * d = q->data->front();
				q->data->pop_front();
				if(data) {
					memcpy(data, d, q->bufSize);
				}
				ACRL_Free(d);
				ret = OSAL_TURE;
			}
			pthread_mutex_unlock(&q->mutex);
			if(ret==OSAL_FALSE) {
				usleep(10000);
			}
		}while(ret==OSAL_FALSE && ACRL_GetTimeMs() < xTicksToWait + now);
		return ret;
	}else{
		return OSAL_FALSE;
	}
}

unsigned char ARCL_ResetQueue(void *queue)
{
	if(queue) {
		my_queue_t* q = (my_queue_t*)queue;
		pthread_mutex_lock(&q->mutex);
		for(std::list<unsigned char *>::iterator it = q->data->begin(); it!=q->data->end(); it++) {
			ACRL_Free(*it);
		}
		q->data->clear();
		pthread_mutex_unlock(&q->mutex);
		return OSAL_TURE;
	}else{
		return OSAL_FALSE;
	}
}

unsigned char ACRL_QueuePeek(void *queue, void *data, unsigned int xTicksToWait)
{
	if(queue) {
		unsigned char ret = OSAL_FALSE;
		my_queue_t* q = (my_queue_t*)queue;
		unsigned int now = ACRL_GetTimeMs();
		do {
			pthread_mutex_lock(&q->mutex);
			if(q->data->size() > 0) {
				unsigned char * d = q->data->front();
				if(data) {
					memcpy(data, d, q->bufSize);
				}
				ret = OSAL_TURE;
			}
			pthread_mutex_unlock(&q->mutex);
			if(ret==OSAL_FALSE) {
				usleep(10000);
			}
		}while(ret==OSAL_FALSE && ACRL_GetTimeMs() < xTicksToWait + now);
		return ret;
	}else{
		return OSAL_FALSE;
	}
}

unsigned char ACRL_QueueOverWrite(void *queue, const void *data, unsigned int xTicksToWait)
{
	return OSAL_FALSE;
}


unsigned char ACRL_DestoryQueue(void **queue)
{
	if(queue && *queue) {
		my_queue_t* q = (my_queue_t*)(*queue);
		pthread_mutex_destroy(&q->mutex);
		for(std::list<unsigned char *>::iterator it = q->data->begin(); it!=q->data->end(); it++) {
			ACRL_Free(*it);
		}
		q->data->clear();
		delete q->data;
		ACRL_Free(*queue);
		*queue = NULL;
	}else{
		return OSAL_FALSE;
	}
}



/////////////////////////////Semaphore/////////////////////////////////////
unsigned char ACRL_CreatSemaphoreBinary(void **semaphore)
{
	sem_t* t = (sem_t*)ACRL_Malloc(sizeof(sem_t));
	sem_init(t, 0, 1);
	*semaphore = t;
	return OSAL_TURE;
}

unsigned char ACRL_TakeSemaphoreBinary(void *semaphore, unsigned int xTicksToWait)
{
	if(semaphore) {
		sem_t* t = (sem_t*)semaphore;
		unsigned char ret = OSAL_FALSE;
		if(xTicksToWait==0) {
			int err = sem_trywait(t);
			if(err == 0) {
				ret = OSAL_TURE;
			}
		}else{
			struct timespec ts;
			clock_gettime(CLOCK_REALTIME, &ts);
			long i = ts.tv_nsec + (xTicksToWait % 1000) * 1000000;
			ts.tv_sec += xTicksToWait / 1000 + i / 1000000000;
			ts.tv_nsec = i % 1000000000;
			int err = sem_timedwait(t, &ts);
			if(err == 0) {
				ret = OSAL_TURE;
			}
		}
		return ret;
	}else{
		return OSAL_FALSE;
	}
}

unsigned char ACRL_GiveSemaphoreBinary(void *semaphore)
{
	if(semaphore) {
		sem_t* t = (sem_t*)semaphore;
		sem_post(t);
		return OSAL_TURE;
	}else{
		return OSAL_FALSE;
	}
}

unsigned char ACRL_DestorySemaphoreBinary(void **semaphore)
{
	if(semaphore && *semaphore) {
		sem_t* t = (sem_t*)(*semaphore);
		sem_destroy(t);
		ACRL_Free(*semaphore);
		*semaphore = NULL;
		return OSAL_TURE;
	}else{
		return OSAL_FALSE;
	}
}


/////////////////////////memery////////////////////////////////
void *ACRL_Malloc(unsigned int size)
{
	return malloc(size);
}

void ACRL_Free(void *ptr)
{
	free(ptr);
}


/////////////////////time/////////////////////////////////
unsigned int ACRL_GetTimeMs(void)
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}




////////////////////////////FileSystem////////////////////////////////////
int ACRL_fopen(void ** fp, const char *filename, const char *mode)
{
	FILE *temp_fp;
	temp_fp = fopen(filename, mode);
	*fp = temp_fp;
	return 0;
}

int ACRL_fclose(void * fp)
{
	return fclose((FILE *)fp);
}



int ACRL_fread(void *ptr, size_t size, size_t nmemb, void *fp)
{
	int result = 0;
	result = fread(ptr, size, nmemb, (FILE *)fp);
	return result;
}


int ACRL_fwrite(void *ptr, size_t size, size_t nmemb, void *fp)
{
	int result = 0;
	result = fwrite(ptr, size, nmemb, (FILE *)fp);
	return result;
}


int ACRL_fseek(void * fp, long offset, int whence)
{
	return fseek((FILE *) fp, offset, whence);
}

int ACRL_unlink(const char *filename)
{
	if(unlink(filename) == 0)
		return OSAL_TURE;
	else
		return OSAL_FALSE;

}

long int ARCL_ftell(void *fp)
{
	return ftell((FILE *)fp);

}

void ACRL_PrintfPlanMessage(const char* fmt, ...)
{
#if 1
	char *logPbuf = (char *)ACRL_Malloc(384);
	va_list args;
	va_start(args, fmt);
	vsnprintf(logPbuf, 384, fmt, args);
	printf("%s", logPbuf);
	va_end(args);
	ACRL_Free(logPbuf);
#endif
}

void  ACRL_PrintfWayError(const char* fmt, ...)
{
#if 1
	char *logPbuf = (char *)ACRL_Malloc(384);
	va_list args;
	va_start(args, fmt);
	vsnprintf(logPbuf, 384, fmt, args);
	printf("%s", logPbuf);
	va_end(args);
	ACRL_Free(logPbuf);
#endif
}

void  ACRL_PrintfEventMessage(const char* fmt, ...)
{
#if 1
	char *logPbuf = (char *)ACRL_Malloc(384);
	va_list args;
	va_start(args, fmt);
	vsnprintf(logPbuf, 384, fmt, args);
	printf("%s", logPbuf);
	va_end(args);
	ACRL_Free(logPbuf);
#endif
}
