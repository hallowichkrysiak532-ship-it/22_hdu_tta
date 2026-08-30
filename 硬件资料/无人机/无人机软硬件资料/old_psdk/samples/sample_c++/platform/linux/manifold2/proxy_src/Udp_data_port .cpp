#include "ARCL_OSAL.h"
#include "UdpDataPort.h"
#include "TcpDataPort.h"
#include "gcs_interface.h"

#include "tcpserver.h"

CTCPServer m_tcp_server;

static UdpDataPort udp("udp", 10086, "127.0.0.1", 47140, DataPort::MODE_INCOME | DataPort::MODE_OUTCOME);
//static TcpDataPort tcp("udp", 10086, "127.0.0.1", 47140, DataPort::MODE_INCOME | DataPort::MODE_OUTCOME);

void *pth_udp;
static int udp_run = 1;
void recv_udp(void* data, int size) {
		EmbeInterfaceSend((unsigned char *)data,size);

}

void send_udp(void* data, int size) {
	m_tcp_server.SendData((uint8_t*)data, size);
}

void* func1(void*)
{
    int count = 0;
	char buffer[1024] = {0};
	int length = 0;

    while(udp_run) {
		length = m_tcp_server.RecvData(buffer, 1024);
		if(length > 0)
		{
			recv_udp(buffer, length);
		}

		unsigned char queue_receive[1024];
		void *send;
		uint32_t send_len=0;

		while(1)
		{
			send_len = EmbeInterfaceReceive(queue_receive,1024);
			if(send_len > 0)
			{
				send = queue_receive;
				send_udp(send,send_len);
			}
			else
				break;
		}



        if(count) {
            usleep(10);
        }
    }
}
void start_udp() {
	// udp.start();

	m_tcp_server.Init(10086);
	ACRL_CreatPthread("udp", &pth_udp, func1,0,50,NULL);
}
void stop_udp() {
	udp_run = 0;
	ACRL_DestoryPthread(&pth_udp);
	// udp.stop();
}

