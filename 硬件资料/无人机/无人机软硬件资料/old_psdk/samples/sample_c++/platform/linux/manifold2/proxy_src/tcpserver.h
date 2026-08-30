#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Log.h"
#ifdef _WIN32
#include <io.h>
#include <process.h>
#else
#include <unistd.h>
#endif

#include <stdint.h>
#include <netinet/in.h>


#define MAX_READ_LINE 1500



typedef void (*tcp_data_callback)(unsigned char* data, int len, void* userdata);

class CTCPServer
{
public:
    CTCPServer();
    ~CTCPServer();

    int Init(int nPort);
    void Uninit();

    void SetDataCB(tcp_data_callback cb, void* pUserData);

    int RecvData(char* buf, int len);

    int  SendData(uint8_t* pData, int nLen);

    int doAccept();

    int setNonBlocking(int fd);

private:
    struct sockaddr_in u_sockaddr;
    struct sockaddr_in  u_client_addr;
    tcp_data_callback m_TcpCallback;
    void* m_pUserData = nullptr;

    int server_fd = -1;
    int accept_fd = -1;
    uint8_t* m_pBuffer = nullptr;
    bool m_bInited = false;
};
