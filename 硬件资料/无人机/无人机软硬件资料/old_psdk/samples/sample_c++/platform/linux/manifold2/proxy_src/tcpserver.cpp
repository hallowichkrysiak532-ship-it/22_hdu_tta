#include "tcpserver.h"
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/select.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

CTCPServer::CTCPServer() :
    m_TcpCallback(nullptr),
    m_pBuffer(nullptr),
    server_fd(-1),
    accept_fd(-1)
{
    memset(&u_sockaddr, 0, sizeof(u_sockaddr));
    memset(&u_client_addr, 0, sizeof(u_client_addr));
}

CTCPServer::~CTCPServer()
{
    Uninit();
}

int CTCPServer::setNonBlocking(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1) {
        perror("fcntl F_GETFL");
        return -1;
    }

    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) == -1) {
        perror("fcntl F_SETFL O_NONBLOCK");
        return -1;
    }

    return 0;
}

int CTCPServer::doAccept()
{
    socklen_t addrlen = sizeof(u_client_addr);
    accept_fd = accept(server_fd, (struct sockaddr*)&u_client_addr, &addrlen);
    if (accept_fd == -1) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // 没有新的连接
            return -1;
        }
        perror("accept failed");
        exit(-1);
    }

    // 设置接受连接后的套接字为非阻塞模式
    if (setNonBlocking(accept_fd) == -1) {
        close(accept_fd);
        return -1;
    }

    // output connect message
    char clientIP[16];
    inet_ntop(AF_INET, &u_client_addr.sin_addr.s_addr, clientIP, sizeof(clientIP));
    unsigned short clientPort = ntohs(u_client_addr.sin_port);
    printf("client %s:%d is connect. \n", clientIP, clientPort);

    if (!m_pBuffer)
    {
        m_pBuffer = new uint8_t[MAX_READ_LINE];
    }

    return 0;
}

int CTCPServer::Init(int nPort)
{
    // memset(&u_sockaddr, 0, sizeof(u_sockaddr));
    u_sockaddr.sin_family = AF_INET;
    u_sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    u_sockaddr.sin_port = htons(nPort);

    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        printf(" socket error %s errno: %d", strerror(errno), errno);
        return -1;
    }

    // 设置监听套接字为非阻塞模式
    if (setNonBlocking(server_fd) == -1) {
        close(server_fd);
        return -1;
    }

    int flag = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, (const char*)&flag, sizeof(flag)) < 0)
    {
        printf("setsockopt error %s errno: %d nPort:%d", strerror(errno), errno, nPort);
        return -1;
    }

    int ret = bind(server_fd, (struct sockaddr*)&u_sockaddr, sizeof(u_sockaddr));
    if (ret < 0) {
        printf("bind socket error %s errno: %d nPort:%d", strerror(errno), errno, nPort);
        return -1;
    }

    // 将一个已经绑定（bind）了 IP 地址和端口号的套接字（socket）转换为监听套接字，使得该套接字可以接受来自客户端的连接请求
    ret = listen(server_fd, 128);
    if (ret == -1) {
        perror("listen failed");
        exit(-1);
    }

    return 0;
}

void CTCPServer::Uninit()
{
    if (m_pBuffer)
    {
        delete[] m_pBuffer;
        m_pBuffer = nullptr;
    }

    m_TcpCallback = nullptr;

    if (accept_fd >= 0)
    {
        close(accept_fd);
        accept_fd = -1;
    }

    if (server_fd >= 0)
    {
        close(server_fd);
        server_fd = -1;
    }
}

void CTCPServer::SetDataCB(tcp_data_callback cb, void* pUserData)
{
    m_TcpCallback = cb;
    m_pUserData = pUserData;
}

int CTCPServer::RecvData(char* buf, int len)
{
    fd_set readfds;
    FD_ZERO(&readfds);
    if (server_fd >= 0) {
        FD_SET(server_fd, &readfds);
    }
    if (accept_fd >= 0) {
        FD_SET(accept_fd, &readfds);
    }

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    int max_fd = (server_fd > accept_fd) ? server_fd : accept_fd;

    int activity = select(max_fd + 1, &readfds, NULL, NULL, &timeout);
    if (activity < 0) {
        perror("select error");
        return -1;
    }

    if (FD_ISSET(server_fd, &readfds)) {
        doAccept();
    }

    if (FD_ISSET(accept_fd, &readfds)) {
        socklen_t socklen = sizeof(u_sockaddr);
        int recv_len = recvfrom(accept_fd, (char*)buf, len, 0, (struct sockaddr*)&u_sockaddr, &socklen);
        if (recv_len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // 没有数据可读
                return 0;
            }
            return -1;
        }

        if (m_TcpCallback)
        {
            m_TcpCallback(m_pBuffer, recv_len, m_pUserData);
        }

        return recv_len;
    }

    return 0;
}

int CTCPServer::SendData(uint8_t* pData, int nLen)
{
    fd_set writefds;
    FD_ZERO(&writefds);
    if (accept_fd >= 0) {
        FD_SET(accept_fd, &writefds);
    }

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    int activity = select(accept_fd + 1, NULL, &writefds, NULL, &timeout);
    if (activity < 0) {
        perror("select error");
        return -1;
    }

    if (FD_ISSET(accept_fd, &writefds)) {
        if (sendto(accept_fd, (char*)pData, nLen, 0, (struct sockaddr*)&u_sockaddr, sizeof(struct sockaddr_in)) < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // 暂时不能发送数据
                return 0;
            }
            printf("tcp send data error: %s errno : %d", strerror(errno), errno);
            accept_fd = -1;
            return -1;
        }
    }

    return 0;
}
