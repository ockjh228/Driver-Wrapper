/* ***************************************************************************
 * Kinova inc.
 * Project :
 *
 * Copyright (c) 2006-2018 Kinova Incorporated. All rights reserved.
 ****************************************************************************/

#ifndef __TRANSPORT_CLIENT_UDP_H__
#define __TRANSPORT_CLIENT_UDP_H__

#if defined(_WIN32) || defined(_WIN64)
// ---- win ----
#include <stdio.h>
#include <winsock2.h>
typedef int32_t socklen_t;
#elif defined(__linux__) || defined(__unix__) || defined(__APPLE__)
// --- linux ---
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>      // host struct
#include <sys/select.h> // use select() for multiplexing
#include <fcntl.h>      // for non-blocking

#include <iostream>
#include <unistd.h>
#include <ctime>
#include <stdio.h>
#include <unistd.h>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#else
#warning "Unknown OS type!"
#endif

#include <atomic>
#include <thread>
#include <mutex>

#include <string>
#include <functional>
#include <exception>

#include <iostream>
#include <chrono>

#include "ITransportClient.h"

namespace Kinova
{
namespace Api
{
    const uint32_t kApiPort = 10000;

    class TransportClientUdp : public ITransportClient
    {
        // Configuration
        bool                   	m_isInitialized;
        struct sockaddr_in      m_socketAddr;
        socklen_t               m_socketAddrSize;
        int32_t                 m_socketFd;
        
        #if defined(_WIN32) || defined(_WIN64)
        WSADATA                 m_wsa;
        #endif

        bool                    m_isUsingRcvThread;
        std::atomic<bool>       m_isRunning{true};
        std::mutex              m_sendMutex;

        // ---- non-blocking ----
        fd_set          m_original_rx;
        fd_set          m_readfds;

        int             numfd;
        struct hostent  *m_host;
        struct timeval  m_tv;
        // ----------------------

        static constexpr uint32_t kMaxTxBufferSize = 65507;
        static constexpr uint32_t kMaxRxBufferSize = 65507;

        char m_txBuffer[kMaxTxBufferSize];
        char m_rxBuffer[kMaxRxBufferSize];

        std::function<void(const char*, uint32_t)> m_onMessageCallback;

    public:
        TransportReadyStateEnum readyState;
        std::thread             m_receiveThread;

        TransportClientUdp(bool isUsingRcvThread = true);
        virtual ~TransportClientUdp();

        virtual bool connect(std::string host = "127.0.0.1", uint32_t port = Kinova::Api::kApiPort) override;
        virtual void disconnect() override;

        virtual void send(const char* txBuffer, uint32_t txSize) override;
        virtual void onMessage(std::function<void(const char*, uint32_t)> callback) override;

        virtual char* getTxBuffer(uint32_t const& allocation_size) override { return m_txBuffer; }
        virtual size_t getMaxTxBufferSize() override { return kMaxTxBufferSize; }

        int processReceive(long rcvTimeout_usec);
        int processReceive(struct timeval rcvTimeout_tv);

        virtual void getHostAddress(std::string &host, uint32_t &port) override {
            host = mHostAddress;
            port = mHostPort;
        };

    private:
        std::string mHostAddress;
        uint32_t mHostPort;

        void receiveThread(std::atomic<bool> &program_is_running);
        int callReceiveFrom();
    };

} // namespace Api
} // namespace Kinova

#endif // __TRANSPORT_CLIENT_UDP_H__

