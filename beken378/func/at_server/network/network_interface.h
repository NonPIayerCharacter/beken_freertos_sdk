/*
 * Tencent is pleased to support the open source community by making IoT Hub
 available.
 * Copyright (C) 2016 THL A29 Limited, a Tencent company. All rights reserved.

 * Licensed under the MIT License (the "License"); you may not use this file
 except in
 * compliance with the License. You may obtain a copy of the License at
 * http://opensource.org/licenses/MIT

 * Unless required by applicable law or agreed to in writing, software
 distributed under the License is
 * distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 KIND,
 * either express or implied. See the License for the specific language
 governing permissions and
 * limitations under the License.
 *
 */

#ifndef _NETWORK_INTERFACE_H_
#define _NETWORK_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

#include "stdbool.h"
#include "rtos_pub.h"


#include "_at_svr_opts.h"

/*
 * Type of network interface
 */

typedef enum { NETWORK_TCP = 0, NETWORK_UDP = 1, NETWORK_TLS = 2, NETWORK_DTLS = 3 } NETWORK_TYPE;

#define LWIP_SOCKET_FD_SHIFT 3


/**
 * @brief Define structure for network stack
 */
typedef struct ATNetwork ATNetwork;


#ifndef AUTH_WITH_NOTLS

#ifndef MAX_SIZE_OF_CLIENT_ID
#define MAX_SIZE_OF_CLIENT_ID (80)
#endif
/**
 * @brief Define structure for TLS connection parameters
 *
 */
typedef struct {
    const char *ca_crt;
    uint16_t    ca_crt_len;

#ifdef AUTH_MODE_CERT
    /**
     * Device with certificate
     */
    const char *cert_file;  // public certificate file
    const char *key_file;   // pravite certificate file
#else
    /**
     * Device with PSK
     */
    const char *psk;                                // PSK string
    char        psk_id[MAX_SIZE_OF_CLIENT_ID + 1];  // PSK ID
#endif

    size_t psk_length;  // PSK length

    unsigned int timeout_ms;  // SSL handshake timeout in millisecond

} SSLConnectParams;

typedef SSLConnectParams TLSConnectParams;
#endif


typedef struct ATNetwork  ATNetwork;
/**
 * @brief Define structure for network stack
 *
 * Network init/connect/read/write/disconnect/state
 */
struct ATNetwork {
    int (*init)(ATNetwork *);

    int (*netconnect)(ATNetwork *);

    int (*netread)(ATNetwork *, unsigned char *, size_t, uint32_t, size_t *);

    int (*netwrite)(ATNetwork *, unsigned char *, size_t, uint32_t, size_t *);

    void (*disconnect)(ATNetwork *);

    int (*is_connected)(ATNetwork *);

    // connetion handle:
    // for non-AT: 0 = not connected, non-zero = connected
    // for AT: 0 = valid connection, MAX_UNSINGED_INT = invalid
    uintptr_t handle;

#ifndef AUTH_WITH_NOTLS
    SSLConnectParams ssl_connect_params;
#endif

    char            host[64];  // server address
    unsigned short  port;  // server port
    NETWORK_TYPE    type;
    int             keepalive;
    int             udpmode;
    char            localip[16];
    unsigned short  localport;
    int             is_server; //0: client 1:server
    int             threadrunflag;//
};

struct Timer{
    #if defined (__linux__) && defined (__GLIBC__)
        struct timeval end_time;
    #else
        uintptr_t end_time;
    #endif
};

typedef struct Timer Timer;



uint32_t HAL_GetTimeMs(void);



/* @brief Check if timer expires or not
*
* @param timer reference to timer
* @return true = expired, false = not expired yet

*/
bool  HAL_Timer_expired(Timer *timer);
/**
* @brief Set the countdown/expired value for the timer
* @param timer reference to timer
* @param timeout_ms countdown/expired value (unit: millisecond)
*/

void HAL_Timer_countdown_ms(Timer *timer, unsigned int timeout_ms);

/**
* @brief Set the countdown/expired value for the timer
* @param timer reference to timer
* @param timeout countdown/expired value (unit: second)
*/

void HAL_Timer_countdown(Timer *timer, unsigned int timeout);

/**
* @brief Check the remain time of the timer
* @param timer reference to timer
* @return if expired, or the left time in millisecond
*/

int HAL_Timer_remain(Timer *timer) ;

/* @brief Init the timer
* @param timer reference to timer
*/

void HAL_Timer_init(Timer *timer);

#define TIME_FORMAT_STR_LEN (20)

/**
* @brief Get local time in format: %y-%m-%d %H:%M:%S
* @return string of formatted time
*/

char *HAL_Timer_current(char *time_str);

/**
* @brief Get timestamp in second
* @return timestamp in second
*/
long HAL_Timer_current_sec(void);



typedef void (*ThreadRunFunc)(void *arg);

typedef struct ThreadParams {
    char      thread_name[64];
    beken_thread_t   thread_id;
    ThreadRunFunc thread_func;
    void *        user_arg;
    uint16_t      priority;
    uint32_t      stack_size;
} ThreadParams,*PThreadParams;

/**
 * @brief Create a thread/task
 *
 * @param params    thread parameters
 * @return 0 when success, or error code otherwise
 */
int HAL_ThreadCreate(ThreadParams *params);

/**
 * @brief Destroy a thread/task
 *
 * @return QCLOUD_RET_SUCCESS for success, or err code for failure
 */
int HAL_ThreadDestroy(void *thread_t);


/**
 * @brief Create a thread/task
 *
 * @param params    thread parameters
 * @return 0 when success, or error code otherwise
 */
int HAL_ThreadCreate(ThreadParams *params);

/**
 * @brief Destroy a thread/task
 *
 * @return QCLOUD_RET_SUCCESS for success, or err code for failure
 */
int HAL_ThreadDestroy(void *thread_t);

/**
 * @brief create semaphore
 *
 * @return a valid semaphore handle when success, or NULL otherwise
 */
void *HAL_SemaphoreCreate(void);

/**
 * @brief Destroy semaphore
 * @param sem   semaphore handle
 */
void HAL_SemaphoreDestroy(void *sem);

/**
 * @brief Post semaphore
 * @param sem   semaphore handle
 */
void HAL_SemaphorePost(void *sem);

/**
 * @brief Wait for semaphore
 * @param sem           semaphore handle
 * @param timeout_ms    waiting timeout value (unit: ms)
 * @return QCLOUD_RET_SUCCESS for success, or err code for failure
 */
int HAL_SemaphoreWait(void *sem, uint32_t timeout_ms);

/**
 * @brief Create mutex
 *
 * @return a valid mutex handle when success, or NULL otherwise
 */
void *HAL_MutexCreate(void);

/**
 * @brief Destroy mutex
 *
 * @param mutex     mutex handle
 */
void HAL_MutexDestroy(void *mutex);

/**
 * @brief Lock a mutex in blocking way
 *
 * @param mutex     mutex handle
 */
void HAL_MutexLock(void *mutex);

/**
 * @brief Lock a mutex in non-blocking way
 *
 * @param mutex     mutex handle
 * @return 0 for success, or err code for failure
 */
int HAL_MutexTryLock(void *mutex);

/**
 * @brief Unlock/release mutex
 *
 * @param mutex     mutex handle
 */
void HAL_MutexUnlock(void *mutex);




/* return the handle */
int is_network_connected(ATNetwork *pNetwork);

/* network stack API */
#ifdef AT_TCP_ENABLED

#define AT_NO_CONNECTED_FD 0xffffffff
#endif


int interace_network_init(ATNetwork *pNetwork);


int  network_at_tcp_read(ATNetwork *pNetwork, unsigned char *data, size_t datalen, uint32_t timeout_ms, size_t *read_len);
int  network_at_tcp_write(ATNetwork *pNetwork, unsigned char *data, size_t datalen, uint32_t timeout_ms,
                          size_t *written_len);
void network_at_tcp_disconnect(ATNetwork *pNetwork);
int  network_at_tcp_connect(ATNetwork *pNetwork);
int  network_at_tcp_init(ATNetwork *pNetwork);



int network_tcp_read(ATNetwork *pNetwork, unsigned char *data, size_t datalen, uint32_t timeout_ms, size_t *read_len);
int network_tcp_write(ATNetwork *pNetwork, unsigned char *data, size_t datalen, uint32_t timeout_ms, size_t *written_len);
void network_tcp_disconnect(ATNetwork *pNetwork);
int  network_tcp_connect(ATNetwork *pNetwork);
int  network_tcp_init(ATNetwork *pNetwork);



int network_tls_read(ATNetwork *pNetwork, unsigned char *data, size_t datalen, uint32_t timeout_ms, size_t *read_len);
int network_tls_write(ATNetwork *pNetwork, unsigned char *data, size_t datalen, uint32_t timeout_ms, size_t *written_len);
void network_tls_disconnect(ATNetwork *pNetwork);
int  network_tls_connect(ATNetwork *pNetwork);
int  network_tls_init(ATNetwork *pNetwork);



int network_udp_read(ATNetwork *pNetwork, unsigned char *data, size_t datalen, uint32_t timeout_ms, size_t *read_len);
int network_udp_write(ATNetwork *pNetwork, unsigned char *data, size_t datalen, uint32_t timeout_ms, size_t *written_len);
void network_udp_disconnect(ATNetwork *pNetwork);
int  network_udp_connect(ATNetwork *pNetwork);
int  network_udp_init(ATNetwork *pNetwork);

int  network_dtls_read(ATNetwork *pNetwork, unsigned char *data, size_t datalen, uint32_t timeout_ms, size_t *read_len);
int  network_dtls_write(ATNetwork *pNetwork, unsigned char *data, size_t datalen, uint32_t timeout_ms,
                        size_t *written_len);
void network_dtls_disconnect(ATNetwork *pNetwork);
int  network_dtls_connect(ATNetwork *pNetwork);
int  network_dtls_init(ATNetwork *pNetwork);



/**
 * @brief Setup TLS connection with server
 *
 * @param   pConnectParams reference to TLS connection parameters
 * @host    server address
 * @port    server port
 * @return  TLS connect handle when success, or 0 otherwise
 */

uintptr_t HAL_TLS_Connect(TLSConnectParams *pConnectParams, const char *host, int port);

/**
 * @brief Disconnect with TLS server and release resources
 *
 * @param handle TLS connect handle
 */
void HAL_TLS_Disconnect(uintptr_t handle);

/**
 * @brief Write data via TLS connection
 *
 * @param handle        TLS connect handle
 * @param data          source data to write
 * @param totalLen      length of data
 * @param timeout_ms    timeout value in millisecond
 * @param written_len   length of data written successfully
 * @return              ATSVR_RET_SUCCESS for success, or err code for failure
 */
int HAL_TLS_Write(uintptr_t handle, unsigned char *data, size_t totalLen, uint32_t timeout_ms, size_t *written_len);

/**
 * @brief Read data via TLS connection
 *
 * @param handle        TLS connect handle
 * @param data          destination data buffer where to put data
 * @param totalLen      length of data
 * @param timeout_ms    timeout value in millisecond
 * @param read_len      length of data read successfully
 * @return              ATSVR_RET_SUCCESS for success, or err code for failure
 */
int HAL_TLS_Read(uintptr_t handle, unsigned char *data, size_t totalLen, uint32_t timeout_ms, size_t *read_len);

/********** DTLS network **********/

typedef SSLConnectParams DTLSConnectParams;

/**
 * @brief Setup DTLS connection with server
 *
 * @param   pConnectParams reference to DTLS connection parameters
 * @host    server address
 * @port    server port
 * @return  DTLS connect handle when success, or 0 otherwise
 */
uintptr_t HAL_DTLS_Connect(DTLSConnectParams *pConnectParams, const char *host, int port);

/**
 * @brief Disconnect with DTLS server and release resources
 *
 * @param handle DTLS connect handle
 */
void HAL_DTLS_Disconnect(uintptr_t handle);

/**
 * @brief Write data via DTLS connection
 *
 * @param handle        DTLS connect handle
 * @param data          source data to write
 * @param totalLen      length of data
 * @param timeout_ms    timeout value in millisecond
 * @param written_len   length of data written successfully
 * @return              ATSVR_RET_SUCCESS for success, or err code for failure
 */
int HAL_DTLS_Write(uintptr_t handle, const unsigned char *data, size_t datalen, size_t *written_len);

/**
 * @brief Read data via DTLS connection
 *
 * @param handle        DTLS connect handle
 * @param data          destination data buffer where to put data
 * @param totalLen      length of data
 * @param timeout_ms    timeout value in millisecond
 * @param read_len      length of data read successfully
 * @return              ATSVR_RET_SUCCESS for success, or err code for failure
 */
int HAL_DTLS_Read(uintptr_t handle, unsigned char *data, size_t datalen, unsigned int timeout_ms, size_t *read_len);




/********** TCP network **********/
/**
 * @brief Setup TCP connection with server
 *
 * @host    server address
 * @port    server port
 * @return  TCP socket handle (value>0) when success, or 0 otherwise
 */
uintptr_t hal_tcp_connect(const char *host, uint32_t port);

/**
 * @brief Disconnect with server and release resource
 *
 * @param fd TCP Socket handle
 * @return  0 when success
 */
int hal_tcp_disconnect(uintptr_t fd);

/**
 * @brief Write data via TCP connection
 *
 * @param fd            TCP socket handle
 * @param data          source data to write
 * @param len           length of data
 * @param timeout_ms    timeout value in millisecond
 * @param written_len   length of data written successfully
 * @return              ATSVR_RET_SUCCESS for success, or err code for failure
 */
int hal_tcp_write(uintptr_t fd, const unsigned char *data, uint32_t len, uint32_t timeout_ms, size_t *written_len);

/**
 * @brief Read data via TCP connection
 *
 * @param fd            TCP socket handle
 * @param data          destination data buffer where to put data
 * @param len           length of data
 * @param timeout_ms    timeout value in millisecond
 * @param read_len      length of data read successfully
 * @return              ATSVR_RET_SUCCESS for success, or err code for failure
 */
int hal_tcp_read(uintptr_t fd, unsigned char *data, uint32_t len, uint32_t timeout_ms, size_t *read_len);

/**
 * @brief Read data via TCP connection
 *
 * @param fd            TCP socket handle
 * @param data          destination data buffer where to put data
 * @param len           length of data
 * @param timeout_ms    timeout value in millisecond
 * @param read_len      length of data read successfully
 * @return              ATSVR_RET_SUCCESS for success, or err code for failure
 */

int hal_tcp_read_noblock(uintptr_t fd, unsigned char *buf, uint32_t len, uint32_t timeout_ms, size_t *read_len);


/********** UDP network **********/
/**
 * @brief Setup UDP connection with server
 *
 * @host    server address
 * @port    server port
 * @return  UPD socket handle (value>0) when success, or 0 otherwise
 */
uintptr_t HAL_UDP_Connect(const char *host, unsigned int port);

/**
 * @brief Disconnect with server and release resource
 *
 * @param fd UDP Socket handle
 * @return  0 when success
 */
void HAL_UDP_Disconnect(uintptr_t fd);

/**
 * @brief Write data via UDP connection
 *
 * @param fd            UDP socket handle
 * @param data          source data to write
 * @param len           length of data
 * @return              length of data written when success, or err code for
 * failure
 */
int HAL_UDP_Write(uintptr_t fd, const unsigned char *data, unsigned int len);

/**
 * @brief Read data via UDP connection
 *
 * @param fd            UDP socket handle
 * @param data          destination data buffer where to put data
 * @param len           length of data
 * @return              length of data read when success, or err code for
 * failure
 */
int HAL_UDP_Read(uintptr_t fd, unsigned char *data, unsigned int len);

/**
 * @brief Read data via UDP connection
 *
 * @param fd            UDP socket handle
 * @param data          destination data buffer where to put data
 * @param len           length of data
 * @param timeout_ms    timeout value in millisecond
 * @return              length of data read when success, or err code for
 * failure
 */
int HAL_UDP_ReadTimeout(uintptr_t fd, unsigned char *p_data, unsigned int datalen, unsigned int timeout_ms);


int   HAL_UDP_CreateBind(const char *host, unsigned short port);
int   HAL_UDP_WriteTo(uintptr_t fd, const unsigned char *p_data, unsigned int datalen, char *host, unsigned short port);
void  HAL_UDP_Close(uintptr_t fd);
char *HAL_UDP_GetErrnoStr();

int HAL_UDP_GetErrno();
int HAL_UDP_ReadTimeoutPeerInfo(uintptr_t fd, unsigned char *p_data, unsigned int datalen, unsigned int timeout_ms,
                                char *recv_ip_addr, unsigned char recv_addr_len, unsigned short *recv_port);
#ifdef __cplusplus
}
#endif
#endif /* _NETWORK_INTERFACE_H_ */
