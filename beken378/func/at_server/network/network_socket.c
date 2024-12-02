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

#include "network_interface.h"
#include "atsvr_error.h"
#include "atsvr_param_check.h"

/*
 * TCP/UDP socket API
 */

int network_tcp_init(ATNetwork *pNetwork)
{
    return ATSVR_RET_SUCCESS;
}

int network_tcp_connect(ATNetwork *pNetwork)
{
    POINTER_SANITY_CHECK(pNetwork, ATSVR_ERR_INVAL);
    pNetwork->handle = hal_tcp_connect(pNetwork->host, pNetwork->port);
    if (0 == pNetwork->handle) {
        return -1;
    }

    return 0;
}

int network_tcp_read(ATNetwork *pNetwork, unsigned char *data, size_t datalen, uint32_t timeout_ms, size_t *read_len)
{
    POINTER_SANITY_CHECK(pNetwork, ATSVR_ERR_INVAL);

    int rc = 0;

    rc = hal_tcp_read_noblock(pNetwork->handle, data, (uint32_t)datalen, timeout_ms, read_len);

    return rc;
}

int network_tcp_write(ATNetwork *pNetwork, unsigned char *data, size_t datalen, uint32_t timeout_ms, size_t *written_len)
{
    POINTER_SANITY_CHECK(pNetwork, ATSVR_ERR_INVAL);

    int rc = 0;

    rc = hal_tcp_write(pNetwork->handle, data, datalen, timeout_ms, written_len);

    return rc;
}

void network_tcp_disconnect(ATNetwork *pNetwork)
{
    POINTER_SANITY_CHECK_RTN(pNetwork);

    if (0 == pNetwork->handle) {
        return;
    }

    hal_tcp_disconnect(pNetwork->handle);
    pNetwork->handle = 0;
    return;
}



int network_udp_init(ATNetwork *pNetwork)
{
    return ATSVR_RET_SUCCESS;
}

int network_udp_read(ATNetwork *pNetwork, unsigned char *data, size_t datalen, uint32_t timeout_ms, size_t *read_len)
{
    POINTER_SANITY_CHECK(pNetwork, ATSVR_ERR_INVAL);

    int ret = HAL_UDP_ReadTimeout(pNetwork->handle, data, datalen, timeout_ms);
    if (ret > 0) {
        *read_len = ret;
        ret       = 0;
    }

    return ret;
}
extern char	udp_remoteip[16];
extern int 	udp_port;

int network_udp_write(ATNetwork *pNetwork, unsigned char *data, size_t datalen, uint32_t timeout_ms, size_t *written_len)
{
    POINTER_SANITY_CHECK(pNetwork, ATSVR_ERR_INVAL);

    int ret  = HAL_UDP_WriteTo(pNetwork->handle, data, datalen, udp_remoteip, udp_port);

    //int ret = HAL_UDP_Write(pNetwork->handle, data, datalen);
    if (ret > 0) {
        *written_len = ret;
        ret          = 0;
    }

    return ret;
}

void network_udp_disconnect(ATNetwork *pNetwork)
{
    POINTER_SANITY_CHECK_RTN(pNetwork);

    HAL_UDP_Disconnect(pNetwork->handle);
    pNetwork->handle = 0;

    return;
}

int network_udp_connect(ATNetwork *pNetwork)
{
    POINTER_SANITY_CHECK(pNetwork, ATSVR_ERR_INVAL);

    if(pNetwork->is_server == 0){
        pNetwork->handle = HAL_UDP_Connect(pNetwork->host, pNetwork->port);
        if (0 == pNetwork->handle) {
            return -1;
        }
    }
    else {
        pNetwork->handle = HAL_UDP_CreateBind(pNetwork->localip, pNetwork->localport);
        if (0 == pNetwork->handle) {
            return -1;
        }
    }

    return 0;
}
