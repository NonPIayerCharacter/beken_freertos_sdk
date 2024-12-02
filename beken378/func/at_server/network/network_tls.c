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
 * TLS/DTLS network API
 */
#ifndef AUTH_WITH_NOTLS

int network_tls_init(ATNetwork *pNetwork)
{
    return ATSVR_RET_SUCCESS;
}

int network_tls_connect(ATNetwork *pNetwork)
{
    POINTER_SANITY_CHECK(pNetwork, ATSVR_ERR_INVAL);

    int ret = ATSVR_ERR_FAILURE;

    pNetwork->handle = (uintptr_t)HAL_TLS_Connect(&(pNetwork->ssl_connect_params), pNetwork->host, pNetwork->port);
	os_printf("network_tls_connect pNetwork->handle=%d\r\n",pNetwork->handle);
	if (pNetwork->handle != 0) {
        ret = ATSVR_RET_SUCCESS;
    }

    return ret;
}

int network_tls_read(ATNetwork *pNetwork, unsigned char *data, size_t datalen, uint32_t timeout_ms, size_t *read_len)
{
    POINTER_SANITY_CHECK(pNetwork, ATSVR_ERR_INVAL);

    int rc = HAL_TLS_Read(pNetwork->handle, data, datalen, timeout_ms, read_len);

    return rc;
}

int network_tls_write(ATNetwork *pNetwork, unsigned char *data, size_t datalen, uint32_t timeout_ms, size_t *written_len)
{
    POINTER_SANITY_CHECK(pNetwork, ATSVR_ERR_INVAL);

    int rc = HAL_TLS_Write(pNetwork->handle, data, datalen, timeout_ms, written_len);

    return rc;
}

void network_tls_disconnect(ATNetwork *pNetwork)
{
    POINTER_SANITY_CHECK_RTN(pNetwork);

    HAL_TLS_Disconnect(pNetwork->handle);
    pNetwork->handle = 0;
}



int network_dtls_init(ATNetwork *pNetwork)
{
    return ATSVR_RET_SUCCESS;
}

int network_dtls_read(ATNetwork *pNetwork, unsigned char *data, size_t datalen, uint32_t timeout_ms, size_t *read_len)
{
    POINTER_SANITY_CHECK(pNetwork, ATSVR_ERR_INVAL);

    return HAL_DTLS_Read(pNetwork->handle, data, datalen, timeout_ms, read_len);
}

int network_dtls_write(ATNetwork *pNetwork, unsigned char *data, size_t datalen, uint32_t timeout_ms, size_t *written_len)
{
    POINTER_SANITY_CHECK(pNetwork, ATSVR_ERR_INVAL);

    return HAL_DTLS_Write(pNetwork->handle, data, datalen, written_len);
}

void network_dtls_disconnect(ATNetwork *pNetwork)
{
    POINTER_SANITY_CHECK_RTN(pNetwork);

    HAL_DTLS_Disconnect(pNetwork->handle);
    pNetwork->handle = 0;

    return;
}

int network_dtls_connect(ATNetwork *pNetwork)
{
    POINTER_SANITY_CHECK(pNetwork, ATSVR_ERR_INVAL);

    int ret = ATSVR_ERR_FAILURE;

    pNetwork->handle = (uintptr_t)HAL_DTLS_Connect(&(pNetwork->ssl_connect_params), pNetwork->host, pNetwork->port);
    if (pNetwork->handle != 0) {
        ret = ATSVR_RET_SUCCESS;
    }

    return ret;
}

#endif
