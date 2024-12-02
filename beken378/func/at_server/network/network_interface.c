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

#ifdef __cplusplus
extern "C" {
#endif

#include "at_svr_opts.h"

#include "network_interface.h"

#include "atsvr_error.h"
#include "atsvr_param_check.h"

int is_network_connected(ATNetwork *pNetwork)
{
    return pNetwork->handle;
}

int interace_network_init(ATNetwork *pNetwork)
{
    POINTER_SANITY_CHECK(pNetwork, ATSVR_ERR_INVAL);

    switch (pNetwork->type) {
        case NETWORK_TCP:
            pNetwork->init         = network_tcp_init;
            pNetwork->netconnect      = network_tcp_connect;
            pNetwork->netread         = network_tcp_read;
            pNetwork->netwrite        = network_tcp_write;
            pNetwork->disconnect   = network_tcp_disconnect;
            pNetwork->is_connected = is_network_connected;
            pNetwork->handle       = 0;

            break;

        case NETWORK_TLS:
            pNetwork->init         = network_tls_init;
            pNetwork->netconnect      = network_tls_connect;
            pNetwork->netread         = network_tls_read;
            pNetwork->netwrite        = network_tls_write;
            pNetwork->disconnect   = network_tls_disconnect;
            pNetwork->is_connected = is_network_connected;
            pNetwork->handle       = 0;
            break;


        case NETWORK_UDP:
            pNetwork->init         = network_udp_init;
            pNetwork->netconnect      = network_udp_connect;
            pNetwork->netread         = network_udp_read;
            pNetwork->netwrite        = network_udp_write;
            pNetwork->disconnect   = network_udp_disconnect;
            pNetwork->is_connected = is_network_connected;
            pNetwork->handle       = 0;
            break;

        case NETWORK_DTLS:
            pNetwork->init         = network_dtls_init;
            pNetwork->netconnect      = network_dtls_connect;
            pNetwork->netread         = network_dtls_read;
            pNetwork->netwrite        = network_dtls_write;
            pNetwork->disconnect   = network_dtls_disconnect;
            pNetwork->is_connected = is_network_connected;
            pNetwork->handle       = 0;
            break;

        default:
            ATSVRLOG("unknown network type: %d", pNetwork->type);
            return ATSVR_ERR_INVAL;
    }
    return pNetwork->init(pNetwork);
}

#ifdef __cplusplus
}
#endif
