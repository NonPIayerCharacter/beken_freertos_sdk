// Copyright 2015-2024 Beken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _SL_TLS_H_
#define _SL_TLS_H_


#include "tls_rtos.h"
#include "tls_client.h"
#include "tls_certificate.h"
#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif





#define DEFAULT_MBEDTLS_READ_BUFFER               (1024U)








extern MbedTLSSession * ssl_create(char *url,char *port);
extern int ssl_txdat_sender(MbedTLSSession *tls_session,int len,char *data);
extern int ssl_read_data(MbedTLSSession *session,unsigned char *msg,unsigned int mlen,unsigned int timeout_ms);
extern int ssl_close(MbedTLSSession *session);

#endif
