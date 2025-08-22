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

#ifndef _SEC_H_
#define _SEC_H_

#if (CFG_SOC_NAME == SOC_BK7221U)

typedef enum AES_RETURN_E
{
    AES_OK              = 0,
    AES_BUSY,
    AES_KEYLEN_ERR,
} AES_RETURN;

typedef void (*sec_done_callback)(void *param);
struct sec_done_des
{
    sec_done_callback callback;
    void  *param;
};

int security_aes_busy(void);
int security_aes_start(unsigned int mode);
int security_aes_init(sec_done_callback callback, void *param);
int security_aes_set_key(const unsigned char *key, unsigned int keybits);
int security_aes_set_block_data(const unsigned char *block_data);
int security_aes_get_result_data(unsigned char *pul_data);
#endif //(CFG_SOC_NAME == SOC_BK7221U)

#endif

