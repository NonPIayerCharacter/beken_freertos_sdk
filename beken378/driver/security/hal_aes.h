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

#ifndef _BK_AES_H_
#define _BK_AES_H_

#if (CFG_SOC_NAME == SOC_BK7221U)

enum AES_MODE
{
    AES128      = 0,
    AES256      = 1,
    AES192      = 2
};

enum AES_ENCODE
{
    DECODE      = 0,
    ENCODE      = 1
};

void hal_aes_init(void *ctx);
int hal_aes_setkey_dec(void *ctx, const unsigned char *key,
                       unsigned int keybits);
int hal_aes_setkey_enc(void *ctx, const unsigned char *key,
                       unsigned int keybits);
int hal_aes_crypt_ecb(void *ctx,
                      int mode,
                      const unsigned char input[16],
                      unsigned char output[16]);

void hal_aes_free(void *ctx);
#endif
#endif

