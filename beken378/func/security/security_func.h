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

#define USE_SEC_TEST_CMD 0

extern int hal_sha256_self_test( int  );

extern void *aes_encrypt_init(const u8 *key, size_t len);
extern void aes_encrypt(void *ctx, const u8 *plain, u8 *crypt);
extern void aes_encrypt_deinit(void *ctx);
extern void *aes_decrypt_init(const u8 *key, size_t len);
extern void aes_decrypt(void *ctx, const u8 *crypt, u8 *plain);
extern void aes_decrypt_deinit(void *ctx);
