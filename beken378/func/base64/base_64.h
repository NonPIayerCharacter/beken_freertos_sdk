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

#ifndef _BSAE64_ENC_H_
#define _BSAE64_ENC_H_

#define CFG_USE_BASE64        1

#ifdef CFG_USE_BASE64
unsigned int base64_calc_encode_length(unsigned int src_Len);
unsigned char base64_encode(const unsigned char *src, int len,
                            int *out_len, unsigned char *out);

unsigned int base64_calc_decode_length(const unsigned char *src, unsigned int src_Len);
unsigned char base64_decode(const unsigned char *src, int len,
                            int *out_len, unsigned char *out);
#endif /*CFG_USE_BASE64*/
#endif /*_base64_Enc_H_*/

