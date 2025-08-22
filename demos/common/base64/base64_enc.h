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

#ifdef _USE_BASE64_
unsigned int calc_Base64_encode_length(unsigned int src_Len);
unsigned char Base64_encode(const unsigned char *src, int len,
                            int *out_len, unsigned char *out);

unsigned int calc_Base64_decode_length(const unsigned char *src, unsigned int src_Len);
unsigned char Base64_decode(const unsigned char *src, int len,
                            int *out_len, unsigned char *out);
#endif /*_USE_BASE64_*/
#endif /*_base64_Enc_H_*/

