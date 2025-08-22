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

#ifndef __VOICE_TRANS_H__
#define __VOICE_TRANS_H__

typedef int (*video_transfer_send_func)(UINT8 *data, UINT32 len);

UINT32 tvoice_transfer_init(video_transfer_send_func send_func);
UINT32 tvoice_transfer_deinit(void);

#endif // __VOICE_TRANS_H__