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

#ifndef _VAD_CORE_H_
#define _VAD_CORE_H_

#define FRAME_TYPE_SPEECH                  (1)
#define FRAME_TYPE_NOISE                   (0)

/*threshold value of power*/
//#ifdef TEST7251_
#define POWER_THRESHOLD_VAL 89925  //26510
//#endif



extern int vad(short samples[], int len);

#endif // _VAD_CORE_H_
// eof
