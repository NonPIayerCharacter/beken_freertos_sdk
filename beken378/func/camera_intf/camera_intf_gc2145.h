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

#ifndef __CAMERA_INTF_GC2145_H__
#define __CAMERA_INTF_GC2145_H__

// interface
int gc2145_probe(void);
int gc2145_reset(void);
void gc2145_init(int mode);
void gc2145_grayscale(int mode);
unsigned char gc2145_fps(unsigned char fps);
void gc2145_user_reg_set(unsigned char *user_reg, unsigned char num);

#endif // __CAMERA_INTF_GC2145_H__
