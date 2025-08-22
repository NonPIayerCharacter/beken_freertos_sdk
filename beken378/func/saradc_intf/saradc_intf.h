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

#ifndef __SARADC_INTF_H__
#define __SARADC_INTF_H__

#define TADC_DEBUG_PRTF                     0

#define TADC_FATAL_PRINTF                   os_printf
#define TADC_WARNING_PRINTF                 os_null_printf

#if TADC_DEBUG_PRTF
#define TADC_PRINTF                         os_printf
#else
#define TADC_PRINTF(...)
#endif //TADC_DEBUG_PRTF

#define TURING_ADC_SCAN_INTERVALV           (10)  // ms

typedef void (*adc_obj_callback)(int new_mv, void *user_data);

typedef struct _adc_obj_ {
    void *user_data;
    UINT32 channel;
    adc_obj_callback cb;
    struct _adc_obj_ *next;
} ADC_OBJ;

void adc_obj_init(ADC_OBJ* handle, adc_obj_callback cb, UINT32 channel, void *user_data);
int adc_obj_start(ADC_OBJ* handle);
void adc_obj_stop(ADC_OBJ* handle);

void saradc_work_create(void);

#endif
