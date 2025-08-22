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

#ifndef _RX_SENSITIVITY_H_
#define _RX_SENSITIVITY_H_

/*******************************************************************************
* Function Declarations
*******************************************************************************/
extern void rs_test(void);
extern void rx_get_rx_result_begin(void);
extern void rx_get_rx_result_end(void);
extern UINT32 rs_set_channel(UINT32 channel_id);
extern UINT32 rs_set_mode(UINT32 mode);

extern void rx_clean_rx_statistic_result(void);
extern UINT32 rx_get_rx20M_statistic_result(void);
extern UINT32 rx_get_rx40M_statistic_result(void);

extern void rx_clean_ble_rx_result(void);
extern void rx_start_ble_rx_counting();
extern void rs_ble_test_start(UINT32 channel);
extern void rs_ble_test_stop(void);
extern void rx_get_ble_rx_result(void);


#endif //_RX_SENSITIVITY_H_
// eof

