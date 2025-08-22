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

#ifndef _USB_MSD_H_
#define _USB_MSD_H_

#define CFG_ENABLE_SYC_OP                1

/*******************************************************************************
* Function Declarations
*******************************************************************************/
extern int usb_sw_init(void);
extern int usb_sw_uninit(void);

uint32_t get_HfiMedium_blksize(void);
uint32_t get_HfiMedium_size(void);

#ifdef CFG_ENABLE_SYC_OP
extern uint32_t MUSB_HfiRead_sync( uint32_t first_block, uint32_t block_num, uint8_t *dest);
extern uint32_t MUSB_HfiWrite_sync( uint32_t first_block, uint32_t block_num, uint8_t *dest);
#endif // CFG_ENABLE_SYC_OP

#endif // _USB_MSD_H_

// EOF
