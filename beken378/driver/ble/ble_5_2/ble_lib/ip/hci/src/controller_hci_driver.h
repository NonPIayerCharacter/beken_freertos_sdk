// Copyright 2020-2021 Beken
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

#pragma once
#include <stdint.h>
/*
 * INCLUDE FILES
 ****************************************************************************************
 */



/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

typedef struct {
    void (*notify_host_recv_cb)(void *data, uint16_t len);
} controller_hci_driver_callbacks_t;

typedef struct {
    void (*init)(controller_hci_driver_callbacks_t *cb);
    void (*deinit)(void);
    int (*hci_data_send)(uint8_t type, uint8_t *data, uint16_t len);
} controller_hci_driver_t;


/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

const controller_hci_driver_t *controller_hci_driver_get_interface(void);

