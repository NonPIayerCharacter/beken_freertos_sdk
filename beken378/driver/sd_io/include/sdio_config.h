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

#pragma once

#define CONFIG_SYSTEM_CTRL                  1

#define CONFIG_SDIO_V2P0                    1
#define CONFIG_SDIO_SLAVE                   0
#define CONFIG_SDIO_HOST                    1
#define CONFIG_SDIO_HOST_DEFAULT_CLOCK_FREQ 1
#define CONFIG_SDIO_HOST_CLR_WRITE_INT      1
#define CONFIG_SDIO_V2P0                    1

#define CONFIG_SDIO_4LINE_EN                0
#define CONFIG_SDCARD_BUSWIDTH_4LINE        CONFIG_SDIO_4LINE_EN

#define CONFIG_SDIO_GDMA_EN                 0

// eof

