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

#ifndef _BK_SPI_PSRAM_H_
#define _BK_SPI_PSRAM_H_

int32_t spi_psram_init(void);
int32_t spi_psram_burst_set(uint32_t burst_size);
int32_t spi_psram_read_id(uint8_t id[12]);
uint32_t spi_psram_read(uint32_t addr, uint8_t* buffer, uint32_t size);
uint32_t spi_psram_write(uint32_t addr, uint8_t* buffer, uint32_t size);

#endif //_BK_SPI_PSRAM_H_