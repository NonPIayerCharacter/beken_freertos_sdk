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

extern int spi_flash_init(UINT32 rate);
extern void spi_flash_deinit(void);
extern UINT32 spi_flash_read_id(void);
extern int spi_flash_read(UINT32 addr, UINT32 size, UINT8 *dst);
extern int spi_flash_write(UINT32 addr, UINT32 size, UINT8 *src);
extern int spi_flash_erase(UINT32 addr, UINT32 size);
extern void spi_flash_protect(void);
extern void spi_flash_unprotect(void);
extern UINT16 spi_flash_read_status(void);
//eof

