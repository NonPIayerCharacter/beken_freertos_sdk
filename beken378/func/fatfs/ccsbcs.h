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

/*
 * ccsbcs.h
 *
 *  Created on: 2017-5-24
 *      Author: bo.wang
 */

#ifndef CCSBCS_H_
#define CCSBCS_H_
#include "integer.h"
#include "ffconf.h"

#if _CODE_PAGE != 936
WCHAR ff_convert (	/* Converted character, Returns zero on error */
    WCHAR	chr,	/* Character code to be converted */
    UINT	dir		/* 0: Unicode to OEM code, 1: OEM code to Unicode */
);

WCHAR ff_wtoupper (	/* Returns upper converted character */
    WCHAR chr		/* Unicode character to be upper converted (BMP only) */
);

#endif

#endif /* CCSBCS_H_ */
