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

#ifndef _FUSB_H_
#define _FUSB_H_

#define FUSB_DEBUG

#ifdef FUSB_DEBUG
#define FUSB_PRT      os_printf
#define FUSB_WARN     warning_prf
#define FUSB_FATAL    fatal_prf
#else
#define FUSB_PRT      null_prf
#define FUSB_WARN     null_prf
#define FUSB_FATAL    null_prf
#endif

#ifdef FMSC_TEST
extern void fmsc_test_init(void);
extern void fmsc_fiddle_process(void);
#elif defined(FUVC_TEST)
extern void fuvc_test_init(uint8_t);
extern void fuvc_notify_uvc_configed(void);
extern void fuvc_fiddle_rx_vs(void);
#endif

#endif
// eof

