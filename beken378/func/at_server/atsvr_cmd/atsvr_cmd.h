#ifndef _ATSVR_CMD_H_
#define _ATSVR_CMD_H_



#include "atsvr_cmd_cfg.h"
#if CFG_USE_NETWORKING
extern void atsvr_cmd_init(void);
extern void atsvr_extern_cmd_init(void);
#endif
#endif

