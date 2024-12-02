#include <stdio.h>
#include "drv_model_pub.h"
#include "flash_pub.h"
#include "drv_model.h"
#include "_at_svr_opts.h"
#include "BkDriverFlash.h"
#include "atsvr_comm.h"
#include "mem_pub.h"
#include "nvds.h"

extern int log_enable();
extern int log_disable();

int write_env_to_flash(AT_ENV_TAG tag, int datalen ,uint8* buf)
{
	int state = 0;
	bk_flash_enable_security(FLASH_PROTECT_NONE);
	bk_flash_erase(BK_PARTITION_USR_CONFIG,tag,datalen);
	state = bk_flash_write(BK_PARTITION_USR_CONFIG,tag,buf,datalen);
	bk_flash_enable_security(FLASH_UNPROTECT_LAST_BLOCK);

	return state;
}

int read_env_from_flash(AT_ENV_TAG tag, int len ,uint8* buf)
{
	return bk_flash_read(BK_PARTITION_USR_CONFIG,tag,buf,len);
}

void log_output_state(int flag)
{
	if(flag)
	{
		log_enable();
	}
	else
	{
		log_disable();
	}
	return ;
}


void *at_malloc(unsigned int size)
{
	return os_malloc(size);
}

void at_free(void *p)
{
	os_free(p);
}
