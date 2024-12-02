#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mem_pub.h"
#include "lite-utils.h"
#include "BkDriverFlash.h"
#include "HAL_Device_bk_flash.h"
#include "qcloud_iot_export.h"

static char version_json[INFO_FILE_MAX_LEN];
static OTA_DATA_ST ota_data_ptr;


void *get_ota_para_data()
{
    return &ota_data_ptr;
}

void ota_flash_init(uint32_t start_addr)
{

    memset(&ota_data_ptr,0,sizeof(OTA_DATA_ST));
    bk_logic_partition_t *pt = bk_flash_get_info(BK_PARTITION_OTA);


    ota_data_ptr.ota_partition_start_addr=pt->partition_start_addr;
    ota_data_ptr.partition_length=pt->partition_length;

    if(start_addr > ota_data_ptr.ota_partition_start_addr)
        ota_data_ptr.flash_address=start_addr;
    else
        ota_data_ptr.flash_address = ota_data_ptr.ota_partition_start_addr;

    bk_flash_enable_security(FLASH_PROTECT_NONE);
    Log_d("ota_flash_init done");
}

void ota_flash_erase_ota_rbl_header(void)
{
    bk_flash_erase(BK_PARTITION_OTA,0,4096);
}

void ota_flash_deinit(void)
{
    bk_flash_enable_security(FLASH_UNPROTECT_LAST_BLOCK);
    Log_d("ota_flash_deinit\r\n");
}


int ota_write_to_flash(uint8_t *src, unsigned len)
{
    int ret;
    if(len<=0)
    {
        return -1;
    }

    char *buffer_swap = malloc(OTA_BUF_LEN);
    if (buffer_swap == NULL)
    {
        Log_e("No memory for http ota!");
        return -1;
    }

    if(ota_data_ptr.flash_address % 0x1000 == 0)
    {
        bk_flash_erase(BK_PARTITION_OTA,ota_data_ptr.flash_address-ota_data_ptr.ota_partition_start_addr,4096);
    }

    if(((u32)ota_data_ptr.flash_address >= ota_data_ptr.ota_partition_start_addr)
        && (((u32)ota_data_ptr.flash_address + len) < (ota_data_ptr.ota_partition_start_addr + ota_data_ptr.partition_length)))
    {

        bk_flash_write(BK_PARTITION_OTA,ota_data_ptr.flash_address-ota_data_ptr.ota_partition_start_addr,(uint8_t *)src,len);

        if(buffer_swap)
        {

            bk_flash_read(BK_PARTITION_OTA,ota_data_ptr.flash_address-ota_data_ptr.ota_partition_start_addr,(uint8_t *)buffer_swap,len);
            if(os_memcmp(src , buffer_swap, len ))
            {
                Log_e("wr flash write err\n");
                ret=-1;
                goto __exit;
            }
        }

        ota_data_ptr.flash_address += len;
        Log_i("ad %x.\r\n",ota_data_ptr.flash_address);
    }
    else
    {
        Log_d("flash_address:%x,ota_partition_start_addr:%x,partition_length:%d,len:%d",ota_data_ptr.flash_address,ota_data_ptr.ota_partition_start_addr,ota_data_ptr.partition_length,len);
    }

__exit:
    free(buffer_swap);
    return ret;
}



static void ota_get_fw_version_parameter(uint8_t *param,int len)
{
     bk_flash_read(BK_PARTITION_QCLOUD_OTA_PARAM,0,(uint8_t*)param,len);
}


void ota_get_local_fw_version(char **ver, char **md5, char **size,char **start_addr,char **preVer)
{
#ifdef FLASH_VERSION_INFO
    ota_get_fw_version_parameter((uint8_t *)version_json,sizeof(version_json));
#else
    strcpy(version_json,DEFAULT_VERSION_INFO);
#endif
    *ver = LITE_json_value_of(KEY_VER, version_json);
    *md5 = LITE_json_value_of(KEY_MD5, version_json);
    *size = LITE_json_value_of(KEY_SIZE, version_json);
    *start_addr=LITE_json_value_of(KEY_START_ADDR, version_json);
    *preVer  = LITE_json_value_of(KEY_PREVER, version_json);

#ifdef FLASH_VERSION_INFO
    if((NULL == *ver) && (NULL == *preVer))
    {
        strcpy(version_json,DEFAULT_VERSION_INFO);
        bk_flash_erase(BK_PARTITION_QCLOUD_OTA_PARAM,0,FLASH_SECTOR_SIZE);
        bk_flash_write(BK_PARTITION_QCLOUD_OTA_PARAM,0,(uint8_t*)version_json,sizeof(version_json));
    }
#endif

}



/* update local firmware info for resuming download from break point */
int ota_update_local_fw_info(const char *version, const char *preVer, const char *md5, uint32_t downloadedSize,uint32_t start_addr,char *ota_state)
{

    int ret = QCLOUD_RET_SUCCESS;
    memset(version_json, 0, INFO_FILE_MAX_LEN);
    HAL_Snprintf(version_json, INFO_FILE_MAX_LEN, "{\"%s\":\"%s\", \"%s\":\"%s\",\"%s\":%d,\"%s\":\"%s\",\"%s\":%d,\"%s\":\"%s\"}", \
                 KEY_VER, version, KEY_MD5, md5, KEY_SIZE, downloadedSize, \
                 KEY_PREVER, (NULL == preVer) ? "1.0.0" : preVer,KEY_START_ADDR,start_addr,KEY_OTA_STATE,ota_state);

    Log_d("update_local_fw_info:%s",version_json);

#ifdef FLASH_VERSION_INFO
    bk_flash_erase(BK_PARTITION_QCLOUD_OTA_PARAM,0,FLASH_SECTOR_SIZE);
    bk_flash_write(BK_PARTITION_QCLOUD_OTA_PARAM,0,(uint8_t*)version_json,sizeof(version_json));
#endif
    return ret;
}

/* calculate left MD5 for resuming download from break point */
int ota_cal_exist_fw_md5(void *h_ota,uint32_t last_start_addr)
{

    char *buff=NULL;
    size_t rlen;
    int Ret = QCLOUD_RET_SUCCESS;
    size_t size=last_start_addr-ota_data_ptr.ota_partition_start_addr;
    uint32_t cal_start_addr=ota_data_ptr.ota_partition_start_addr;

    if(last_start_addr<=ota_data_ptr.ota_partition_start_addr)
    {
        return QCLOUD_ERR_FAILURE;
    }

    buff=malloc(CAL_BUFF_LEN);
    memset(buff,0,CAL_BUFF_LEN);
    while (size > 0)
    {
        rlen = (size > CAL_BUFF_LEN) ? CAL_BUFF_LEN : size;
        bk_flash_read(BK_PARTITION_OTA,cal_start_addr-ota_data_ptr.ota_partition_start_addr,(uint8_t*)buff,rlen);
        cal_start_addr+=rlen;
        IOT_OTA_UpdateClientMd5(h_ota, buff, rlen);
        size -= rlen;
    }
    free(buff);
    return Ret;

}

void qcloud_write_triple_info_to_flash(sTripleInfo *info)
{
    info->magic_header=TRIPLE_VAIL_MAGIC_INFO;
    bk_flash_enable_security(FLASH_PROTECT_NONE);
    bk_flash_erase(BK_PARTITION_TRIPLE_PARAM,0,4096);
    bk_flash_write(BK_PARTITION_TRIPLE_PARAM, 0, (uint8_t *)info,sizeof(sTripleInfo));
    bk_flash_enable_security(FLASH_UNPROTECT_LAST_BLOCK);
}

uint8_t qcloud_read_triple_info_from_flash(sTripleInfo *info)
{
    bk_flash_read(BK_PARTITION_TRIPLE_PARAM,0,(uint8_t *)info,sizeof(sTripleInfo));
    if(info->magic_header!=TRIPLE_VAIL_MAGIC_INFO)
        return -1;
    return 0;
}

#if QCLOUD_AT_CFG
#include "qcloud_at_flash.h"

int ota_load_fw_info(sOTAFirmwareInfo *fw_info)
{
    int ret=0;
    char *preVer = NULL,*ver=NULL,*md5=NULL,*size=NULL,*ota_state=NULL;
    ota_get_fw_version_parameter((uint8_t *)version_json,sizeof(version_json));
    ver = LITE_json_value_of(KEY_VER, version_json);
    md5 = LITE_json_value_of(KEY_MD5, version_json);
    size = LITE_json_value_of(KEY_SIZE, version_json);
    preVer  = LITE_json_value_of(KEY_PREVER, version_json);
    ota_state=LITE_json_value_of(KEY_OTA_STATE, version_json);


    if ( (NULL != ver) && (0 == strcmp(ota_state, "finished")) )
    {
        fw_info->fw_state=OTA_FW_MCU_VALID;
        memcpy(fw_info->fw_version,ver,strlen(ver));
        memcpy(fw_info->fw_md5,md5,strlen(md5));
        fw_info->fw_size=atoi(size);
        fw_info->fw_max_size_of_module=AT_OTA_FLASH_MAX_SIZE;
    }
    else
    {
        fw_info->fw_state=OTA_FW_MCU_DOWNLOADING;
        ret=-1;
    }

    if (NULL != ver)
    {
        HAL_Free(ver);
        ver = NULL;
    }
    if (NULL != md5)
    {
        HAL_Free(md5);
        md5 = NULL;
    }
    if (NULL != size)
    {
        HAL_Free(size);
        size = NULL;
    }
    if (NULL != preVer)
    {
        HAL_Free(preVer);
        preVer = NULL;
    }
    if (NULL != ota_state)
    {
        HAL_Free(ota_state);
        ota_state = NULL;
    }
    return ret;
}

int ota_read_fw_from_flash(char *buf, int alloc_bytes, int g_fw_fetched_bytes, int g_fw_size_bytes)
{
    int ret;
    ret=bk_flash_read(BK_PARTITION_OTA,g_fw_fetched_bytes,(uint8_t *)buf,alloc_bytes);
    return ret;
}

void ota_clear_fw_info(void)
{
    bk_flash_erase(BK_PARTITION_QCLOUD_OTA_PARAM,0,4096);
}

#endif