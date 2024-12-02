/**
 ****************************************************************************************
 *
 * @file mesh_general_api.c
 *
 * @brief mesh Application Module general api entry point
 *
 * @auth  gang.cheng
 *
 * @date  2019.11.28
 *
 * Copyright (C) Beken 2009-2020
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

 /*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <string.h>
#include "mesh_general_api.h"                // Bracese Application Module Definitions
#include "app_mm_msg.h"                // Bracese Application Module Definitions
#include "m_api.h"
#include "mal_int.h"
#include "lld_adv_test.h"
#include "application.h"                     // Application Definitions
#include "app_task.h"                // application task definitions
#include "common_bt.h"
#include "prf_types.h"               // Profile common types definition
#include "architect.h"                    // Platform Definitions
#include "prf.h"
#include "lld_evt.h"
#include "ble_uart.h"
#include "mesh_api_msg.h"
#include "mal.h"
#include "m_bcn.h"
#include "m_prov_int.h"     // Mesh Provisioning Internal Defines

#include "user_config.h"

uint8_t g_dev_key[MESH_KEY_LEN] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0f, 0x10};
uint8_t g_net_key[MESH_KEY_LEN] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xff, 0x11};
uint16_t g_net_key_id = 0;
uint8_t g_app_key[MESH_KEY_LEN] = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1f, 0x11};
uint16_t g_app_key_id = 0;

void app_set_dev_key_param(uint8_t *p_dev_key)
{
	if(p_dev_key != NULL)
	{
		memcpy(g_dev_key, p_dev_key, MESH_KEY_LEN);
	}
}

void app_set_net_key_param(uint8_t *p_net_key, uint16 key_id)
{
	if(p_net_key != NULL)
	{
		memcpy(g_net_key, p_net_key, MESH_KEY_LEN);
		g_net_key_id = key_id;
	}
}

void app_set_app_key_param(uint8_t *p_app_key, uint16 key_id)
{
	if(p_app_key != NULL)
	{
		memcpy(g_app_key, p_app_key, MESH_KEY_LEN);
		g_app_key_id = key_id;
	}
}

void app_mesh_enable(void)
{

    MESH_APP_PRINT_DEBUG("app_mesh_enable,msgid:%x\r\n", MESH_API_CMD);

#if MAC78da07bcd71b
    MESH_APP_PRINT_DEBUG("MAC78da07bcd71b\r\n");
#endif

#if MAC78da07bcd71c
    MESH_APP_PRINT_DEBUG("MAC78da07bcd71c\r\n");
#endif

#if MAC78da07bcd71d
    MESH_APP_PRINT_DEBUG("MAC78da07bcd71d\r\n");
#endif
    mesh_api_cmd_t *cmd = KERNEL_MSG_ALLOC(MESH_API_CMD, prf_get_task_from_id(TASK_BLE_ID_MESH), TASK_BLE_APP, mesh_api_cmd);
    //
    cmd->cmd_code = M_API_ENABLE;
    //
    kernel_msg_send(cmd);//mal_app_id_get()

    MESH_APP_PRINT_DEBUG("app_mesh_enable send msgid:0x%x,code:0x%x,dst:0x%x\r\n", MESH_API_CMD, M_API_ENABLE, prf_get_task_from_id(TASK_BLE_ID_MESH));
}

void app_mesh_disable(void)
{

    MESH_APP_PRINT_DEBUG("app_mesh_disable,msgid:%x\r\n", MESH_API_CMD);

    mesh_api_cmd_t *cmd = KERNEL_MSG_ALLOC(MESH_API_CMD, prf_get_task_from_id(TASK_BLE_ID_MESH), TASK_BLE_APP, mesh_api_cmd);
    //
    cmd->cmd_code = M_API_DISABLE;
    //
    kernel_msg_send(cmd);//mal_app_id_get()

    MESH_APP_PRINT_DEBUG("app_mesh_disable send msgid:0x%x,code:0x%x,dst:0x%x\r\n", MESH_API_CMD, M_API_ENABLE, prf_get_task_from_id(TASK_BLE_ID_MESH));
}

void app_store_mesh_info(void)
{
    MESH_APP_PRINT_DEBUG("app_store_mesh_info send\r\n");
    m_api_storage_load_cmd_t *cmd = KERNEL_MSG_ALLOC(MESH_API_CMD, prf_get_task_from_id(TASK_BLE_ID_MESH), TASK_BLE_APP, m_api_storage_load_cmd); //

    cmd->cmd_code = M_API_STORAGE_LOAD;
    cmd->length = 0;
    kernel_msg_send(cmd);
}

uint8_t app_mesh_start_unpb_adv(void)
{
    if (kernel_state_get(TASK_BLE_APP) == APPM_READY)
    {
        m_api_cmd_t *cmd = KERNEL_MSG_ALLOC(MESH_API_CMD, prf_get_task_from_id(TASK_BLE_ID_MESH), TASK_BLE_APP, m_api_cmd); //

        cmd->cmd_code = M_API_START_UNPROV;
        kernel_msg_send(cmd);
        // Set the state of the task to APPM_UNPB_ADVTERSING
        kernel_state_set(TASK_BLE_APP, APPM_UNPB_ADVTERSING);	
    }
    else
    {
        return -1;
    }

    return 0;
}

uint8_t app_mesh_stop_unpb_adv(void)
{
    if (kernel_state_get(TASK_BLE_APP) == APPM_UNPB_ADVTERSING)
    {
        m_api_cmd_t *cmd = KERNEL_MSG_ALLOC(MESH_API_CMD, prf_get_task_from_id(TASK_BLE_ID_MESH), TASK_BLE_APP, m_api_cmd); //

        cmd->cmd_code = M_API_STOP_UNPROV;
        kernel_msg_send(cmd);
        // Set the state of the task to APPM_UNPB_ADVTERSING
        kernel_state_set(TASK_BLE_APP, APPM_READY);	
    }
    else
    {
        return -1;
    }

    return 0;
}

uint8_t app_mesh_start_cus_adv(uint32_t adv_len, uint8_t *adv_data, uint32_t rsp_len, uint8_t *rsp_data)
{
    if (kernel_state_get(TASK_BLE_APP) == APPM_READY)
    {
        m_api_cus_adv_msg_t *cmd = KERNEL_MSG_ALLOC(MESH_API_CMD, prf_get_task_from_id(TASK_BLE_ID_MESH), TASK_BLE_APP, m_api_cus_adv_msg); //

        cmd->cmd_code = M_API_START_CUS_ADV;
        cmd->adv_data_len = adv_len;
        memcpy(cmd->adv_data, adv_data, adv_len);
        cmd->rsp_data_len = rsp_len;
        memcpy(cmd->rsp_data, rsp_data, rsp_len);
        kernel_msg_send(cmd);
        // Set the state of the task to APPM_UNPB_ADVTERSING
        kernel_state_set(TASK_BLE_APP, APPM_CUSTOMER_ADVTERSING);	
    }
    else
    {
        return -1;
    }

    return 0;
}

uint8_t app_mesh_get_adv_state(void)
{
    uint8_t state;

    switch(kernel_state_get(TASK_BLE_APP))
    {
        case APPM_CUSTOMER_ADVTERSING:
        {
            state = CUS_ADV_RUNNING;
        }
        break;

        case APPM_UNPB_ADVTERSING:
        {
            state = PB_ADV_RUNNING;
        }
        break;

        case APPM_UNPB_CUSTOMER_ADVTERSING:
        {
            state = PB_CUS_RUNNING;
        }
        break;

        case APPM_READY:
        {
            state = ADV_IDLE;
        }
        break;

        default:
        {
            state = NEED_WAIT;
        }
        break;
    }

    return state;
}

uint8_t app_mesh_stop_cus_adv(void)
{
    if (kernel_state_get(TASK_BLE_APP) == APPM_CUSTOMER_ADVTERSING)
    {
        m_api_cmd_t *cmd = KERNEL_MSG_ALLOC(MESH_API_CMD, prf_get_task_from_id(TASK_BLE_ID_MESH), TASK_BLE_APP, m_api_cmd); //

        cmd->cmd_code = M_API_STOP_CUS_ADV;
        kernel_msg_send(cmd);
        // Set the state of the task to APPM_UNPB_ADVTERSING
        kernel_state_set(TASK_BLE_APP, APPM_READY);	
    }
    else
    {
        return -1;
    }

    return 0;
}

uint8_t app_mesh_start_unpb_cus_adv(uint32_t adv_len, uint8_t *adv_data, uint32_t rsp_len, uint8_t *rsp_data)
{
    if (kernel_state_get(TASK_BLE_APP) == APPM_READY)
    {
        m_api_cus_adv_msg_t *cmd = KERNEL_MSG_ALLOC(MESH_API_CMD, prf_get_task_from_id(TASK_BLE_ID_MESH), TASK_BLE_APP, m_api_cus_adv_msg); //

        cmd->cmd_code = M_API_START_CUS_PB_ADV;
        cmd->adv_data_len = adv_len;
        memcpy(cmd->adv_data, adv_data, adv_len);
        cmd->rsp_data_len = rsp_len;
        memcpy(cmd->rsp_data, rsp_data, rsp_len);
        kernel_msg_send(cmd);

        // Set the state of the task to APPM_UNPB_ADVTERSING
        kernel_state_set(TASK_BLE_APP, APPM_UNPB_CUSTOMER_ADVTERSING);	
    }
    else
    {
        return -1;
    }

    return 0;
}

uint8_t app_mesh_stop_unpb_cus_adv(void)
{
    if (kernel_state_get(TASK_BLE_APP) == APPM_UNPB_CUSTOMER_ADVTERSING)
    {
        m_api_cmd_t *cmd = KERNEL_MSG_ALLOC(MESH_API_CMD, prf_get_task_from_id(TASK_BLE_ID_MESH), TASK_BLE_APP, m_api_cmd); //

        cmd->cmd_code = M_API_STOP_CUS_PB_ADV;
        kernel_msg_send(cmd);
        // Set the state of the task to APPM_UNPB_ADVTERSING
        kernel_state_set(TASK_BLE_APP, APPM_READY);	
    }
    else
    {
        return -1;
    }

    return 0;
}

uint8_t app_relay_user_adv(uint16_t interval,uint8_t nb_tx,uint8_t data_len, const uint8_t* data)
{

	//MESH_APP_PRINT_DEBUG("%s\r\n",__func__); 
	uint8_t status = COMMON_ERROR_COMMAND_DISALLOWED;

	mal_adv_env_t *p_env = &(p_mal_env->adv);
    if(p_env)
    {
        // if(p_env->user_adv_finish == 1)
        {
            status = lld_adv_test_start( interval, 0x07, nb_tx, NULL,
                            data_len, data, MESH_API_USER_ADV_RELAY_IND, prf_get_task_from_id(TASK_BLE_ID_MESH),
                           0, 0);
            if(status == COMMON_ERROR_NO_ERROR)
            {
                p_env->user_adv_finish = 0;
            }
        }
    }
    MESH_APP_PRINT_DEBUG("%s,status:0x%x\r\n",__func__,status); 
    return status;
	
}

sys_reset_src_t sys_check_reset_src(void)
{    
#define SYS_MEM_INIT_VAL  0xaaaaaaaa    
#define SYS_MEM_CHECK_ADDR  0x00817FF0    
    uint32_t check_val;    
    check_val = REG_PL_RD(SYS_MEM_CHECK_ADDR);    
    if(check_val == SYS_MEM_INIT_VAL)
    {        
        MESH_APP_PRINT_INFO("Sys reset from power on:%x\n", check_val);        
        return SYS_RESET_BY_POWER_ON;
    }
    else    
    {        
        MESH_APP_PRINT_INFO("Sys reset from wdt:%x\n", check_val);
        return SYS_RESET_BY_WDT;
    }
}

#if (UART_CMD_PROV_EN)
extern void m_fnd_confs_cb_appkey_added(uint16_t status, m_lid_t app_key_lid);
static void app_test_cb_netkey_added(uint16_t status, m_lid_t net_key_lid)
{
    if (status == MESH_ERR_NO_ERROR)
    {
        m_tb_key_app_add(g_app_key_id, (const uint8_t *)&g_app_key[0], net_key_lid,
                                          m_fnd_confs_cb_appkey_added);
    }
}
  
void app_test_add_key(void)
{
    m_tb_key_dev_add((const uint8_t *)&g_dev_key[0], M_TB_KEY_DEVICE_LID);
	//m_tb_mio_set_prim_addr(0x0001);
	//m_tb_key_model_bind(0, 0);
    m_tb_key_net_add(g_net_key_id, (const uint8_t *)&g_net_key[0], 0, app_test_cb_netkey_added);
}
#endif /* UART_CMD_PROV_EN */
