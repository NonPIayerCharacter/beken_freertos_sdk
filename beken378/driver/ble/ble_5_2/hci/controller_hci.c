#include <stddef.h>     // standard definition
#include <stdarg.h>
#include <stdint.h>        // standard integer definition
#include <string.h>        // string manipulation
#include <stdio.h>
#include "sys_config.h"
#include "ble_api_5_x.h"
#include "rtos_pub.h"
#include "mem_pub.h"
#include "ble.h"
#include "bk_err.h"
#include "controller_hci_driver.h"


struct hci_context {
    beken_mutex_t lock;
    int initialized;
};

static ble_hci_to_host_cb s_ble_hci_evt_to_host_cb = NULL;
static ble_hci_to_host_cb s_ble_hci_acl_to_host_cb = NULL;
static struct hci_context g_hci_send_context;

static void* s_ble_hci_to_host_msg_que;
void* s_ble_hci_to_host_thread_handle;

static void hci_lock(struct hci_context *context)
{
    if (context->initialized)
        rtos_lock_mutex(&(context->lock));
}

static void hci_unlock(struct hci_context *context)
{
    if (context->initialized)
        rtos_unlock_mutex(&(context->lock));
}

static void ble_hci_to_host_thread(void *arg)
{
    while (1)
    {
        bk_err_t err;
        BLE_TO_HOST_MSG_T msg;

        err = rtos_pop_from_queue(&s_ble_hci_to_host_msg_que, &msg, BEKEN_WAIT_FOREVER);
        ASSERT(err == kNoErr);

        if (kNoErr == err)
        {
            //BLE_LOGI("%s recv msg %d\n", __func__, msg.msg_id);

            switch (msg.msg_id)
            {
                case BLE_MSG_TO_HOST_HCI:

                    switch (msg.type)
                    {
                        case BK_BLE_HCI_TYPE_EVT:
                            if (s_ble_hci_evt_to_host_cb)
                            {
                                s_ble_hci_evt_to_host_cb(((uint8_t *)msg.data) + 1, msg.len - 1);
                            }

                            break;

                        case BK_BLE_HCI_TYPE_ACL:
                            if (s_ble_hci_acl_to_host_cb)
                            {
                                s_ble_hci_acl_to_host_cb(((uint8_t *)msg.data) + 1, msg.len - 1);
                            }

                            break;

                        default:
                            bk_printf("%s unknow type %d\n", __func__, msg.type);
                            break;
                    }

                    os_free(msg.data);
                    break;

                default:
                    break;
            }
        }
        else
        {
            ASSERT(kNoErr == err);
        }
    }

    rtos_deinit_queue(&s_ble_hci_to_host_msg_que);
    s_ble_hci_to_host_msg_que = NULL;
    s_ble_hci_to_host_thread_handle = NULL;
    rtos_delete_thread(NULL);
}

static bk_err_t create_hci_report_thread(void)
{
    bk_err_t ret = BK_OK;

    if (!s_ble_hci_to_host_thread_handle && !s_ble_hci_to_host_msg_que)
    {
        ret = rtos_init_queue(&s_ble_hci_to_host_msg_que,
                              "ble_hci_to_host_msg_que",
                              sizeof(BLE_TO_HOST_MSG_T),
                              128);

        ASSERT(0 == ret);

        if (ret != 0)
        {
            return BK_FAIL;
        }

        ret = rtos_create_thread(&s_ble_hci_to_host_thread_handle,
                                 4,
                                 "hci_report",
                                 (void*)ble_hci_to_host_thread,
                                 4096,
                                 (void*)0);

        ASSERT(0 == ret);

        if (ret != 0)
        {
            rtos_deinit_queue(&s_ble_hci_to_host_msg_que);
            s_ble_hci_to_host_msg_que = NULL;
            return BK_FAIL;
        }
    }

    return ret;
}

static bool ble_is_hci_reg_recv_callback(void)
{
    if (s_ble_hci_evt_to_host_cb || s_ble_hci_acl_to_host_cb)
    {
        return true;
    }

    return false;
}

static int ble_hci_send_to_host(uint8_t *bufptr, uint32_t size)
{
    BLE_TO_HOST_MSG_T msg = {0};
    int ret = 0;

    if (!s_ble_hci_to_host_msg_que)
    {
        bk_printf("s_ble_hci_to_host_msg_que not create");
        return BK_FAIL;
    }


    if (bufptr[0] == 0x02 && (((uint16_t)bufptr[4]) << 8 ) + bufptr[3] == 0 )
    {
        bk_printf("%s acl send len 0, ignore\n", __func__);
        return BK_OK;
    }

    msg.type = bufptr[0];
    msg.len = size;
    msg.msg_id = BLE_MSG_TO_HOST_HCI;

    if (size)
    {
        msg.data = (void *)os_malloc(size);
        ASSERT(NULL != msg.data);

        if (msg.data)
        {
            memcpy(msg.data, bufptr, size);
            ret = rtos_push_to_queue(&s_ble_hci_to_host_msg_que, &msg, BEKEN_NO_WAIT);

            if (ret != 0)
            {
                bk_printf("%s push queue err ret %d\n", __func__, ret);
                return ret;
            }
        }
        else
        {
            return BK_ERR_NO_MEM;
        }
    }

    return BK_OK;
}

static void controller_only_by_api_recv_pkt_cb(void *data, uint16_t len)
{
    if (ble_is_hci_reg_recv_callback())
    {
        ble_hci_send_to_host(data, len);
    }
}

ble_err_t bk_ble_hci_to_controller(uint8_t type, uint8_t *buf, uint16_t len)
{
    ble_err_t status = ERR_SUCCESS;

    hci_lock(&g_hci_send_context);

    status = controller_hci_driver_get_interface()->hci_data_send(type, buf, len);

    hci_unlock(&g_hci_send_context);

    return status;
}

ble_err_t bk_ble_hci_cmd_to_controller(uint8_t *buf, uint16_t len)
{
    return bk_ble_hci_to_controller(BK_BLE_HCI_TYPE_CMD, buf, len);
}

ble_err_t bk_ble_hci_acl_to_controller(uint8_t *buf, uint16_t len)
{
    return bk_ble_hci_to_controller(BK_BLE_HCI_TYPE_ACL, buf, len);
}

static controller_hci_driver_callbacks_t hci_driver = {
    .notify_host_recv_cb = controller_only_by_api_recv_pkt_cb,
};

ble_err_t bk_ble_reg_hci_recv_callback(ble_hci_to_host_cb evt_cb, ble_hci_to_host_cb acl_cb)
{
    ble_err_t ret = ERR_SUCCESS;

    ret = create_hci_report_thread();

    if (ret != ERR_SUCCESS)
    {
        return ret;
    }

    s_ble_hci_evt_to_host_cb = evt_cb;
    s_ble_hci_acl_to_host_cb = acl_cb;

    if (kNoErr == rtos_init_mutex(&g_hci_send_context.lock)) {
        g_hci_send_context.initialized = 1;
    }
    else {
        return ERR_NO_MEM;
    }

	controller_hci_driver_get_interface()->init(&hci_driver);

    return ERR_SUCCESS;
}