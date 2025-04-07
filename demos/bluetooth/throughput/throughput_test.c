
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <string.h>

#include "error.h"
#include "rtos_pub.h"
#include "uart_pub.h"

#if THROUGHPUT_DEMO
#include "throughput_test.h"
#include "FreeRTOS.h"
#include "str_pub.h"
#include "mem_pub.h"
#include "app_sdp.h"
#include "ble_api_5_x.h"
#include "app_ble.h"
#include "ble_ui.h"
#include "l2cap_int.h"
#include "l2cap_hl_api.h"

extern int demo_start(void);
extern int hexstr2bin(const char *hex, u8 *buf, size_t len);


/*
 * DEFINES
 ****************************************************************************************
 */
#define BK_ATT_DECL_PRIMARY_SERVICE_128     {0x00,0x28,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
#define BK_ATT_DECL_CHARACTERISTIC_128      {0x03,0x28,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
#define BK_ATT_DESC_CLIENT_CHAR_CFG_128     {0x02,0x29,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
#define PT_WRITE_CMD_CHARACTERISTIC_128     {0x01,0xFF,0,0,0x34,0x56,0,0,0,0,0x28,0x37,0,0,0,0}
#define PT_NOTIFY_CHARACTERISTIC_128        {0x02,0xFF,0,0,0x34,0x56,0,0,0,0,0x28,0x37,0,0,0,0}

#define ATT_OPCODE_LEN          (1)
#define ATT_HANDLE_LEN          (2)
#define ATT_PKT_LEN             (LE_MAX_OCTETS - L2CAP_HEADER_LEN)
#define ATT_DATA_LEN            (ATT_PKT_LEN - ATT_OPCODE_LEN - ATT_HANDLE_LEN)

#define CONN_INTERVAL           (30)    //*1.25ms
#define COEX_DURATION           (29)    //slot
#define WC_HANDLE               (24)
#define STOP_CMD_LEN            (10)
#define CHECK_TRANS_CONDITION()         while (l2cap_chan_ll_buf_nb_avail_get() < 3) {rtos_thread_msleep(1);}
#define CHECK_TRANS_STATE()             if (tp_env.sending || tp_env.receiving) {\
                                            bk_printf("Test is running. Only respond to stop_tx command.\r\n");\
                                            return;\
                                        }

/*
 * LOCAL VARIABLES DEFINITIONS
 ****************************************************************************************
 */
static beken_queue_t ble_throughput_msg_que = NULL;
static const uint8_t tp_test_svc_uuid[16] = {0xFF,0xFF,0,0,0x34,0x56,0,0,0,0,0x28,0x37,0,0,0,0};
static uint8_t tx_stopped_msg[STOP_CMD_LEN] = {0x84,0x88,0x32,0x83,0x84,0x79,0x80,0x80,0x69,0x68};
static uint8_t rx_stopped_msg[STOP_CMD_LEN] = {0x82,0x88,0x32,0x83,0x84,0x79,0x80,0x80,0x69,0x68};
struct tp_env_tag tp_env;
beken_timer_t tp_cal_timer;
struct bd_addr peer_addr;

enum
{
    PT_TEST_IDX_SVC,
    PT_TEST_IDX_FF01_VAL_CHAR,
    PT_TEST_IDX_FF01_VAL_VALUE,
    PT_TEST_IDX_FF02_VAL_CHAR,
    PT_TEST_IDX_FF02_VAL_VALUE,
    PT_TEST_IDX_FF02_VAL_NTF_CFG,
    PT_TEST_IDX_NB,
};

bk_attm_desc_t pt_test_att_db[PT_TEST_IDX_NB] =
{
    //  Service Declaration
    [PT_TEST_IDX_SVC]              = {BK_ATT_DECL_PRIMARY_SERVICE_128, PROP(RD), 0},

    //  Level Characteristic Declaration
    [PT_TEST_IDX_FF01_VAL_CHAR]    = {BK_ATT_DECL_CHARACTERISTIC_128, PROP(RD), 0},
    //  Level Characteristic Value
    [PT_TEST_IDX_FF01_VAL_VALUE]   = {PT_WRITE_CMD_CHARACTERISTIC_128, PROP(WC), 256|OPT(NO_OFFSET)},

    //  Level Characteristic Declaration
    [PT_TEST_IDX_FF02_VAL_CHAR]    = {BK_ATT_DECL_CHARACTERISTIC_128, PROP(RD), 0},
    //  Level Characteristic Value
    [PT_TEST_IDX_FF02_VAL_VALUE]   = {PT_NOTIFY_CHARACTERISTIC_128, PROP(N), 256|OPT(NO_OFFSET)},
    //  Level Characteristic - Client Characteristic Configuration Descriptor
    [PT_TEST_IDX_FF02_VAL_NTF_CFG] = {BK_ATT_DESC_CLIENT_CHAR_CFG_128, PROP(RD)|PROP(WR), OPT(NO_OFFSET)},
};

/*
 * FUNCTIONS DECLARATION
 ****************************************************************************************
 */
static void ble_throughput_recv_stop_handler(recv_stop_reason_code_t reason);

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
static uint8_t ble_throughput_post_msg(uint16_t msg_id, void *data, uint32_t len)
{
    ble_tp_msg_t msg;

    if (ble_throughput_msg_que) {
        msg.msg_id = msg_id;
        msg.data = os_malloc(len);
        os_memcpy(msg.data, data, len);
        if (rtos_push_to_queue(&ble_throughput_msg_que, &msg, BEKEN_NO_WAIT) != 0) {
            bk_printf("%s cannot enqueue\r\n", __func__);
            if (msg.data) {
                os_free(msg.data);
            }
            return TP_ERR_BLE_FAIL;
        }
    } else {
        bk_printf("%s queue is not ready\r\n", __func__);
        return TP_ERR_BLE_FAIL;
    }

    return TP_ERR_BLE_SUCCESS;
}

void ble_tp_cmd_cb(ble_cmd_t cmd, ble_cmd_param_t *param)
{
    bk_printf("cmd:%d idx:%d status:%d\r\n", cmd, param->cmd_idx, param->status);

    switch (cmd) {
        case BLE_INIT_CREATE:
        {
            bk_ble_init_set_connect_dev_addr(param->cmd_idx, &peer_addr, 0);
            bk_ble_init_start_conn(param->cmd_idx,10000,ble_tp_cmd_cb);
        } break;
        default:
        {
        } break;
    }
}

static void ble_throughtput_slave_config_handler(void)
{
    if (tp_env.state >= SLAVE_CONNECTION) {
        bk_printf("ERROR: More than one connection! STOP TEST!\r\n");
        return;
    }
    tp_env.state = SLAVE_CONNECTION;
}

static void ble_throughtput_master_config_handler(void * param)
{
    uint8_t connidx = *(uint8_t *)param;
    ble_set_phy_t phy_cfg;

    if (tp_env.state >= SLAVE_CONNECTION) {
        bk_printf("ERROR: More than one connection! STOP TEST!\r\n");
        return;
    }
    tp_env.state = MASTER_CONNECTION;

    app_ble_mtu_exchange(connidx);
    sdp_discover_all_service(connidx);
    phy_cfg.phy_opt = 0;
    phy_cfg.rx_phy = 1 << 1;
    phy_cfg.tx_phy = 1 << 1;
    app_ble_gap_set_phy(connidx, &phy_cfg);
    app_ble_set_le_pkt_size(connidx, LE_MAX_OCTETS);
}

static void ble_throughput_calculate_timer_cb(void *param)
{
    int data_size;

    (void)param;
    data_size = tp_env.pkt_num_1s * LE_MAX_OCTETS * 8;
    bk_printf("%d bps\r\n", data_size);

    tp_env.pkt_num_last_total = tp_env.pkt_tatoal;
    tp_env.pkt_num_1s = 0;

    if (tp_env.pkt_lost_num_1s) {
        bk_printf("lost pkt num: 0x%x\r\n", tp_env.pkt_lost_num_1s);
        tp_env.pkt_lost_num_1s = 0;
    }

    if (++tp_env.test_time == 0xFF00) {
        ble_throughput_recv_stop_handler(RECV_STOP_CODE_BY_SELF);
    }
}

static void ble_tg_send_value(uint32_t len,uint8_t *buf)
{
    if (tp_env.state == SLAVE_CONNECTION) {
        bk_ble_send_ntf_value(len, buf, 0, PT_TEST_IDX_FF02_VAL_VALUE);
    } else if (tp_env.state == MASTER_CONNECTION) {
        bk_ble_write_service_data_req(0, WC_HANDLE, len, buf, NULL);
    }
}

static void ble_throughput_send_start_handler(void * param)
{

    uint8 write_buffer[ATT_DATA_LEN]={0};

    if (tp_env.state < SLAVE_CONNECTION) {
        bk_printf("ERROR: no connection!\r\n");
        return;
    }

    if ((tp_env.sending != SEND_READY) || (tp_env.receiving != RECEIVE_READY)) {
        bk_printf("ERROR: throughput test is already running!\r\n");
        return;
    }

    tp_env.sending = SEND_ONGOING;
    tp_env.pkt_tatoal = 0;

    while (1) {
        CHECK_TRANS_CONDITION();
        if (tp_env.sending == SEND_READY) {
            tp_env.pkt_tatoal = 0;
            tp_env.pkt_num_1s = 0;
            ble_tg_send_value(STOP_CMD_LEN, tx_stopped_msg);
            bk_printf("ThroughPut Test Stop!\r\n");
            break;
        }
        if (tp_env.pkt_tatoal % 3) {
            rtos_thread_msleep(1);
        }
        tp_env.pkt_tatoal++;
        tp_env.pkt_num_1s++;
        memcpy(write_buffer, &tp_env.pkt_tatoal, 4);
        ble_tg_send_value(ATT_DATA_LEN, write_buffer);
    }
}

static void ble_throughput_recv_start_handler(void)
{
    tp_env.receiving = RECEIVE_ONGOING;
    tp_env.pkt_num_1s = 0;
    tp_env.pkt_tatoal = 0;
    tp_env.pkt_lost_num = 0;
    tp_env.pkt_lost_num_1s = 0;
    tp_env.test_time = 0;
    rtos_start_timer(&tp_cal_timer);
}

static void ble_throughput_recv_pkt_handler(void * param)
{
    uint32_t data;

    memcpy(&data, param, sizeof(data));
    tp_env.pkt_num_1s++;
    tp_env.pkt_tatoal++;
    if (data != tp_env.pkt_tatoal) {
        tp_env.pkt_tatoal = data;
        tp_env.pkt_lost_num++;
        tp_env.pkt_lost_num_1s++;
    }
}

static void ble_throughput_recv_stop_handler(recv_stop_reason_code_t reason)
{
    int data_size_avg;

    if (tp_env.receiving == RECEIVE_ONGOING) {
        rtos_stop_timer(&tp_cal_timer);

        data_size_avg = tp_env.pkt_num_last_total * 8 / tp_env.test_time * LE_MAX_OCTETS;
        bk_printf("%d *8 / %d *251 \r\n", tp_env.pkt_num_last_total, tp_env.test_time);
        bk_printf("avg: %ld bps\r\n", data_size_avg);
        bk_printf("ptk_total_num=%d, pkt_lost_num=%d\r\n", tp_env.pkt_tatoal, tp_env.pkt_lost_num);
        bk_printf("ThroughPut Test Stop!\r\n");

        tp_env.pkt_tatoal = 0;
        tp_env.pkt_lost_num = 0;
        tp_env.pkt_lost_num_1s = 0;
        tp_env.pkt_num_1s = 0;
        tp_env.pkt_num_last_total = 0;
        tp_env.test_time = 0;
    }

    tp_env.receiving = RECEIVE_READY;
    if (reason == RECV_STOP_CODE_BY_SELF) {
        tp_env.receiving = RECEIVE_STOPPED;
        ble_tg_send_value(STOP_CMD_LEN, rx_stopped_msg);
    }

}

static void ble_throughtput_disconnect_handler(void)
{
    if (tp_env.state == SLAVE_CONNECTION) {
        bk_ble_start_advertising(tp_env.actv_idx, 0, ble_tp_cmd_cb);
        tp_env.state = ADV;
    } else if (tp_env.state == MASTER_CONNECTION) {
        bk_ble_init_start_conn(tp_env.actv_idx, 10000,ble_tp_cmd_cb);
        tp_env.state = INIT;
    }

    if (tp_env.receiving == RECEIVE_ONGOING) {
        ble_throughput_recv_stop_handler(RECV_STOP_CODE_DISCONNECT);
    }
}

static void ble_throughput_set_duration_handler(void * param)
{
    uint8_t duration = *(uint8_t *)param;

    if (tp_env.state != SLAVE_CONNECTION) {
        bk_printf("ERROR: STATE ERR!\r\n");
        return;
    }
    if (!duration) {
        duration = COEX_DURATION;
    }
    bk_printf("duration =%d\r\n", duration);
    app_ble_set_pref_slave_evt_dur(0, duration);
}

static void ble_tp_notice_cb(ble_notice_t notice, void *param)
{
    switch (notice) {
        case BLE_5_CONNECT_EVENT:
        {
            conn_ind_t *c_ind = (conn_ind_t *)param;
            bk_printf("c_ind:conn_idx:%d, addr_type:%d, peer_addr:%02x:%02x:%02x:%02x:%02x:%02x\r\n",
                c_ind->conn_idx, c_ind->peer_addr_type, c_ind->peer_addr[0], c_ind->peer_addr[1],
                c_ind->peer_addr[2], c_ind->peer_addr[3], c_ind->peer_addr[4], c_ind->peer_addr[5]);
            ble_throughput_post_msg(TP_SLAVE_CONFIG, NULL, 0);
            break;
        }
        case BLE_5_INIT_CONNECT_EVENT:
        {
            conn_ind_t *c_ind = (conn_ind_t *)param;
            bk_printf("BLE_5_INIT_CONNECT_EVENT:conn_idx:%d, addr_type:%d, peer_addr:%02x:%02x:%02x:%02x:%02x:%02x\r\n",
                c_ind->conn_idx, c_ind->peer_addr_type, c_ind->peer_addr[0], c_ind->peer_addr[1],
                c_ind->peer_addr[2], c_ind->peer_addr[3], c_ind->peer_addr[4], c_ind->peer_addr[5]);
            ble_throughput_post_msg(TP_MASTER_CONFIG, &c_ind->conn_idx, 1);
            break;
        }
        case BLE_5_DISCONNECT_EVENT:
        case BLE_5_INIT_DISCONNECT_EVENT:
        {
            tp_env.sending = SEND_READY;
            ble_throughput_post_msg(TP_DISCONNECTED, NULL, 0);
            break;
        }
        case BLE_5_CREATE_DB:
        {
            create_db_t *cd_ind = (create_db_t *)param;
            bk_printf("cd_ind:prf_id:%d, status:%d\r\n", cd_ind->prf_id, cd_ind->status);
            break;
        }
        case BLE_5_MTU_CHANGE:
        {
            mtu_change_t *m_ind = (mtu_change_t *)param;
            bk_printf("BLE_5_MTU_CHANGE:conn_idx:%d, mtu_size:%d\r\n", m_ind->conn_idx, m_ind->mtu_size);
            break;
        }
        case BLE_5_PHY_IND_EVENT:
        {
            conn_phy_ind_t *p_ind = (conn_phy_ind_t *)param;
            bk_printf(" tx_phy=0x%x, rx_phy=0x%x\r\n", p_ind->tx_phy, p_ind->rx_phy);
            if ((p_ind->tx_phy != (1 << 1)) || (p_ind->rx_phy != (1 << 1))) {
                bk_printf("ERROR: 2M PHY set fail!\r\n");
            }
            break;
        }
        case BLE_5_WRITE_EVENT:
        {
            write_req_t *w_cmd = (write_req_t *)param;
            if (w_cmd->len == STOP_CMD_LEN) {
                if ((tp_env.receiving != RECEIVE_READY) && !os_memcmp(tx_stopped_msg, w_cmd->value, STOP_CMD_LEN)) {
                    bk_printf("Recieve TX_Stopped_MSG, Stop RX!\r\n");
                    ble_throughput_post_msg(TP_WC_RECV_STOP, NULL, 0);
                } else if ((tp_env.sending == SEND_ONGOING) && !os_memcmp(rx_stopped_msg, w_cmd->value, STOP_CMD_LEN)) {
                    bk_printf("Recieve RX_Stopped_MSG, Stop TX!\r\n");
                    tp_env.sending = SEND_READY;
                }
            }
            if (w_cmd->len == ATT_DATA_LEN) {
                if (tp_env.receiving == RECEIVE_READY) {
                    ble_throughput_post_msg(TP_WC_RECV_SATRT, NULL, 0);
                }
                ble_throughput_post_msg(TP_WC_RECV_PKT, w_cmd->value, w_cmd->len);
            }
            break;
        }
        default:
            break;
    }
}

static void ble_tp_sdp_event_cb(sdp_notice_t notice, void *param)
{
    switch (notice) {
        case SDP_CHARAC_NOTIFY_EVENT:
        {
            sdp_event_t *g_sdp = (sdp_event_t *)param;
            if (g_sdp->value_length == STOP_CMD_LEN) {
                if ((tp_env.receiving != RECEIVE_READY) && !os_memcmp(tx_stopped_msg, g_sdp->value, STOP_CMD_LEN)) {
                    bk_printf("Recieve TX_Stopped_MSG, Stop RX!\r\n");
                    ble_throughput_post_msg(TP_NTF_RECV_STOP, NULL, 0);
                } else if ((tp_env.sending == SEND_ONGOING) && !os_memcmp(rx_stopped_msg, g_sdp->value, STOP_CMD_LEN)) {
                    bk_printf("Recieve RX_Stopped_MSG, Stop TX!\r\n");
                    tp_env.sending = SEND_READY;
                }
            }
            if (g_sdp->value_length == ATT_DATA_LEN) {
                if (tp_env.receiving == RECEIVE_READY) {
                    ble_throughput_post_msg(TP_NTF_RECV_SATRT, NULL, 0);
                }
                ble_throughput_post_msg(TP_NTF_RECV_PKT, g_sdp->value, g_sdp->value_length);
            }
            break;
        }
        case SDP_DISCOVER_SVR_DONE:
        {
            bk_printf("[SDP_DISCOVER_SVR_DONE]\r\n");
            break;
        }
        case SDP_CHARAC_WRITE_DONE:
            break;
        default:
        {
            bk_printf("[%s]Event:%d\r\n",__func__,notice);
            break;
        }
    }
}

static void ble_tp_start_adv_handler(void)
{
    struct bk_ble_db_cfg ble_db_cfg;
    struct adv_param adv_info;
    uint8_t actv_idx;

    if (tp_env.state != IDLE) {
        bk_printf("ERROR: STATE ERR!\r\n");
        return;
    }
    tp_env.state = ADV;

    ble_set_notice_cb(ble_tp_notice_cb);
    ble_db_cfg.att_db = pt_test_att_db;
    ble_db_cfg.att_db_nb = PT_TEST_IDX_NB;
    ble_db_cfg.prf_task_id = 0;
    ble_db_cfg.start_hdl = 0;
    ble_db_cfg.svc_perm = BK_PERM_SET(SVC_UUID_LEN, UUID_16);
    memcpy(&(ble_db_cfg.uuid[0]), &tp_test_svc_uuid[0], 16);
    bk_ble_create_db(&ble_db_cfg);

    adv_info.channel_map = 7;
    adv_info.duration = 0;
    adv_info.prop = (1 << ADV_PROP_CONNECTABLE_POS) | (1 << ADV_PROP_SCANNABLE_POS);
    adv_info.interval_min = 160;
    adv_info.interval_max = 160;
    adv_info.advData[0] = 0x0D;
    adv_info.advData[1] = 0x09;
    memcpy(&adv_info.advData[2], "7238_TP_TEST", 12);
    adv_info.advDataLen = 14;
    adv_info.respData[0] = 0x08;
    adv_info.respData[1] = 0x08;
    memcpy(&adv_info.respData[2], "7238_TP", 7);
    adv_info.respDataLen = 9;
    actv_idx = app_ble_get_idle_actv_idx_handle();
    tp_env.actv_idx = actv_idx;
    bk_ble_adv_start(actv_idx, &adv_info, ble_tp_cmd_cb);
}

static void ble_tp_start_init_handler(void *param)
{
    uint8_t actv_idx;
    uint8_t interval = *(uint8_t *)param;
    struct appm_create_conn_param init_par;
    uint8_t phy_mask;

    if (tp_env.state != IDLE) {
        bk_printf("ERROR: STATE ERR!\r\n");
        return;
    }
    tp_env.state = INIT;

    ble_set_notice_cb(ble_tp_notice_cb);
    sdp_set_notice_cb(ble_tp_sdp_event_cb);

    phy_mask = 1;
    if (!interval) {
        interval = CONN_INTERVAL;
    }
    memset(&init_par, 0 , sizeof(init_par));
    init_par.conn_intv_max = interval;
    init_par.conn_intv_min = interval;
    bk_ble_gap_prefer_ext_connect_params_set(phy_mask, &init_par, NULL, NULL);

    actv_idx = app_ble_get_idle_conn_idx_handle();
    tp_env.actv_idx = actv_idx;
    bk_printf("------------->conn_idx:%d, conn_interval=%d\r\n",actv_idx, interval);
    bk_ble_create_init(actv_idx, ble_tp_cmd_cb);
}

static void throuthput_msg_handler(ble_tp_msg_t *msg)
{
    switch (msg->msg_id) {
        case TP_START_ADV:
            ble_tp_start_adv_handler();
            break;
        case TP_START_INIT:
            ble_tp_start_init_handler(msg->data);
            break;
        case TP_SLAVE_CONFIG:
            ble_throughtput_slave_config_handler();
            break;
        case TP_SET_DURATION:
            ble_throughput_set_duration_handler(msg->data);
            break;
        case TP_MASTER_CONFIG:
            ble_throughtput_master_config_handler(msg->data);
            break;
        case TP_DISCONNECTED:
            ble_throughtput_disconnect_handler();
            break;
        case TP_NTF_SEND_START:
        case TP_WC_SEND_START:
            ble_throughput_send_start_handler(msg->data);
            break;
        case TP_NTF_SEND_STOP:
        case TP_WC_SEND_STOP:
            break;
        case TP_NTF_RECV_SATRT:
        case TP_WC_RECV_SATRT:
            ble_throughput_recv_start_handler();
            break;
        case TP_NTF_RECV_PKT:
        case TP_WC_RECV_PKT:
            ble_throughput_recv_pkt_handler(msg->data);
            break;
        case TP_NTF_RECV_STOP:
        case TP_WC_RECV_STOP:
            ble_throughput_recv_stop_handler(RECV_STOP_CODE_TX_STOP);
            break;
        default:
            break;
    }
}

static void ble_throughput_thread(void *arg)
{
    memset(&tp_env, 0, sizeof(tp_env));
    tp_env.actv_idx = 0xFF;
    rtos_init_timer(&tp_cal_timer, 1000, ble_throughput_calculate_timer_cb, (void *)0);

    while(1) {
        uint8_t err;
        ble_tp_msg_t msg;
        err = rtos_pop_from_queue(&ble_throughput_msg_que, &msg, BEKEN_WAIT_FOREVER);
        if (0 == err) {
            throuthput_msg_handler(&msg);
            if (msg.data > 0) {
                os_free(msg.data);
                msg.data = NULL;
            }
        }
    }
    rtos_deinit_queue(&ble_throughput_msg_que);
    ble_throughput_msg_que = NULL;
    rtos_delete_thread(NULL);
}

void ble_throughput_test_start(void)
{
    OSStatus ret;

    if (ble_throughput_msg_que == NULL) {
        ret = rtos_init_queue(&ble_throughput_msg_que,
                                "ble_throughput_msg_que",
                                sizeof(ble_tp_msg_t),
                                20);
        ASSERT(0 == ret);

        ret = rtos_create_thread(NULL,
                7,
                "ble_throughput",
                ble_throughput_thread,
                0x400,
                0);
        ASSERT(0 == ret);
    }
}

void ble_tp_cli_cmd(int argc, char **argv)
{
    if (os_strcmp(argv[1], "init") == 0) {
        CHECK_TRANS_STATE();
        ble_throughput_test_start();
    }
    if (os_strcmp(argv[1], "start_adv") == 0) {
        CHECK_TRANS_STATE();
        ble_throughput_post_msg(TP_START_ADV, NULL, 0);
    }
    if (os_strcmp(argv[1], "start_init") == 0) {
        CHECK_TRANS_STATE();
        uint8_t interval;
        interval = os_strtoul(argv[3], NULL, 10);
        hexstr2bin(argv[2], peer_addr.addr, GAP_BD_ADDR_LEN);
        ble_throughput_post_msg(TP_START_INIT, &interval, 1);
    }
    if (os_strcmp(argv[1], "start_tx") == 0) {
        CHECK_TRANS_STATE();
        ble_throughput_post_msg(TP_NTF_SEND_START, NULL, 0);
    }
    if (os_strcmp(argv[1], "stop_tx") == 0) {
        tp_env.sending = 0;
    }
    if (os_strcmp(argv[1], "wifi_coex") == 0) {
        CHECK_TRANS_STATE();
        uint8_t duration;
        duration = os_strtoul(argv[2], NULL, 10);
        ble_throughput_post_msg(TP_SET_DURATION, &duration, 1);
    }
}

#endif
