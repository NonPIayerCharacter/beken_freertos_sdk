#include "include.h"
#include "atsvr_ble.h"
#include "atsvr_cmd.h"
#include "atsvr_comm.h"
#include "at_server.h"

#include "string.h"
#include "stdio.h"
#include "stdlib.h"

#include "wlan_ui_pub.h"
#include "rtos_pub.h"

#include "app_ble.h"
#include "app_sdp.h"
#include "app_ble_init.h"
#include "BkDriverFlash.h"
#include "cJSON.h"
#include "ble_api_5_x.h"
#include "app_sdp.h"
#include "common.h"

static hal_ble_env_t hal_ble_env;
static char at_rsp_msg_buf[200];
static ServerList_t srv_db_list;
static void at_ble_param_debug_printf(void);
static void at_ble_connected_callback(uint8_t conn_idx, uint8_t *remote_address);
static void at_ble_connect_timeout_callback(uint8_t conn_idx);
static void at_ble_disconnect_callback(uint8_t conn_idx);
//static void at_ble_app_sdp_characteristic_cb(unsigned char conidx,uint16_t chars_val_hdl,unsigned char uuid_len,unsigned char *uuid);
//static void at_app_sdp_charac_cb(CHAR_TYPE type,uint8 conidx,uint16_t hdl,uint16_t len,uint8 *data);
//static void at_sdp_comm_callback(MASTER_COMMON_TYPE type, uint8 conidx, void *param);

extern UINT64 fclk_get_tick(void);
extern void register_app_sdp_common_callback(app_sdp_comm_callback comm_cb);
extern int hexstr2bin(const char * hex, u8 * buf, size_t len);
extern ble_err_t app_ble_set_le_pkt_size(uint8_t conn_idx, uint16_t pkt_size);

static const bk_attm_desc_t attrListPattern[BK_Attr_IDX_NB]=
{
    //  Service Declaration
    [BK_Attr_IDX_SVC]               = {BK_ATT_DECL_PRIMARY_SERVICE, BK_PERM_SET(RD, ENABLE), 0},

    //  Level Characteristic Declaration
    [BK_Attr_IDX_CHAR]              = {BK_ATT_DECL_CHARACTERISTIC, BK_PERM_SET(RD, ENABLE), 0},

    //  Level Client Characteristic Configuration
    [BK_Attr_IDX_CLIENT_CHAR_CFG]   = {BK_ATT_DESC_CLIENT_CHAR_CFG, BK_PERM_SET(RD, ENABLE)|BK_PERM_SET(WRITE_REQ,ENABLE), 0},
};

static uint8_t svr_create_flag=0;
static uint8_t svr_start_flag=0;

static void  at_bk_ble_svr_init(uint8_t srv_index)
{
    BtServer *srv_tmp=NULL;
    struct bk_ble_db_cfg ble_db_cfg;
    uint32_t tick=0;

    srv_tmp=srv_db_list.srv;
    while(srv_tmp)
    {
        if(srv_tmp->service_id == srv_index)
        {
            break;
        }
        srv_tmp=srv_tmp->next;
    }
    if(!srv_tmp){
        bk_printf("ERROR: wrong srv_index\r\n");
        return;
    }
    if(srv_tmp->start)
    {
        bk_printf("ERROR: This service already started.\r\n");
        return;
    }
    ble_db_cfg.att_db = srv_tmp->att_db;
    ble_db_cfg.att_db_nb = srv_tmp->max_att_handle;
    ble_db_cfg.prf_task_id = srv_tmp->service_id-1;
    ble_db_cfg.start_hdl = 0;

    if(srv_tmp->uuidLen==16)
        ble_db_cfg.svc_perm = BK_PERM_SET(SVC_UUID_LEN, UUID_128);
    else
        ble_db_cfg.svc_perm = BK_PERM_SET(SVC_UUID_LEN, UUID_16);

    memcpy(&(ble_db_cfg.uuid[0]), srv_tmp->uuid, 16);

    for(int i=0;i<srv_tmp->uuidLen/2;i++)
    {
        uint8_t tmp;
        tmp=ble_db_cfg.uuid[i];
        ble_db_cfg.uuid[i]=ble_db_cfg.uuid[srv_tmp->uuidLen-1-i];
        ble_db_cfg.uuid[srv_tmp->uuidLen-1-i]=tmp;
    }

    for(int i=0;i<ble_db_cfg.att_db_nb;i++)
    {
        ble_db_cfg.att_db[i].info = (ble_db_cfg.att_db[i].info>>8);

        if(BK_PERM_RIGHT_UUID_128==BK_PERM_GET(ble_db_cfg.att_db[i].ext_info,UUID_LEN))
        {
            ble_db_cfg.att_db[i].info=ble_db_cfg.att_db[i].info|0x8000;
        }
        bk_printf("info:0x%d,exinfo:0x%x\r\n",ble_db_cfg.att_db[i].info,ble_db_cfg.att_db[i].ext_info);
    }

    svr_start_flag=0;
    bk_ble_create_db(&ble_db_cfg);
    tick=fclk_get_tick();

    while(!svr_start_flag)
    {
        if(fclk_get_tick()-tick>1000)
        {
            bk_printf("creare svr timeout\r\n");
        }
        rtos_delay_milliseconds(50);
    }

    bk_printf("create svr %d success \r\n",srv_index);
    srv_tmp->start=1;
}

static void str_to_hex(uint8_t *data_buffer, uint8_t *str, uint16_t str_len )
{
    char all_char[16]="0123456789abcdef";
    char all_char_A[16]="0123456789ABCDEF";
    uint8_t h_bit=0,l_bit=0;
    uint8_t hexbit=0;

    for(uint16_t i=0;i<str_len;i=i+3)
    {
        for(uint8_t j=0;j<16;j++)
        {
            if((all_char[j]==str[i]) || (all_char_A[j]==str[i]))
            {
                h_bit = j;
                break;
            }
        }

        for(uint8_t j=0;j<16;j++)
        {
            if((all_char[j]==str[i+1]) || (all_char_A[j]==str[i+1]))
            {
                l_bit = j;
                break;
            }
        }
        data_buffer[hexbit++]=h_bit<<4|l_bit;

    }
}

uint8_t appm_adv_data_decode(uint8_t len,const uint8_t *data,uint8_t *find_str,uint8_t str_len)
{
    uint8_t find = 0;
    uint8_t index;

    for(index = 0; index < len;)
    {
        switch(data[index + 1])
        {
            case GAP_AD_TYPE_FLAGS:
            {
                index +=(data[index] + 1);
            }
            break;
            case GAP_AD_TYPE_SHORTENED_NAME:
            case GAP_AD_TYPE_COMPLETE_NAME:
            {
                if(strncmp((char*)&data[index + 2],(const char *)find_str,str_len) == 0 )
                {
                    find = 1;
                }
                index +=(data[index] + 1);
            }
            break;
            case GAP_AD_TYPE_MORE_16_BIT_UUID:
            {
                index +=(data[index] + 1);
            }
            break;
            case GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID:
            {
                if( (data[index+2]==0x12) && (data[index+3]==0x18) )////HID?υτ???????????
                {

                }
                index +=(data[index] + 1);
            }
            break;
            default:
            {
                index +=(data[index] + 1);
            }
            break;
        }
    }
    return find;
}

static void at_ble_notice_cb(ble_notice_t notice, void *param)
{
    uint16 rsp_len=0;
    switch (notice) {
    case BLE_5_STACK_OK:
        bk_printf("ble stack ok");
        break;
    case BLE_5_WRITE_EVENT:
    {
        write_req_t *w_req = (write_req_t *)param;
        bk_printf("write_cb:conn_idx:%d, prf_id:%d, add_id:%d, len:%d, data[0]:%02x\r\n",
            w_req->conn_idx, w_req->prf_id, w_req->att_idx, w_req->len, w_req->value[0]);

        rsp_len=snprintf(at_rsp_msg_buf,sizeof(at_rsp_msg_buf),"+WRITE:%d,%d,%d,%d,%02x\r\n",
                                    hal_ble_env.connidx,w_req->prf_id,w_req->att_idx,w_req->len,w_req->value[0]);
        atsvr_output_msg(at_rsp_msg_buf,rsp_len);
        break;
    }
    case BLE_5_READ_EVENT:
    {
        read_req_t *r_req = (read_req_t *)param;
        bk_printf("read_cb:conn_idx:%d, prf_id:%d, add_id:%d\r\n",
            r_req->conn_idx, r_req->prf_id, r_req->att_idx);
        r_req->value[0] = 0x12;
        r_req->value[1] = 0x34;
        r_req->value[2] = 0x56;
        r_req->length = 3;

        rsp_len=snprintf(at_rsp_msg_buf,sizeof(at_rsp_msg_buf),"+READ:%d,%d,%d,%d,%02x\r\n",
                                    hal_ble_env.connidx,r_req->prf_id,r_req->att_idx,r_req->length,r_req->value[0]);
        atsvr_output_msg(at_rsp_msg_buf,rsp_len);
        break;
    }
    case BLE_5_REPORT_ADV:
    {
        recv_adv_t *r_ind = (recv_adv_t *)param;
        struct scan_param scan = app_ble_env.actvs[r_ind->actv_idx].param.scan;

        if(scan.filter_type == 1)//mac
        {
            uint8_t addr[6];
            str_to_hex(addr,scan.filter_param,strlen((char *)scan.filter_param));
            if(memcmp(addr,r_ind->adv_addr,GAP_BD_ADDR_LEN)!=0)
            {
                break;
            }
        }else if(scan.filter_type == 2)//name
        {
            if(appm_adv_data_decode(r_ind->data_len,r_ind->data,scan.filter_param,strlen((char*)scan.filter_param))==0)
            {
                break;
            }
        }
        bk_printf("[%s]r_ind:actv_idx:%d,evt_type:%d adv_addr:%02x:%02x:%02x:%02x:%02x:%02x,rssi:%d\r\n",
            ((r_ind->evt_type&0x7) == 3)?"scan-rsp":((r_ind->evt_type&0x7) == 1)?"adv":"unknow",
            r_ind->actv_idx,r_ind->evt_type, r_ind->adv_addr[0], r_ind->adv_addr[1], r_ind->adv_addr[2],
            r_ind->adv_addr[3], r_ind->adv_addr[4], r_ind->adv_addr[5],r_ind->rssi);
        break;
    }
    case BLE_5_MTU_CHANGE:
    {
        mtu_change_t *m_ind = (mtu_change_t *)param;
        bk_printf("m_ind:conn_idx:%d, mtu_size:%d\r\n", m_ind->conn_idx, m_ind->mtu_size);
        break;
    }
    case BLE_5_CONNECT_EVENT:
    {
        conn_ind_t *c_ind = (conn_ind_t *)param;
        hal_ble_env.connidx =c_ind->conn_idx;
        bk_printf("c_ind:conn_idx:%d, addr_type:%d, peer_addr:%02x:%02x:%02x:%02x:%02x:%02x\r\n",
            c_ind->conn_idx, c_ind->peer_addr_type, c_ind->peer_addr[0], c_ind->peer_addr[1],
            c_ind->peer_addr[2], c_ind->peer_addr[3], c_ind->peer_addr[4], c_ind->peer_addr[5]);
        at_ble_connected_callback(c_ind->conn_idx,c_ind->peer_addr);
        break;
    }
    case BLE_5_DISCONNECT_EVENT:
    {
        discon_ind_t *d_ind = (discon_ind_t *)param;
        bk_printf("d_ind:conn_idx:%d,reason:%d\r\n", d_ind->conn_idx,d_ind->reason);
        at_ble_disconnect_callback(d_ind->conn_idx);
        break;
    }
    case BLE_5_ATT_INFO_REQ:
    {
        att_info_req_t *a_ind = (att_info_req_t *)param;
        bk_printf("a_ind:conn_idx:%d\r\n", a_ind->conn_idx);
        a_ind->length = 128;
        a_ind->status = ERR_SUCCESS;
        break;
    }
    case BLE_5_CREATE_DB:
    {
        create_db_t *cd_ind = (create_db_t *)param;
        bk_printf("cd_ind:prf_id:%d, status:%d\r\n", cd_ind->prf_id, cd_ind->status);
        svr_start_flag=1;
        break;
    }
    #if (BLE_CENTRAL)
    case BLE_5_INIT_CONNECT_EVENT:
    {
        conn_ind_t *c_ind = (conn_ind_t *)param;
        #if (CFG_SOC_NAME == SOC_BK7238)
        app_ble_get_peer_feature(c_ind->conn_idx);
        app_ble_set_le_pkt_size(c_ind->conn_idx,LE_MAX_OCTETS);
        sdp_discover_all_service(c_ind->conn_idx);
        #endif
        bk_printf("BLE_5_INIT_CONNECT_EVENT:conn_idx:%d, addr_type:%d, peer_addr:%02x:%02x:%02x:%02x:%02x:%02x\r\n",
            c_ind->conn_idx, c_ind->peer_addr_type, c_ind->peer_addr[0], c_ind->peer_addr[1],
            c_ind->peer_addr[2], c_ind->peer_addr[3], c_ind->peer_addr[4], c_ind->peer_addr[5]);

        at_ble_connected_callback(c_ind->conn_idx,c_ind->peer_addr);
        break;
    }
    case BLE_5_INIT_DISCONNECT_EVENT:
    {
        discon_ind_t *d_ind = (discon_ind_t *)param;
        #if (CFG_SOC_NAME == SOC_BK7238)
        sdp_common_cleanup(d_ind->conn_idx);
        #endif
        bk_printf("BLE_5_INIT_DISCONNECT_EVENT:conn_idx:%d,reason:0x%x\r\n", d_ind->conn_idx,d_ind->reason);
        at_ble_disconnect_callback(d_ind->conn_idx);
        break;
    }
    #endif
    case BLE_5_INIT_CONN_PARAM_UPDATE_REQ_EVENT:
    {
        conn_param_t *d_ind = (conn_param_t *)param;
        bk_printf("BLE_5_INIT_CONN_PARAM_UPDATE_REQ_EVENT:conn_idx:%d,intv_min:%d,intv_max:%d,time_out:%d\r\n",d_ind->conn_idx,
            d_ind->intv_min,d_ind->intv_max,d_ind->time_out);
        app_ble_send_conn_param_update_cfm(d_ind->conn_idx,true);
    }
    break;
    case BLE_5_INIT_CONN_PARAM_UPDATE_IND_EVENT:
    {
        conn_update_ind_t *d_ind = (conn_update_ind_t *)param;
        bk_printf("BLE_5_INIT_CONN_PARAM_UPDATE_IND_EVENT:conn_idx:%d,interval:%d,time_out:%d,latency\r\n",d_ind->conn_idx,
            d_ind->interval,d_ind->time_out,d_ind->latency);
        hal_ble_env.conn.con_interval = d_ind->interval;
        hal_ble_env.conn.con_latency = d_ind->latency;
        hal_ble_env.conn.sup_to = d_ind->time_out;
    }
    break;
    case BLE_5_INIT_CONNECT_FAILED_EVENT:
    {
        discon_ind_t *d_ind = (discon_ind_t *)param;
        bk_printf("BLE_5_INIT_CONNECT_FAILED_EVENT:conn_idx:%d,reason:%d\r\n", d_ind->conn_idx,d_ind->reason);
        at_ble_connect_timeout_callback(d_ind->conn_idx);
    }
    break;
    case BLE_5_SDP_REGISTER_FAILED:
        bk_printf("BLE_5_SDP_REGISTER_FAILED\r\n");
        break;
    default:
        break;
    }
}

static void at_ble_cmd_cb(ble_cmd_t cmd, ble_cmd_param_t *param)
{

    bk_printf("cmd:%d idx:%d status:%d\r\n", cmd, param->cmd_idx, param->status);

    switch (cmd) {
        case BLE_CREATE_ADV:
            break;
        case BLE_SET_ADV_DATA:
            break;
        case BLE_SET_RSP_DATA:
            break;
        case BLE_START_ADV:
            break;
        case BLE_STOP_ADV:
            break;
        case BLE_DELETE_ADV:
            break;
        case BLE_CREATE_SCAN:
            break;
        case BLE_START_SCAN:
            break;
        case BLE_STOP_SCAN:
            break;
        case BLE_DELETE_SCAN:
            break;
        case BLE_CONN_UPDATE_MTU:
            break;
        case BLE_CONN_UPDATE_PARAM:
            break;
        case BLE_CONN_DIS_CONN:
            bk_printf("BLE_CONN_DIS_CONN\r\n");
            break;
        case BLE_INIT_WRITE_CHAR:
            bk_printf("BLE_INIT_WRITE_CHAR\r\n");
            break;
        case BLE_INIT_CREATE:
            bk_printf("BLE_INIT_CREATE\r\n");
            break;
        case BLE_INIT_START_CONN:
            bk_printf("BLE_INIT_START_CONN\r\n");
            break;
        case BLE_INIT_STOP_CONN:
            bk_printf("BLE_INIT_STOP_CONN\r\n");
            break;
        default:
            break;
    }
}

static ble_err_t bk_ble_get_mac(uint8_t *mac)
{
    if (!mac)
        return -1;

    memcpy(mac, common_default_bdaddr.addr, BD_ADDR_LEN);
    return 0;
}

static void at_ble_get_mode(int argc, char **argv)
{
    uint8_t len;
    memset(at_rsp_msg_buf,0,sizeof(at_rsp_msg_buf));
    len=snprintf(at_rsp_msg_buf,sizeof(at_rsp_msg_buf),"+BLEINIT:%d\r\nOK\r\n",hal_ble_env.mode);
    atsvr_output_msg(at_rsp_msg_buf,len);
}

void at_sdp_discovery_cb(uint8_t con_idx,sdp_att_type_t notice, void *param)
{
    bk_printf("notice:%d \r\n",notice);
    switch (notice){
        case SDP_ATT_GET_SVR_UUID_ALL:
        {
            struct db *p_svr=(struct db *)param;
            uint8_t uuid_len;
            char uuid_str[32+1];
            char *buffer_ptr;
            char ble_buf[200];
            int ble_len=0;
            uint8_t srv_idx;
            struct sdp_env_tag * p_env = sdp_get_env_use_conidx(con_idx);
            struct sdp_db *p_db = (struct sdp_db *)common_list_pick(&p_env->svr_list);
            struct db *p_st;

            srv_idx=1;
            while (p_db) {      // get svr_idx of this svr
                p_st = &p_db->svr;
                if(memcmp(p_st->svc.uuid,p_svr->svc.uuid,16)==0)
                {
                    break;
                }
                p_db = (struct sdp_db *)common_list_next(&p_db->hdr);
                srv_idx++;
            }

            if(p_svr->svc.uuid_type == BK_PERM_RIGHT_UUID_128)
            {
                uuid_len = 16;
            }else
            {
                uuid_len = 2;
            }
            memset(uuid_str,0,33);
            buffer_ptr=uuid_str;
            for(int i=0;i<uuid_len;i++)
            {
                buffer_ptr+=snprintf(buffer_ptr,sizeof(uuid_str)-(buffer_ptr-uuid_str),"%02x",p_svr->svc.uuid[uuid_len-i-1]);
            }
            memset(ble_buf,0,sizeof(ble_buf));
            ble_len = snprintf(ble_buf,sizeof(ble_buf),"+BLEGATTCPRIMSRV:%d,%d,0x%s,1\r\n",con_idx,srv_idx,uuid_str);
            atsvr_output_msg(ble_buf,ble_len);
            atsvr_output_msg("OK\r\n",strlen("OK\r\n"));
        }
            break;
        case SDP_ATT_SVR_ATT_BY_SVR_UUID:
        {
            struct db *p_svr=(struct db *)param;
            uint8_t uuid_len;
            char uuid_str[32+1];
            char *buffer_ptr;
            uint16_t hdl;
            char ble_buf[200];
            int ble_len=0;
            uint8_t srv_idx;
            uint8_t desc_index;
            uint8_t char_end_hdl;
            struct sdp_env_tag * p_env = sdp_get_env_use_conidx(con_idx);
            struct sdp_db *p_db = (struct sdp_db *)common_list_pick(&p_env->svr_list);
            struct db *p_st;

            srv_idx=1;
            while (p_db) {      // get svr_idx of this svr
                p_st = &p_db->svr;
                if(memcmp(p_st->svc.uuid,p_svr->svc.uuid,16)==0)
                {
                    break;
                }
                p_db = (struct sdp_db *)common_list_next(&p_db->hdr);
                srv_idx++;
            }

            for(int i=0;i<p_svr->chars_nb;i++)
            {
                desc_index = 0;
                if(p_svr->chars[i].uuid_type == BK_PERM_RIGHT_UUID_128)
                {
                    uuid_len = 16;
                }
                else
                {
                    uuid_len = 2;
                }
                memset(uuid_str,0,33);
                buffer_ptr=uuid_str;

                for(int j=0;j<uuid_len;j++)
                {
                    buffer_ptr+=snprintf(buffer_ptr,sizeof(uuid_str)-(buffer_ptr-uuid_str),"%02x",p_svr->chars[i].uuid[uuid_len-j-1]);
                }
                memset(ble_buf,0,sizeof(ble_buf));
                ble_len = snprintf(ble_buf,sizeof(ble_buf),"+BLEGATTCCHAR:\"char\",%d,%d,%d,0x%s,0x%x\r\n",con_idx,srv_idx,i+1,uuid_str,p_svr->chars[i].prop);
                atsvr_output_msg(ble_buf,ble_len);

                hdl = p_svr->chars[i].val_hdl+1;
                if (i==p_svr->chars_nb - 1)
                {
                    char_end_hdl = p_svr->svc.end_hdl;
                } else
                {
                    char_end_hdl = p_svr->chars[i+1].val_hdl-2;
                }
                for(int desc_hdl=hdl;desc_hdl<=char_end_hdl;desc_hdl++)
                {
                    for(int des_idx=0;des_idx<p_svr->descs_nb;des_idx++)
                    {
                        if(desc_hdl == p_svr->descs[des_idx].desc_hdl)
                        {
                            desc_index++;
                            if(p_svr->descs[des_idx].uuid_type == BK_PERM_RIGHT_UUID_128)
                            {
                                uuid_len = 16;
                            }else
                            {
                                uuid_len = 2;
                            }
                            memset(uuid_str,0,33);
                            buffer_ptr=uuid_str;
                            for(int j=0;j<uuid_len;j++)
                            {
                                buffer_ptr+=snprintf(buffer_ptr,sizeof(uuid_str)-(buffer_ptr-uuid_str),"%02x",p_svr->descs[des_idx].uuid[uuid_len-j-1]);
                            }

                            memset(ble_buf,0,sizeof(ble_buf));
                            ble_len = snprintf(ble_buf,sizeof(ble_buf),"+BLEGATTCCHAR:\"desc\",%d,%d,%d,%d,0x%s,0x%x\r\n",con_idx,srv_idx,i+1,desc_index,uuid_str,p_svr->descs[des_idx].prop);
                            atsvr_output_msg(ble_buf,ble_len);
                        }
                    }
                }
                atsvr_output_msg("OK\r\n",strlen("OK\r\n"));
            }
        }
            break;
        default:
            break;
    }
}

void at_sdp_event_cb(sdp_notice_t notice, void *param)
{
    switch (notice) {
        case SDP_CHARAC_NOTIFY_EVENT:
            {
                sdp_event_t *g_sdp = (sdp_event_t *)param;
                char ble_buf[50];
                int ble_len=0;
                memset(ble_buf,0,sizeof(ble_buf));
                ble_len = snprintf(ble_buf,sizeof(ble_buf),"+BLEGATTSNTFY:%d,%d",g_sdp->con_idx,g_sdp->value_length);
                ble_len += snprintf(ble_buf+ble_len,sizeof(ble_buf),"\r\n");
                atsvr_output_msg(ble_buf,ble_len);
                atsvr_output_msg("OK\r\n",strlen("OK\r\n"));
            }
            break;
        case SDP_CHARAC_INDICATE_EVENT:
            {
                sdp_event_t *g_sdp = (sdp_event_t *)param;
                char ble_buf[50];
                int ble_len=0;
                memset(ble_buf,0,sizeof(ble_buf));
                ble_len = snprintf(ble_buf,sizeof(ble_buf),"+BLEGATTSIND:%d,%d",g_sdp->con_idx,g_sdp->value_length);
                ble_len += snprintf(ble_buf+ble_len,sizeof(ble_buf),"\r\n");
                atsvr_output_msg(ble_buf,ble_len);
                atsvr_output_msg("OK\r\n",strlen("OK\r\n"));
            }
            break;
        case SDP_CHARAC_READ:
            {
                sdp_event_t *g_sdp = (sdp_event_t *)param;
                char ble_buf[50];
                int ble_len=0;
                memset(ble_buf,0,sizeof(ble_buf));
                ble_len = snprintf(ble_buf,sizeof(ble_buf),"+BLEGATTCRD:%d,%d",g_sdp->con_idx,g_sdp->value_length);
                ble_len += snprintf(ble_buf+ble_len,sizeof(ble_buf),"\r\n");
                atsvr_output_msg(ble_buf,ble_len);
                atsvr_output_msg("OK\r\n",strlen("OK\r\n"));
            }
            break;
        case SDP_DISCOVER_SVR_DONE:
            {
                bk_printf("[SDP_DISCOVER_SVR_DONE]\r\n");
            }
            break;
        case SDP_CHARAC_WRITE_DONE:
            {
                bk_printf("[SDP_CHARAC_WRITE_DONE]\r\n");
            }
            break;
        default:
            bk_printf("[%s]Event:%d\r\n",__func__,notice);
            break;
    }
}

static void at_ble_set_mode(int argc, char **argv)
{
    hal_ble_env.mode=atoi(argv[1]);
    bk_printf("ble mode:%d \r\n",hal_ble_env.mode);
    at_ble_param_debug_printf();

    ble_set_notice_cb(at_ble_notice_cb);
    if(1==hal_ble_env.mode)
    {
        #if BLE_SDP_CLIENT && (CFG_SOC_NAME == SOC_BK7231N)
        register_app_sdp_service_tab(sizeof(service_tab)/sizeof(app_sdp_service_uuid),(app_sdp_service_uuid *)service_tab);
        app_sdp_service_filtration(0);
        register_app_sdp_characteristic_callback(at_ble_app_sdp_characteristic_cb);
        register_app_sdp_charac_callback(at_app_sdp_charac_cb);
        register_app_sdp_common_callback(at_sdp_comm_callback);
        #elif (CFG_SOC_NAME == SOC_BK7238)
        sdp_set_notice_cb(at_sdp_event_cb);
        sdp_set_discovery_svc_cb(at_sdp_discovery_cb);
        uint8_t conn_idx;
        conn_idx = app_ble_get_idle_conn_idx_handle();
        bk_printf("------------->conn_idx:%d\r\n",conn_idx);
        hal_ble_env.sdp_conn_idx = conn_idx;
        bk_ble_create_init(conn_idx, 0, 0, 0,at_ble_cmd_cb);
        #endif
    }

    atsvr_output_msg("OK\r\n",4);

    return;
}

static void at_ble_get_mac(int argc, char **argv)
{
    uint8_t len;
    uint8_t mac[6]={0};

    bk_ble_get_mac(mac);
    bk_printf("get ble mac: %02x:%02x:%02x:%02x:%02x:%02x\r\n",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);

    memset(at_rsp_msg_buf,0,sizeof(at_rsp_msg_buf));
    len=snprintf(at_rsp_msg_buf,sizeof(at_rsp_msg_buf),"+BLEADDR:\"%02x:%02x:%02x:%02x:%02x:%02x\"\r\nOK\r\n",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
    atsvr_output_msg(at_rsp_msg_buf,len);
}

static void at_ble_set_mac(int argc, char **argv)
{
    bk_printf("set mac not support\r\n");
    //bk_ble_set_mac();

    atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
    return;
}

static void at_ble_get_name(int argc, char **argv)
{
    uint8_t len;

    memset(at_rsp_msg_buf,0,sizeof(at_rsp_msg_buf));
    len=snprintf(at_rsp_msg_buf,sizeof(at_rsp_msg_buf),"+BLENAME:\"%s\"\r\nOK\r\n",g_env_param.ble_param.ble_device_name);
    atsvr_output_msg(at_rsp_msg_buf,len);
}

static void at_ble_set_name(int argc, char **argv)
{
    sscanf(argv[1],"\"%[^\"]\"",argv[1]);

    if(strlen(argv[1])>BLE_DEVICE_NAME_SIZE)
    {
        bk_printf("name size too big,set ble name fail\r\n");
        atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
        return;
    }

    memcpy(hal_ble_env.ble_device_name,argv[1],strlen(argv[1]));

    if(g_env_param.sysstore)
    {
        memset(g_env_param.ble_param.ble_device_name,0,BLE_DEVICE_NAME_SIZE);
        memcpy(g_env_param.ble_param.ble_device_name,hal_ble_env.ble_device_name,strlen(argv[1]));
        write_env_to_flash(TAG_SYSSTORE_OFFSET,sizeof(ENV_PARAM),(uint8*)&g_env_param);
    }

    if(ble_appm_set_dev_name(strlen(argv[1]),(uint8_t*)argv[1]))
    {
        bk_printf("set ble name success\r\n");
        atsvr_output_msg("OK\r\n",4);
    }
    else
    {
        bk_printf("set ble name fail\r\n");
        atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
    }

    return;
}

static void at_ble_get_scan_param(int argc, char **argv)
{
    uint8_t len;
    memset(at_rsp_msg_buf,0,sizeof(at_rsp_msg_buf));

    len=snprintf(at_rsp_msg_buf,sizeof(at_rsp_msg_buf),"+BLESCANPARAM:%d,%d,%d,%d,%d\r\nOK\r\n",
    hal_ble_env.scan.scan_type,
    hal_ble_env.scan.own_addr_type,
    hal_ble_env.scan.filter_policy,
    hal_ble_env.scan.scan_intvl,
    hal_ble_env.scan.scan_wd);

    atsvr_output_msg(at_rsp_msg_buf,len);
}

static void at_ble_set_scan_param(int argc, char **argv)
{
    if(argc<6)
    {
        bk_printf("param error\r\n");
        atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
        return;
    }
    uint16_t scan_intv;
    uint16_t scan_wd;
    scan_intv = os_strtoul(argv[4], NULL, 10);
    scan_wd = os_strtoul(argv[5], NULL, 10);
    if (scan_intv < SCAN_INTERVAL_MIN || scan_intv > SCAN_INTERVAL_MAX ||
        scan_wd < SCAN_WINDOW_MIN || scan_wd > SCAN_WINDOW_MAX ||
        scan_intv < scan_wd)
    {
        bk_printf("\nThe four/five param is wrong!\n");
        atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
        return;
    }
    hal_ble_env.scan.scan_type=atoi(argv[1]);
    hal_ble_env.scan.own_addr_type=atoi(argv[2]);
    hal_ble_env.scan.filter_policy=atoi(argv[3]);
    hal_ble_env.scan.scan_intvl=atoi(argv[4]);
    hal_ble_env.scan.scan_wd=atoi(argv[5]);

    if(g_env_param.sysstore)
    {
        g_env_param.ble_param.scan_type=hal_ble_env.scan.scan_type;
        g_env_param.ble_param.own_addr_type=hal_ble_env.scan.own_addr_type;
        g_env_param.ble_param.filter_policy=hal_ble_env.scan.filter_policy;
        g_env_param.ble_param.scan_intvl=hal_ble_env.scan.scan_intvl;
        g_env_param.ble_param.scan_wd=hal_ble_env.scan.scan_wd;
    }


    bk_printf("scan param:%d,%d,%d,%d,%d\r\n",
    hal_ble_env.scan.scan_type,
    hal_ble_env.scan.own_addr_type,
    hal_ble_env.scan.filter_policy,
    hal_ble_env.scan.scan_intvl,
    hal_ble_env.scan.scan_wd);

    atsvr_output_msg("OK\r\n",4);

    return;
}

static void at_ble_enable_scan(int argc, char **argv)
{
    uint8_t len=0;
    uint8_t enable=0;
    uint8_t filter_type=0;
    uint8_t filter_param[32];
    ble_err_t ret = ERR_SUCCESS;

    struct scan_param scan_info;

    enable=atoi(argv[1]);
    if(enable)
    {
        if(hal_ble_env.scan.scan_state)
        {
            bk_printf("ble scan state is 1\r\n");
            return;
        }
        if(argc>=3)
            hal_ble_env.scan.timeout=atoi(argv[2]);

        if(argc>=5)
        {
            filter_type=atoi(argv[3]);
            len=strlen(argv[4])>32?32:strlen(argv[4])+1;
            memcpy(filter_param,argv[4],len);

            if(1==filter_type)
            {
                hal_ble_env.scan.filter_type=1;
                memcpy(hal_ble_env.scan.filter_param,filter_param,len);
            }
            else if(2==filter_type)
            {
                hal_ble_env.scan.filter_type=2;
                memcpy(hal_ble_env.scan.filter_param,filter_param,len);
            }
            else
            {
                hal_ble_env.scan.filter_type=0;
                memset(hal_ble_env.scan.filter_param,0,sizeof(hal_ble_env.scan.filter_param));
            }
        }
        scan_info.channel_map = 7;
        scan_info.interval = hal_ble_env.scan.scan_intvl;
        scan_info.window = hal_ble_env.scan.scan_wd;
        scan_info.filter_type = filter_type;
        memcpy(scan_info.filter_param,hal_ble_env.scan.filter_param,len);

        hal_ble_env.scan.scan_idx = app_ble_get_idle_actv_idx_handle();
        bk_printf("bk_ble_scan_start idx:%d,filter_type:%d,len:%d,%d,%d\r\n",hal_ble_env.scan.scan_idx,scan_info.filter_type,
        len,scan_info.interval,scan_info.window);
        ret = bk_ble_scan_start(hal_ble_env.scan.scan_idx,&scan_info,at_ble_cmd_cb);
        if(ret == ERR_SUCCESS)
            hal_ble_env.scan.scan_state = 1;
    }
    else
    {
        if(!hal_ble_env.scan.scan_state)
        {
            bk_printf("ble scan state is 0\r\n");
            return;
        }
        ret = bk_ble_scan_stop(hal_ble_env.scan.scan_idx,at_ble_cmd_cb);
        if(ret == ERR_SUCCESS)
            hal_ble_env.scan.scan_state = 0;
    }

    atsvr_output_msg("OK\r\n",4);

    return;
}

static void at_ble_server_create(int argc, char **argv)
{
    uint8_t header[4]={0};
    uint16_t data_len=0;
    uint8_t  *data_buf;
    uint8_t att_offset =0;

    BtServer *srv_tmp=NULL;

    bk_flash_read(BK_PARTITION_BLE_SVR_CONFIG,0,header,4);
    bk_printf("ble svr header:0x%x%x%x%x\r\n",header[0],header[1],header[2],header[3]);

    if((header[2])!=0x27 || (header[3])!=0x95)
    {
        atsvr_output_msg("FAIL\r\n",6);
        return;
    }

    data_len= ((header[3]<<8) | header[2])&0xffff;

    bk_printf("ble svr data_len is :%x\r\n",data_len);

    data_buf=os_zalloc(data_len);

    if(!data_buf)
    {
        bk_printf("ble svr malloc fail\r\n");
        atsvr_output_msg("FAIL\r\n",6);
        return;
    }

    if(svr_create_flag)
    {
        bk_printf("ble svr already created\r\n");
        atsvr_output_msg("FAIL\r\n",6);
        return;
    }

    bk_flash_read(BK_PARTITION_BLE_SVR_CONFIG,4,data_buf,data_len);

    bk_printf("ble server data: ");
    for(int i=0;i<data_len;i++)
    {
        bk_printf("%c",data_buf[i]);
    }
    bk_printf("\r\n");

    cJSON *root = NULL,*service = NULL,*index=NULL,*server_list=NULL,
    *uuid=NULL,*uuid_len=NULL,*val_max_len=NULL,*value=NULL,*perm=NULL,*val_cur_len=NULL;

    root = cJSON_Parse((const char *)data_buf);
    service = cJSON_GetObjectItem(root, "Service");

    server_list  = service->child;

    while(NULL!=server_list)
    {

        index =  cJSON_GetObjectItem(server_list, "index");
        bk_printf("index:%d\r\n",index->valueint);

        uuid =  cJSON_GetObjectItem(server_list, "uuid");
        bk_printf("uuid:%s\r\n",uuid->valuestring);

        uuid_len =  cJSON_GetObjectItem(server_list, "uuid_len");
        bk_printf("uuid_len:%d\r\n",uuid_len->valueint);

        val_max_len =  cJSON_GetObjectItem(server_list, "val_max_len");
        bk_printf("val_max_len:%d\r\n",val_max_len->valueint);

        value =  cJSON_GetObjectItem(server_list, "value");
        bk_printf("value:%s\r\n",value->valuestring);

        perm =  cJSON_GetObjectItem(server_list, "perm");
        bk_printf("perm:%d\r\n",perm->valueint);

        val_cur_len =  cJSON_GetObjectItem(server_list, "val_cur_len");
        bk_printf("val_cur_len:%d\r\n",val_cur_len->valueint);

        if(uuid_len->valueint==16 && 0==os_strcmp(uuid->valuestring,"2800"))
        {
            if(srv_db_list.srv_num)
            {
                bk_printf("last server\r\n");
                srv_tmp->att_db=os_zalloc(srv_tmp->max_att_handle*sizeof(bk_attm_desc_t));
                srv_tmp->next=os_zalloc(sizeof(BtServer));
                srv_tmp=srv_tmp->next;
            }
            else
            {
                bk_printf("first server\r\n");
                srv_db_list.srv=os_zalloc(sizeof(BtServer));
                srv_tmp=srv_db_list.srv;
            }

            srv_tmp->uuidLen=val_cur_len->valueint;
            srv_tmp->uuid=os_zalloc(srv_tmp->uuidLen);
            hexstr2bin(value->valuestring,srv_tmp->uuid,srv_tmp->uuidLen);
            srv_tmp->service_id=++srv_db_list.srv_num;
            srv_tmp->start = 0;
            bk_printf("service_id=%d\r\n", srv_tmp->service_id);

            srv_tmp->max_att_handle=0;
        }

        srv_tmp->max_att_handle++;
        server_list=server_list->next;
    }

    srv_tmp->att_db=os_zalloc(srv_tmp->max_att_handle*sizeof(bk_attm_desc_t));

    server_list  = service->child;
    srv_tmp=srv_db_list.srv;

    while(NULL!=server_list)
    {
        index =  cJSON_GetObjectItem(server_list, "index");
        uuid =  cJSON_GetObjectItem(server_list, "uuid");
        uuid_len =  cJSON_GetObjectItem(server_list, "uuid_len");
        val_max_len =  cJSON_GetObjectItem(server_list, "val_max_len");
        value =  cJSON_GetObjectItem(server_list, "value");
        perm =  cJSON_GetObjectItem(server_list, "perm");
        val_cur_len =  cJSON_GetObjectItem(server_list, "val_cur_len");

        if(uuid_len->valueint==16 && 0==strcmp(uuid->valuestring,"2800") && index->valueint >= srv_db_list.srv->max_att_handle)
        {
            srv_tmp=srv_tmp->next;
            att_offset=0;
        }

        if(uuid_len->valueint==16 && 0==strcmp(uuid->valuestring,"2800"))
        {
            memcpy(&srv_tmp->att_db[att_offset++],&attrListPattern[BK_Attr_IDX_SVC],sizeof(bk_attm_desc_t)); //add primary service declartion
        }
        else if(uuid_len->valueint==16 && 0==strcmp(uuid->valuestring,"2803"))
        {
            memcpy(&srv_tmp->att_db[att_offset++],&attrListPattern[BK_Attr_IDX_CHAR],sizeof(bk_attm_desc_t)); ////add char declaration
        }
        else if(uuid_len->valueint==16 && 0==strcmp(uuid->valuestring,"2902"))
        {
            memcpy(&srv_tmp->att_db[att_offset++],&attrListPattern[BK_Attr_IDX_CLIENT_CHAR_CFG],sizeof(bk_attm_desc_t)); //add client char cfg
        }
        else
        {
            srv_tmp->att_db[att_offset].info=perm->valueint;

            if(uuid_len->valueint==128)
                srv_tmp->att_db[att_offset].ext_info=BK_PERM_SET(RI, ENABLE)|BK_PERM_SET(UUID_LEN, UUID_128);
            else
                srv_tmp->att_db[att_offset].ext_info=BK_PERM_SET(RI, ENABLE)|BK_PERM_SET(UUID_LEN, UUID_16);

            hexstr2bin(uuid->valuestring,srv_tmp->att_db[att_offset++].uuid,uuid_len->valueint>>3);

            for(int i=0;i<uuid_len->valueint>>4;i++)
            {
                bk_printf("1uuid[%d]:%02x,uuid[%d]:%02x\r\n",i,srv_tmp->att_db[att_offset-1].uuid[i],(uuid_len->valueint>>3)-1-i,srv_tmp->att_db[att_offset-1].uuid[(uuid_len->valueint>>3)-1-i]);
                uint8_t tmp;
                tmp=srv_tmp->att_db[att_offset-1].uuid[i];
                srv_tmp->att_db[att_offset-1].uuid[i]=srv_tmp->att_db[att_offset-1].uuid[(uuid_len->valueint>>3)-1-i];
                srv_tmp->att_db[att_offset-1].uuid[(uuid_len->valueint>>3)-1-i]=tmp;
                bk_printf("1uuid[%d]:%02x,uuid[%d]:%02x\r\n",i,srv_tmp->att_db[att_offset-1].uuid[i],(uuid_len->valueint>>3)-1-i,srv_tmp->att_db[att_offset-1].uuid[(uuid_len->valueint>>3)-1-i]);
            }
        }
        server_list=server_list->next;
    }

    srv_tmp=srv_db_list.srv;
    for(int i=0;i<srv_db_list.srv_num;i++)
    {
        bk_printf("server[%d]:\r\n",i);
        for(int j=0;j<srv_tmp->max_att_handle;j++)
        {
            bk_printf("perm:%d,ext_perm:%d,uuid:%x:%x\r\n",srv_tmp->att_db[j].info,srv_tmp->att_db[j].ext_info,srv_tmp->att_db[j].uuid[0],srv_tmp->att_db[j].uuid[1]);
        }
        srv_tmp=srv_tmp->next;
    }

    svr_create_flag=1;
    os_free(data_buf);
    atsvr_output_msg("OK\r\n",4);
}

static void at_ble_server_start(int argc, char **argv)
{
    uint8_t srv_index=atoi(argv[1]);
    bk_printf("srv_index:%d\r\n",srv_index);
    if(srv_index)
    {
        at_bk_ble_svr_init(srv_index);
    }else
    {
        for(int i=1;i<=srv_db_list.srv_num;i++)
        {
            at_bk_ble_svr_init(i);
        }
    }
    atsvr_output_msg("OK\r\n",4);
}

static void at_ble_server_stop(int argc, char **argv)
{
    uint8_t svr_index=atoi(argv[1]);
    bk_printf("svr_index:%d\r\n",svr_index);

    atsvr_output_msg("OK\r\n",4);
}

static void at_ble_server_read(int argc, char **argv)
{
    uint16_t len=0;
    char uuid_str[32+1];
    char *buffer_ptr;
    BtServer *srv_tmp=NULL;

    srv_tmp=srv_db_list.srv;

    for(int index=0;index<srv_db_list.srv_num;index++)
    {
        buffer_ptr=uuid_str;

        for(int i=0;i<srv_tmp->uuidLen;i++)
        {
            buffer_ptr+=snprintf(buffer_ptr,sizeof(uuid_str)-(buffer_ptr-uuid_str),"%02x",srv_tmp->uuid[i]);
        }

        len=snprintf(at_rsp_msg_buf,sizeof(at_rsp_msg_buf),"+BLEGATTSSRV:%d,%d,0x%s,1\r\n",
                                                    (srv_tmp->service_id),srv_tmp->start,uuid_str);
        atsvr_output_msg(at_rsp_msg_buf,len);
        srv_tmp=srv_tmp->next;
    }

    atsvr_output_msg("OK\r\n",4);
}

static void at_ble_character_read(int argc, char **argv)
{
    uint16_t len=0;
    char uuid_str[32+1];
    char *buffer_ptr;
    uint8_t uuidLen=0;
    uint8_t att_index=0;
    uint8_t char_index=0;
    uint8_t desc_index=0;
    BtServer *srv_tmp=NULL;

    srv_tmp=srv_db_list.srv;

    for(int i=0;i<srv_db_list.srv_num;i++)
    {
        att_index=0;
        char_index=0;
        while(att_index<srv_tmp->max_att_handle){
            if (srv_tmp->att_db[att_index].uuid[0]==0x03 && srv_tmp->att_db[att_index].uuid[1]==0x28)
            {
                char_index++;
                desc_index = 0;
                att_index++;    //att_index of char value declaration
                if(BK_PERM_RIGHT_UUID_128==BK_PERM_GET(srv_tmp->att_db[att_index].ext_info,UUID_LEN))
                    uuidLen=16;
                else
                    uuidLen=2;

                buffer_ptr=uuid_str;
                for(int i=0;i<uuidLen;i++)
                {
                    buffer_ptr+=snprintf(buffer_ptr,sizeof(uuid_str)-(buffer_ptr-uuid_str),"%02x",srv_tmp->att_db[att_index].uuid[i]);
                }

                len=snprintf(at_rsp_msg_buf,sizeof(at_rsp_msg_buf),"+BLEGATTSCHAR:\"char\",%d,%d,0x%s,0x%x\r\n",
                                                    srv_tmp->service_id,char_index,uuid_str,srv_tmp->att_db[att_index].info);
                atsvr_output_msg(at_rsp_msg_buf,len);

            }else if(srv_tmp->att_db[att_index].uuid[0]<=0x05 && srv_tmp->att_db[att_index].uuid[1]==0x29)
            {
                desc_index++;
                len=snprintf(at_rsp_msg_buf,sizeof(at_rsp_msg_buf),"+BLEGATTSCHAR:\"desc\",%d,%d,%d\r\n",
                                                    srv_tmp->service_id,char_index,desc_index);
                atsvr_output_msg(at_rsp_msg_buf,len);
            }else
            {
                bk_printf("att_uuid:%02x %02x\r\n",srv_tmp->att_db[att_index].uuid[0],srv_tmp->att_db[att_index].uuid[1]);
            }
            att_index++;
        }
        srv_tmp=srv_tmp->next;
    }
    atsvr_output_msg("OK\r\n",4);
}

static void at_ble_set_advdata_ex(int argc, char **argv)
{
    bk_printf("dev_name:%s,uuid:%s,manufacturer_data:%s,include_power:%s\r\n",argv[1],argv[2],argv[3],argv[4]);

    sscanf(argv[1],"\"%[^\"]\"",argv[1]);
    sscanf(argv[2],"\"%[^\"]\"",argv[2]);
    sscanf(argv[3],"\"%[^\"]\"",argv[3]);

    bk_printf("dev_name:%s,uuid:%s,manufacturer_data:%s,include_power:%s\r\n",argv[1],argv[2],argv[3],argv[4]);

    strcpy(hal_ble_env.adv.adv_name,argv[1]);

    bk_printf("argv[2] len:%d\r\n",strlen(argv[2]));

    memset(hal_ble_env.adv.uuid,0x0,16);
    memset(hal_ble_env.adv.manufacturer_data,0x0,16);
    hal_ble_env.adv.uuid_len=strlen(argv[2])/2;
    hal_ble_env.adv.manufacturer_data_len=strlen(argv[3])/2;

    hexstr2bin(argv[2],hal_ble_env.adv.uuid,hal_ble_env.adv.uuid_len);

    hexstr2bin(argv[3],hal_ble_env.adv.manufacturer_data,hal_ble_env.adv.manufacturer_data_len );

    hal_ble_env.adv.include_power=atoi(argv[4]);

    if(g_env_param.sysstore)
    {
        g_env_param.ble_param.uuid_len=hal_ble_env.adv.uuid_len;
        g_env_param.ble_param.manufacturer_data_len=hal_ble_env.adv.manufacturer_data_len;
        memcpy(g_env_param.ble_param.manufacturer_data,hal_ble_env.adv.manufacturer_data,hal_ble_env.adv.manufacturer_data_len);
        memcpy(g_env_param.ble_param.uuid,hal_ble_env.adv.uuid,hal_ble_env.adv.uuid_len);

        write_env_to_flash(TAG_SYSSTORE_OFFSET,sizeof(g_env_param),(uint8*)&g_env_param);
    }

    struct adv_param adv_info;
    adv_info.channel_map = 7;
    adv_info.duration = 0;
    adv_info.prop = (1 << ADV_PROP_CONNECTABLE_POS) | (1 << ADV_PROP_SCANNABLE_POS);
    adv_info.interval_min = 160;
    adv_info.interval_max = 160;
    uint8_t adv_offset=0;

    //name
    adv_info.advData[adv_offset++] = strlen(hal_ble_env.adv.adv_name)+1;
    adv_info.advData[adv_offset++] = 0x09;
    memcpy(&adv_info.advData[adv_offset], hal_ble_env.adv.adv_name, strlen(hal_ble_env.adv.adv_name));

    adv_offset+=strlen(hal_ble_env.adv.adv_name);

    //uuid
    if(hal_ble_env.adv.uuid_len==16)
    {
        adv_info.advData[adv_offset++] = 16+1;
        adv_info.advData[adv_offset++] = 0x07;
        memcpy(&adv_info.advData[adv_offset], hal_ble_env.adv.uuid, 16);
        adv_offset+=16;
    }
    else if(hal_ble_env.adv.uuid_len==2)
    {
        adv_info.advData[adv_offset++] = 2+1;
        adv_info.advData[adv_offset++] = 0x03;
        adv_info.advData[adv_offset++]=hal_ble_env.adv.uuid[1];
        adv_info.advData[adv_offset++]=hal_ble_env.adv.uuid[0];
    }

    adv_info.advData[adv_offset++] = hal_ble_env.adv.manufacturer_data_len+1;
    adv_info.advData[adv_offset++] =0xff;
    memcpy(&adv_info.advData[adv_offset], hal_ble_env.adv.manufacturer_data,hal_ble_env.adv.manufacturer_data_len);
    adv_offset+=hal_ble_env.adv.manufacturer_data_len;

    adv_info.advDataLen = adv_offset;

    bk_printf("adv_info.advDataLen:%d\r\n",adv_info.advDataLen);
    bk_printf("advData:");
    for(int i=0;i<adv_info.advDataLen;i++)
    {
        bk_printf("%02x ",adv_info.advData[i]);
    }
    bk_printf("\r\n");

    adv_info.respData[0] = (strlen(hal_ble_env.adv.adv_name)<4?strlen(hal_ble_env.adv.adv_name):4) +1;
    adv_info.respData[1] = 0x08;

    memcpy(&adv_info.respData[2], hal_ble_env.adv.adv_name, adv_info.respData[0]-1);
    adv_info.respDataLen = adv_info.respData[0]+1;

    hal_ble_env.adv.adv_idx = app_ble_get_idle_actv_idx_handle();
    bk_ble_adv_start(hal_ble_env.adv.adv_idx, &adv_info, at_ble_cmd_cb);

    atsvr_output_msg("OK\r\n",4);
}

static void at_ble_get_advdata_ex(int argc, char **argv)
{
    uint16_t len;
    char uuid_str[32+1];
    char *buffer_ptr;
    char manufacturer_data_str[32];

    buffer_ptr=uuid_str;
    for(int i=0;i<hal_ble_env.adv.uuid_len;i++)
    {
        buffer_ptr+=snprintf(buffer_ptr,sizeof(uuid_str)-(buffer_ptr-uuid_str),"%02x",hal_ble_env.adv.uuid[i]);
    }

    buffer_ptr=manufacturer_data_str;
    for(int i=0;i<hal_ble_env.adv.manufacturer_data_len;i++)
    {
        buffer_ptr+=snprintf(buffer_ptr,sizeof(manufacturer_data_str)-(buffer_ptr-manufacturer_data_str),"%02x",hal_ble_env.adv.manufacturer_data[i]);
    }
    bk_printf("dev_name:%s,uuid:%s,manufacturer_data:%s,include_power:%d\r\n",hal_ble_env.adv.adv_name,uuid_str,manufacturer_data_str,hal_ble_env.adv.include_power);

    len=snprintf(at_rsp_msg_buf,sizeof(at_rsp_msg_buf),"+BLEADVDATAEX:\"%s\",\"%s\",\"%s\",%d\r\nOK\r\n",
    hal_ble_env.adv.adv_name,
    uuid_str,
    manufacturer_data_str,
    hal_ble_env.adv.include_power);

    atsvr_output_msg(at_rsp_msg_buf,len);
}

static void at_ble_set_adv_rspdata(int argc, char **argv)
{
    if(argc!=2)
    {
        atsvr_cmd_rsp_error();
    }
    sscanf(argv[1],"\"%[^\"]\"",argv[1]);
    hal_ble_env.adv.respDataLen=strlen(argv[1])/2;
    memset(hal_ble_env.adv.respData,0,sizeof(hal_ble_env.adv.respData));
    hexstr2bin(argv[1],hal_ble_env.adv.respData,hal_ble_env.adv.respDataLen);

    if(g_env_param.sysstore)
    {
        g_env_param.ble_param.respDataLen=hal_ble_env.adv.respDataLen;
        memcpy(g_env_param.ble_param.respData,hal_ble_env.adv.respData,hal_ble_env.adv.respDataLen);
        write_env_to_flash(TAG_SYSSTORE_OFFSET,sizeof(g_env_param),(uint8*)&g_env_param);
    }

    atsvr_output_msg("OK\r\n",4);
}

static void at_ble_set_adv_data(int argc, char **argv)
{
    if(argc!=2)
    {
        atsvr_cmd_rsp_error();
    }
    sscanf(argv[1],"\"%[^\"]\"",argv[1]);

    hal_ble_env.adv.advDataLen=strlen(argv[1])/2;
    memset(hal_ble_env.adv.advData,0,sizeof(hal_ble_env.adv.advData));
    hexstr2bin(argv[1],hal_ble_env.adv.advData,hal_ble_env.adv.advDataLen);

    if(g_env_param.sysstore)
    {
        memcpy(g_env_param.ble_param.advData,hal_ble_env.adv.advData,hal_ble_env.adv.advDataLen);
        write_env_to_flash(TAG_SYSSTORE_OFFSET,sizeof(g_env_param),(uint8*)&g_env_param);
    }

    atsvr_output_msg("OK\r\n",4);
}

static void at_ble_set_adv_param(int argc, char **argv)
{
    hal_ble_env.adv.interval_min=atoi(argv[1]);
    hal_ble_env.adv.interval_max=atoi(argv[2]);
    hal_ble_env.adv.adv_type=atoi(argv[3]);
    hal_ble_env.adv.channel_map=atoi(argv[4]);

    if(g_env_param.sysstore)
    {
        g_env_param.ble_param.interval_min=hal_ble_env.adv.interval_min;
        g_env_param.ble_param.interval_max=hal_ble_env.adv.interval_max;
        g_env_param.ble_param.adv_type=hal_ble_env.adv.adv_type;
        g_env_param.ble_param.channel_map=hal_ble_env.adv.channel_map;

        write_env_to_flash(TAG_SYSSTORE_OFFSET,sizeof(g_env_param),(uint8*)&g_env_param);
    }

    atsvr_output_msg("OK\r\n",4);
}

static void at_ble_get_adv_param(int argc, char **argv)
{
    uint16_t len;

    len=snprintf(at_rsp_msg_buf,sizeof(at_rsp_msg_buf),"+BLEADVPARAM:%d,%d,%d,%d\r\nOK\r\n",
    hal_ble_env.adv.interval_min,
    hal_ble_env.adv.interval_max,
    hal_ble_env.adv.adv_type,
    hal_ble_env.adv.channel_map);

    atsvr_output_msg(at_rsp_msg_buf,len);
}

static void at_ble_start_adv(int argc, char **argv)
{
    struct adv_param adv_info;
    adv_info.channel_map = hal_ble_env.adv.channel_map;
    adv_info.duration = 0;
    adv_info.prop = (1 << ADV_PROP_CONNECTABLE_POS) | (1 << ADV_PROP_SCANNABLE_POS);
    adv_info.interval_min = hal_ble_env.adv.interval_min;
    adv_info.interval_max = hal_ble_env.adv.interval_max;

    memcpy(adv_info.advData,hal_ble_env.adv.advData,hal_ble_env.adv.advDataLen);
    adv_info.advDataLen = hal_ble_env.adv.advDataLen;

    memcpy(adv_info.respData,hal_ble_env.adv.respData,hal_ble_env.adv.respDataLen);
    adv_info.respDataLen = hal_ble_env.adv.respDataLen;

    if(app_ble_actv_state_find(ACTV_ADV_STARTED)!=UNKNOW_ACT_IDX)
    {
        bk_printf("adv started\n");
    }else if(app_ble_actv_state_find(ACTV_ADV_CREATED)!=UNKNOW_ACT_IDX)
    {
        bk_ble_start_advertising(app_ble_actv_state_find(ACTV_ADV_CREATED),0,at_ble_cmd_cb);
    }else
    {
        hal_ble_env.adv.adv_idx = app_ble_get_idle_actv_idx_handle();
        bk_ble_adv_start(hal_ble_env.adv.adv_idx, &adv_info, at_ble_cmd_cb);
    }
    atsvr_output_msg("OK\r\n",4);
}

static void at_ble_stop_adv(int argc, char **argv)
{
    bk_ble_adv_stop(hal_ble_env.adv.adv_idx, at_ble_cmd_cb);
    atsvr_output_msg("OK\r\n",4);
}

static void at_ble_get_con_param(int argc, char **argv)
{
    uint8_t len=0;

    if(hal_ble_env.sdp_conn_idx==0xff)
    {
        bk_printf("ble not connected\r\n");
        atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
        return;
    }

    len=snprintf(at_rsp_msg_buf,sizeof(at_rsp_msg_buf),"+BLECONNPARAM:%d,%d,%d,%d,%d,%d\r\nOK\r\n",
    hal_ble_env.sdp_conn_idx,
    hal_ble_env.conn.con_interval,
    hal_ble_env.conn.con_interval,
    hal_ble_env.conn.con_interval,
    hal_ble_env.conn.con_latency,
    hal_ble_env.conn.sup_to);

    atsvr_output_msg(at_rsp_msg_buf,len);
}

static void at_ble_set_con_param(int argc, char **argv)
{
    if(hal_ble_env.sdp_conn_idx==0xff)
    {
        bk_printf("ble not connected\r\n");
        atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
        return;
    }
    bk_printf("hal_ble_env.sdp_conn_idx=%d\r\n",hal_ble_env.sdp_conn_idx);

    if(argc<6)
    {
        bk_printf("param error\r\n");
        atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
        return;
    }

    if(g_env_param.sysstore)
    {
        g_env_param.ble_param.con_interval=atoi(argv[3]);
        g_env_param.ble_param.con_latency=atoi(argv[4]);
        g_env_param.ble_param.sup_to=atoi(argv[5]);
    }

    bk_ble_update_param(hal_ble_env.sdp_conn_idx,atoi(argv[2]),atoi(argv[3]),atoi(argv[4]),atoi(argv[5]),at_ble_cmd_cb);

    atsvr_output_msg("OK\r\n",4);
}

void at_sdp_comm_callback(MASTER_COMMON_TYPE type, uint8 conidx, void *param)
{
    uint16_t len=0;
    char uuid_str[32+1];
    char *buffer_ptr;

    switch (type) {
        case MST_TYPE_ATTC_SVR_UUID:
            bk_printf("MST_TYPE_ATTC_SVR_UUID\r\n");
            struct mst_sdp_svc_ind *svr=(struct mst_sdp_svc_ind *)param;

            buffer_ptr=uuid_str;
            for(int i=0;i<svr->uuid_len;i++)
            {
                buffer_ptr+=snprintf(buffer_ptr,sizeof(uuid_str)-(buffer_ptr-uuid_str),"%02x",svr->uuid[svr->uuid_len-1-i]);
            }

            len=snprintf(at_rsp_msg_buf,sizeof(at_rsp_msg_buf),"+BLEGATTCPRIMSRV:%d,%d,0x%s,1\r\n",
                                                        hal_ble_env.sdp_conn_idx,svr->svr_id,uuid_str);
            atsvr_output_msg(at_rsp_msg_buf,len);

            bk_printf("svrid:%d,handle:%d-%d,uuid_len:%d,uuid:%02x %02x\r\n",svr->svr_id,svr->start_hdl,svr->end_hdl,svr->uuid_len,svr->uuid[0],svr->uuid[1]);

            break;

        case MST_TYPE_ATTC_ATT_UUID:

            bk_printf("MST_TYPE_ATTC_ATT_UUID\r\n");
            struct mst_sdp_char_inf *att=(struct mst_sdp_char_inf *)param;

            buffer_ptr=uuid_str;
            for(int i=0;i<att->uuid_len;i++)
            {
                buffer_ptr+=snprintf(buffer_ptr,sizeof(uuid_str)-(buffer_ptr-uuid_str),"%02x",att->uuid[att->uuid_len-1-i]);
            }

            len=snprintf(at_rsp_msg_buf,sizeof(at_rsp_msg_buf),"+BLEGATTCCHAR:\"char\",%d,%d,%d,0x%s,0x%x\r\n",
                                                                        hal_ble_env.sdp_conn_idx,att->svr_id,
                                                                        att->val_hdl,uuid_str,att->prop);

            atsvr_output_msg(at_rsp_msg_buf,len);
            bk_printf("svrid:%d,handle:%d-%d,uuid_len:%d,uuid:%02x %02x\r\n",att->svr_id,att->char_hdl,att->val_hdl,att->uuid_len,att->uuid[0],att->uuid[1]);
            break;

        case MST_TYPE_ATTC_ATT_DESC:
            bk_printf("MST_TYPE_ATTC_ATT_DESC\r\n");
            struct mst_sdp_char_desc_inf *desc=(struct mst_sdp_char_desc_inf *)param;

            buffer_ptr=uuid_str;
            for(int i=0;i<desc->uuid_len;i++)
            {
                buffer_ptr+=snprintf(buffer_ptr,sizeof(uuid_str)-(buffer_ptr-uuid_str),"%02x",desc->uuid[desc->uuid_len-1-i]);
            }

            len=snprintf(at_rsp_msg_buf,sizeof(at_rsp_msg_buf),"+BLEGATTCCHAR:\"desc\",%d,%d,%d,%d,0x%s\r\n",
                                                                        hal_ble_env.sdp_conn_idx,desc->svr_id,
                                                                        desc->desc_hdl,desc->desc_hdl,uuid_str);

            atsvr_output_msg(at_rsp_msg_buf,len);

            bk_printf("svrid:%d,handle:%d-%d,uuid_len:%d,uuid:%02x %02x\r\n",desc->svr_id,desc->char_code,desc->desc_hdl,desc->uuid_len,desc->uuid[0],desc->uuid[1]);
            break;

        case MST_TYPE_ATTC_END:
            bk_printf("MST_TYPE_ATTC_END\r\n");
            if(hal_ble_env.sdp_svr_ing)
            {
                atsvr_output_msg("OK\r\n",4);
                hal_ble_env.sdp_svr_ing=0;
            }

            if(hal_ble_env.sdp_char_ing)
            {
                atsvr_output_msg("OK\r\n",4);
                hal_ble_env.sdp_char_ing=0;
            }
            break;

        case MST_TYPE_MTU_EXC:
            bk_printf("MST_TYPE_MTU_EXC\r\n");
            break;

        case MST_TYPE_MTU_EXC_DONE:
            bk_printf("MST_TYPE_MTU_EXC_DONE\r\n");
            break;

        case MST_TYPE_UPP_ASK:
            bk_printf("MST_TYPE_UPP_ASK\r\n");
            break;

        case MST_TYPE_INIT_CREATE_OK:
            bk_printf("MST_TYPE_INIT_CREATE_OK\r\n");
            break;

        default:
            break;
    }
}

static void at_ble_connected_callback(uint8_t conn_idx, uint8_t *remote_address)
{
    uint16_t len;

    len=snprintf(at_rsp_msg_buf,sizeof(at_rsp_msg_buf),"+BLECONN:%d,\"%02x:%02x:%02x:%02x:%02x:%02x\r\nOK\r\n",
    conn_idx,
    remote_address[0], remote_address[1], remote_address[2],
    remote_address[3], remote_address[4], remote_address[5]);

    memcpy(hal_ble_env.remote_address,remote_address,6);
    hal_ble_env.conn.con_state = 1;
    atsvr_output_msg(at_rsp_msg_buf,len);
}

static void at_ble_connect_timeout_callback(uint8_t conn_idx)
{
    uint16_t len;

    len=snprintf(at_rsp_msg_buf,sizeof(at_rsp_msg_buf),"+BLECONN:%d,-1\r\nERROR\r\n",conn_idx);

    atsvr_output_msg(at_rsp_msg_buf,len);
}

static void at_ble_start_connect(int argc, char **argv)
{
    struct bd_addr bdaddr;
    unsigned char addr_type = ADDR_PUBLIC;

    if(argc>3)
    {
        addr_type = atoi(argv[3]);
    }

    sscanf(argv[2],"\"%[^\"]\"",argv[2]);
    hexstr2bin(argv[2], bdaddr.addr, GAP_BD_ADDR_LEN);

    bk_printf("------------->conn_idx:%d--%d\r\n",hal_ble_env.sdp_conn_idx,atoi(argv[1]));

    bk_ble_init_set_connect_dev_addr(hal_ble_env.sdp_conn_idx,&bdaddr,addr_type);
    bk_ble_init_start_conn(hal_ble_env.sdp_conn_idx,at_ble_cmd_cb);

    bk_printf("idx:%d, addr_type:%d\r\n",hal_ble_env.sdp_conn_idx,addr_type);

    atsvr_output_msg("OK\r\n",4);
}

static void at_ble_get_con_info(int argc, char **argv)
{
    uint16_t len;
    if(hal_ble_env.sdp_conn_idx == 0xff)
    {
        bk_printf("ble not connected\r\n");
        atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
        return;
    }
    if(!hal_ble_env.conn.con_state)
    {
        len=snprintf(at_rsp_msg_buf,sizeof(at_rsp_msg_buf),"+BLECONN:%d,-1\r\nERROR\r\n",hal_ble_env.sdp_conn_idx);
        atsvr_output_msg(at_rsp_msg_buf,len);
        return;
    }
    len=snprintf(at_rsp_msg_buf,sizeof(at_rsp_msg_buf),"+BLECONN:%d,\"%02x:%02x:%02x:%02x:%02x:%02x\r\nOK\r\n",
    hal_ble_env.sdp_conn_idx,
    hal_ble_env.remote_address[0], hal_ble_env.remote_address[1], hal_ble_env.remote_address[2],
    hal_ble_env.remote_address[3], hal_ble_env.remote_address[4], hal_ble_env.remote_address[5]);

    atsvr_output_msg(at_rsp_msg_buf,len);
}

static void at_ble_disconnect_callback(uint8_t conn_idx)
{
    uint16_t len;

    len=snprintf(at_rsp_msg_buf,sizeof(at_rsp_msg_buf),"+BLEDISCONN:%d,\"%02x:%02x:%02x:%02x:%02x:%02x\r\n",
    conn_idx,
    hal_ble_env.remote_address[0],hal_ble_env. remote_address[1], hal_ble_env.remote_address[2],
    hal_ble_env.remote_address[3], hal_ble_env.remote_address[4], hal_ble_env.remote_address[5]);
    hal_ble_env.conn.con_state = 0;

    atsvr_output_msg(at_rsp_msg_buf,len);
}

static void at_ble_stop_connect(int argc, char **argv)
{
    app_ble_master_appm_disconnect(atoi(argv[1]));
    atsvr_output_msg("OK\r\n",4);
}

static void at_ble_set_pkt_size(int argc, char **argv)
{
    if(hal_ble_env.sdp_conn_idx==0xff)
    {
        bk_printf("ble not connected\r\n");
        atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
        return;
    }
    if(argc < 3)
    {
        bk_printf("param error\r\n");
        atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
        return;
    }
    if(atoi(argv[2]) > LE_MAX_OCTETS || atoi(argv[2]) < LE_MIN_OCTETS)
    {
        bk_printf("pkt_data_len error\r\n");
        atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
        return;
    }
    app_ble_set_le_pkt_size(atoi(argv[1]),atoi(argv[2]));
    atsvr_output_msg("OK\r\n",4);
}

static void at_ble_get_con_mtu(int argc, char **argv)
{
    uint8_t len=0;
    uint16_t mtu[1]={0};
    uint8_t conn_index=hal_ble_env.sdp_conn_idx;

    if(hal_ble_env.sdp_conn_idx==0xff)
    {
        bk_printf("ble not connected\r\n");
        atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
        return;
    }

    bk_ble_get_con_mtu(conn_index,mtu);

    len=snprintf(at_rsp_msg_buf,sizeof(at_rsp_msg_buf),"+BLECFGMTU:%d,%d\r\nOK\r\n",
    hal_ble_env.sdp_conn_idx,mtu[0]);

    atsvr_output_msg(at_rsp_msg_buf,len);

}

static void at_ble_set_con_mtu(int argc, char **argv)
{
    if(hal_ble_env.sdp_conn_idx==0xff)
    {
        bk_printf("ble not connected\r\n");
        atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
        return;
    }

    uint8_t conn_index=atoi(argv[1]);
    uint16_t mtu=atoi(argv[2]);
    bk_printf("mtu:%d\r\n",mtu);

    bk_ble_gatt_mtu_change(conn_index,at_ble_cmd_cb);

    atsvr_output_msg("OK\r\n",4);

}

uint8_t at_ble_get_att_index_of_char_value(uint8_t srv_index, uint8_t char_index)
{
    BtServer *srv_tmp=NULL;
    uint8_t char_idx;
    uint8_t att_idx=0;

    srv_tmp=srv_db_list.srv;
    for(int i=0;i<srv_db_list.srv_num;i++)
    {
        if(srv_tmp->service_id == srv_index)
        {
            char_idx = 0;
            for(int j=0;j<srv_tmp->max_att_handle;j++)
            {
                if (srv_tmp->att_db[j].uuid[0]==0x03 && srv_tmp->att_db[j].uuid[1]==0x28)
                {
                    char_idx++;
                    if(char_idx == char_index){
                        att_idx = j+1;//char value declaration after char declaration
                        break;
                    }
                }
            }
            break;
        }
        srv_tmp=srv_tmp->next;
    }
    return att_idx;
}

static void at_ble_send_notofy(int argc, char **argv)
{
    if(hal_ble_env.connidx==0xff)
    {
        bk_printf("ble not conencted\r\n");
        atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
        return;
    }

    if(argc<5)
    {
        bk_printf("param error\r\n");
        atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
        return;
    }

    uint8_t srv_index=atoi(argv[2]);
    uint8_t char_index=atoi(argv[3]);
    uint16_t length=atoi(argv[4]);
    uint8_t att_index;
    char data[255];
    unsigned int read_len=0;

    if(length>255 || length<1)
    {
        bk_printf("data length ranges from 1 to 255\r\n");
        atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
        return;
    }

    atsvr_output_msg(">",1);
    read_len=atsvr_input_msg_get(data,length);

    for(int i=0;i<read_len;i++)
    {
        bk_printf("%02x",data[i]);
    }
    bk_printf("\r\nread_len:%d start notofy to master\r\n",read_len);

    att_index=at_ble_get_att_index_of_char_value(srv_index, char_index);
    if(!att_index) {
        atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
        return;
    }
    if(ERR_SUCCESS != bk_ble_conidx_send_ntf(hal_ble_env.connidx, read_len, (unsigned char *)data, srv_index-1, att_index))
    {
        atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
        return;
    }

    atsvr_output_msg("OK\r\n",4);
}

static void at_ble_send_indicate(int argc, char **argv)
{
    if(hal_ble_env.connidx==0xff)
    {
        bk_printf("ble not conencted\r\n");
        atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
        return;
    }

    if(argc<5)
    {
        bk_printf("param error\r\n");
        atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
        return;
    }

    uint8_t srv_index=atoi(argv[2]);
    uint8_t char_index=atoi(argv[3]);
    uint16_t length=atoi(argv[4]);
    uint8_t att_index;
    uint8_t data[255];
    int read_len=0;

    if(length>255 || length<1)
    {
        bk_printf("data length ranges from 1 to 255\r\n");
        atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
        return;
    }

    atsvr_output_msg(">",1);
    read_len=atsvr_input_msg_get((char *)data,length);

    for(int i=0;i<read_len;i++)
    {
        bk_printf("%02x",data[i]);
    }
    bk_printf("\r\nread_len:%d start indicate to master\r\n",read_len);

    att_index=at_ble_get_att_index_of_char_value(srv_index, char_index);
    if(!att_index) {
        atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
        return;
    }
    if(ERR_SUCCESS != bk_ble_conidx_send_ind(hal_ble_env.connidx, read_len, data, srv_index-1, att_index))
    {
        atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
        return;
    }

    atsvr_output_msg("OK\r\n",4);
}

static void at_ble_character_desc_set(int argc, char **argv)
{
    os_printf("no support\r\n");
    atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
}

void ble_get_all_char(uint8_t conn_hdl,uint8_t svr_id)
{
    struct sdp_att_event_t msg_event;

    os_memset(&msg_event, 0, sizeof(msg_event));
    msg_event.type = SDP_ATT_SVR_ATT_BY_SVR_UUID;
    msg_event.svr_id=svr_id;

    sdp_get_att_infor(conn_hdl, &msg_event);
}

static void ble_get_all_service(uint8_t conn_hdl)
{
    struct sdp_att_event_t msg_event;

    os_memset(&msg_event, 0, sizeof(msg_event));
    msg_event.type = SDP_ATT_GET_SVR_UUID_ALL;

    sdp_get_att_infor(conn_hdl, &msg_event);
}

static void at_ble_gattc_get_primsrv(int argc, char **argv)
{
    hal_ble_env.sdp_svr_ing=1;
    ble_get_all_service(hal_ble_env.sdp_conn_idx);
}

static void at_ble_gattc_get_char(int argc, char **argv)
{
    if(argc<3)
    {
        bk_printf("param error\r\n");
        atsvr_output_msg("ERROR\r\n",strlen("ERROR\r\n"));
        return;
    }

    hal_ble_env.sdp_char_ing=1;
    bk_printf("at_ble_gattc_get_char svr:%d\r\n",atoi(argv[2]));
    ble_get_all_char(hal_ble_env.sdp_conn_idx,atoi(argv[2]));
}

static void at_ble_gattc_get_inclsrv(int argc, char **argv)
{
    bk_printf("no support\r\n");
    atsvr_output_msg("ERROR\r\n",strlen("ERROR\r\n"));
}

static void at_ble_gattc_get_value(int argc, char **argv)
{
    if(hal_ble_env.sdp_conn_idx==0xff)
    {
        bk_printf("ble not connected\r\n");
        atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
        return;
    }
    char ret = -1;
    int att_hdl=0;
    if(argc<4)
    {
        bk_printf("param error\r\n");
        atsvr_output_msg("ERROR\r\n",strlen("ERROR\r\n"));
        return;
    }

    if(argc == 4)
    {
        att_hdl = sdp_get_att_hdl(atoi(argv[1]),atoi(argv[2]),atoi(argv[3]),0);
    }
    else
    {
        att_hdl = sdp_get_att_hdl(atoi(argv[1]),atoi(argv[2]),atoi(argv[3]),atoi(argv[4]));
    }

    if(att_hdl != 0xffff)
    {
        ret = bk_ble_read_service_data_by_handle_req(atoi(argv[1]),att_hdl,at_ble_cmd_cb);
    }
    if(ret == 0)
    {
        //atsvr_output_msg("OK\r\n",strlen("OK\r\n"));
    }
    else
    {
        atsvr_output_msg("ERROR\r\n",strlen("ERROR\r\n"));
    }
}

static void at_ble_gattc_set_value(int argc, char **argv)
{
    if(hal_ble_env.sdp_conn_idx==0xff)
    {
        bk_printf("ble not connected\r\n");
        atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
        return;
    }

    uint16 data_len=0;
    int read_len=0;
    uint8_t data[255];
    char ret = -1;
    uint8_t srv_idx;
    uint8_t char_idx;
    uint8_t desc_idx=0;
    uint8_t hdl;
    if(argc==5)
    {
        data_len=atoi(argv[4]);
    }
    else if(argc==6)
    {
        desc_idx=atoi(argv[4]);
        data_len=atoi(argv[5]);
    }

    if(data_len>255 || data_len<1)
    {
        bk_printf("data length ranges from 1 to 255\r\n");
        atsvr_output_msg("ERROR\r\n",strlen("ERROR"));
        return;
    }
    srv_idx=atoi(argv[2]);
    char_idx=atoi(argv[3]);
    atsvr_output_msg(">",1);
    read_len=atsvr_input_msg_get((char *)data,data_len);

    for(int i=0;i<read_len;i++)
    {
        bk_printf("%02x",data[i]);
    }
    bk_printf("\r\nread_len:%d start write to slave\r\n",read_len);

    hdl=sdp_get_att_hdl(atoi(argv[1]),srv_idx,char_idx,desc_idx);
    ret = bk_ble_write_service_data_req(atoi(argv[1]),hdl,read_len,data,at_ble_cmd_cb);
    if(ret == 0)
    {
        atsvr_output_msg("OK\r\n",strlen("OK\r\n"));
    }
    else
    {
        atsvr_output_msg("ERROR\r\n",strlen("ERROR\r\n"));
    }
}

static void at_ble_no_support_func(int argc, char **argv)
{
    bk_printf("no support\r\n");
    atsvr_output_msg("ERROR\r\n",strlen("ERROR\r\n"));
}

const struct _atsvr_command at_ble_cmds_table[] = {

    /**************** BLE AT commands ***************/
    ATSVR_CMD_HADLER("AT+BLEINIT?","AT+BLEINIT?",at_ble_get_mode),
    ATSVR_CMD_HADLER("AT+BLEINIT","AT+BLEINIT=",at_ble_set_mode),
    ATSVR_CMD_HADLER("AT+BLEADDR?","AT+BLEADDR?",at_ble_get_mac),
    ATSVR_CMD_HADLER("AT+BLEADDR","AT+BLEADDR=",at_ble_set_mac),
    ATSVR_CMD_HADLER("AT+BLENAME?","AT+BLENAME?",at_ble_get_name),
    ATSVR_CMD_HADLER("AT+BLENAME","AT+BLENAME=",at_ble_set_name),

    //slave
    // ADV
    ATSVR_CMD_HADLER("AT+BLESCANRSPDATA","AT+BLESCANRSPDATA=<scan_rsp_data>",at_ble_set_adv_rspdata),
    ATSVR_CMD_HADLER("AT+BLEADVDATA","AT+BLEADVDATA=<adv_data>",at_ble_set_adv_data),
    ATSVR_CMD_HADLER("AT+BLEADVPARAM?","AT+BLEADVPARAM?",at_ble_get_adv_param),
    ATSVR_CMD_HADLER("AT+BLEADVPARAM","AT+BLEADVPARAM=adv_int_min>,<adv_int_max>,<adv_type>,<channel_map>",at_ble_set_adv_param),
    ATSVR_CMD_HADLER("AT+BLEADVSTART","AT+BLEADVSTART>",at_ble_start_adv),
    ATSVR_CMD_HADLER("AT+BLEADVSTOP","AT+BLEADVSTOP>",at_ble_stop_adv),
    ATSVR_CMD_HADLER("AT+BLEADVDATAEX?","AT+BLEADVDATAEX?",at_ble_get_advdata_ex),
    ATSVR_CMD_HADLER("AT+BLEADVDATAEX","AT+BLEADVDATAEX=<dev_name>,<uuid>,<manufacturer_data>,<include_power>",at_ble_set_advdata_ex),

    //gatts SERVER
    ATSVR_CMD_HADLER("AT+BLEGATTSSRVCRE","AT+BLEGATTSSRVCRE",at_ble_server_create),
    ATSVR_CMD_HADLER("AT+BLEGATTSSRVSTART","AT+BLEGATTSSRVSTART=<srv_index>",at_ble_server_start),
    ATSVR_CMD_HADLER("AT+BLEGATTSSRVSTOP","AT+BLEGATTSSRVSTOP=<srv_index>",at_ble_server_stop),
    ATSVR_CMD_HADLER("AT+BLEGATTSSRV?","AT+BLEGATTSSRV?",at_ble_server_read),
    ATSVR_CMD_HADLER("AT+BLEGATTSCHAR?","AT+BLEGATTSCHAR?",at_ble_character_read),
    ATSVR_CMD_HADLER("AT+BLEGATTSSETATTR","AT+BLEGATTSSETATTR=<srv_index>,<char_index>,[<desc_index>],<length>",at_ble_character_desc_set),

    // NTF && IND
    ATSVR_CMD_HADLER("AT+BLEGATTSNTFY","AT+BLEGATTSNTFY=<conn_index>,<srv_index>,<char_index>,<length>",at_ble_send_notofy),
    ATSVR_CMD_HADLER("AT+BLEGATTSIND","AT+BLEGATTSIND=<conn_index>,<srv_index>,<char_index>,<length>",at_ble_send_indicate),

    //SCAN
    ATSVR_CMD_HADLER("AT+BLESCANPARAM?","AT+BLESCANPARAM?",at_ble_get_scan_param),
    ATSVR_CMD_HADLER("AT+BLESCANPARAM","AT+BLESCANPARAM=<scan_type>,<own_addr_type>,<filter_policy>,<scan_interval>,<scan_window>",at_ble_set_scan_param),
    ATSVR_CMD_HADLER("AT+BLESCAN","AT+BLESCAN=<enable>[,<interval>][,<filter_type>,<filter_param>]",at_ble_enable_scan),

    //CON PARAM
    ATSVR_CMD_HADLER("AT+BLECONNPARAM?","AT+BLECONNPARAM?",at_ble_get_con_param),
    ATSVR_CMD_HADLER("AT+BLECONNPARAM","AT+BLECONNPARAM=<conn_index>,<min_interval>,<max_interval>,<latency>,<timeout>",at_ble_set_con_param),

    //master
    ATSVR_CMD_HADLER("AT+BLECONN","AT+BLECONN=<conn_index>,<remote_address>[,<addr_type>,<timeout>]",at_ble_start_connect),
    ATSVR_CMD_HADLER("AT+BLECONN?","AT+BLECONN?",at_ble_get_con_info),
    ATSVR_CMD_HADLER("AT+BLEDISCONN","AT+BLEDISCONN=<conn_index>",at_ble_stop_connect),
    ATSVR_CMD_HADLER("AT+BLEDATALEN","AT+BLEDATALEN=<conn_index>,<pkt_data_len>",at_ble_set_pkt_size),
    ATSVR_CMD_HADLER("AT+BLECFGMTU?","AT+BLECFGMTU?",at_ble_get_con_mtu),
    ATSVR_CMD_HADLER("AT+BLECFGMTU","AT+BLECFGMTU=<conn_index>,<mtu_size>",at_ble_set_con_mtu),
    ATSVR_CMD_HADLER("AT+BLEGATTCPRIMSRV","AT+BLEGATTCPRIMSRV=<conn_index>",at_ble_gattc_get_primsrv),
    ATSVR_CMD_HADLER("AT+BLEGATTCINCLSRV","AT+BLEGATTCINCLSRV=<conn_index>,<srv_index>",at_ble_gattc_get_inclsrv),
    ATSVR_CMD_HADLER("AT+BLEGATTCCHAR","AT+BLEGATTCCHAR=<conn_index>,<srv_index>",at_ble_gattc_get_char),
    ATSVR_CMD_HADLER("AT+BLEGATTCRD","AT+BLEGATTCRD=<conn_index>,<srv_index>,<char_index>[,<desc_index>]",at_ble_gattc_get_value),
    ATSVR_CMD_HADLER("AT+BLEGATTCWR","AT+BLEGATTCWR=<conn_index>,<srv_index>,<char_index>[,<desc_index>],<length>",at_ble_gattc_set_value),


    //no support at cmd
    ATSVR_CMD_HADLER("AT+BLESPPCFG?","AT+BLESPPCFG?",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLESPPCFG","AT+BLESPPCFG=",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLESPP","AT+BLESPP",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLESECPARAM?","AT+BLESECPARAM?",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLESECPARAM","AT+BLESECPARAM=",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLEENC","AT+BLEENC=<conn_index>,<sec_act>",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLEENCRSP","AT+BLEENCRSP=<conn_index>,<accept>",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLEKEYREPLY","AT+BLEKEYREPLY=<conn_index>,<key>",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLECONFREPLY","AT+BLECONFREPLY=<conn_index>,<confirm>",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLEENCDEV","AT+BLEENCDEV",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLEENCCLEAR","AT+BLEENCCLEAR=<enc_dev_index>",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLESETKEY?","AT+BLESETKEY?",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLESETKEY","AT+BLESETKEY=<static_key>",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLEHIDINIT?","AT+BLEHIDINIT?",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLEHIDINIT","AT+BLEHIDINIT=<init>",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLEHIDKB","AT+BLEHIDKB=<key_1>,<key_2>,<key_3>,<key_4>,<key_5>,<key_6>",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLEHIDMUS","AT+BLEHIDMUS=<buttons>,<X_displacement>,<Y_displacement>,<wheel>",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLEHIDCONSUMER","AT+BLEHIDCONSUMER=<consumer_usage_id>",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLUFI?","AT+BLUFI?",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLUFI","AT+BLUFI=<option>[,<auth floor>]",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLUFINAME?","AT+BLUFINAME?",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLUFINAME","AT+BLUFINAME=<device_name>",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLEPERIODICDATA","AT+BLEPERIODICDATA=<periodic_data>",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLEPERIODICSTART","AT+BLEPERIODICSTART",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLEPERIODICSTOP","AT+BLEPERIODICSTOP",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLESYNCSTART","AT+BLESYNCSTART=<target_address>",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLESYNCSTOP","AT+BLESYNCSTOP",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLEREADPHY","AT+BLEREADPHY==<conn_index>",at_ble_no_support_func),
    ATSVR_CMD_HADLER("AT+BLESETPHY","AT+BLESETPHY==<conn_index>,<tx_rx_phy>",at_ble_no_support_func),

    /****************  BLE AT commands end ***************/
};

static void at_ble_param_debug_printf(void)
{
    if(g_env_param.sysstore)
    {
        bk_printf("ble_device_name:%s\r\n",g_env_param.ble_param.ble_device_name);
        bk_printf("adv_type:%d\r\n",g_env_param.ble_param.adv_type);
        bk_printf("interval_min:%d\r\n",g_env_param.ble_param.interval_min);
        bk_printf("interval_max:%d\r\n",g_env_param.ble_param.interval_max);
        bk_printf("channel_map;%d\r\n",g_env_param.ble_param.channel_map);
        bk_printf("uuid_len:%d\r\n",g_env_param.ble_param.uuid_len);
        bk_printf("uuid:%x%x\r\n",g_env_param.ble_param.uuid[0],g_env_param.ble_param.uuid[1]);

        bk_printf("manufacturer_data_len:%d\r\n",g_env_param.ble_param.manufacturer_data_len);
        bk_printf("manufacturer_data:%x:%x\r\n",g_env_param.ble_param.manufacturer_data[0],g_env_param.ble_param.manufacturer_data[1]);
        bk_printf("include_power:%d\r\n",g_env_param.ble_param.include_power);
        bk_printf("respDataLen:%d\r\n",g_env_param.ble_param.respDataLen);

        bk_printf("scan_type:%d\r\n",g_env_param.ble_param.scan_type);
        bk_printf("scan_wd:%d\r\n",g_env_param.ble_param.scan_wd);
        bk_printf("own_addr_type:%d\r\n",g_env_param.ble_param.own_addr_type);
        bk_printf("filter_policy:%d\r\n",g_env_param.ble_param.filter_policy);
        bk_printf("scan_intvl:%d\r\n",g_env_param.ble_param.scan_intvl);
        bk_printf("con_interval:%d\r\n",g_env_param.ble_param.con_interval);
        bk_printf("con_latency:%d\r\n",g_env_param.ble_param.con_latency);
        bk_printf("sup_to:%d\r\n",g_env_param.ble_param.sup_to);

        bk_printf("advData:");
        for(int i=0;i<hal_ble_env.adv.advDataLen;i++)
        {
            bk_printf("%02x",hal_ble_env.adv.advData[i]);
        }
        bk_printf("\r\n");
        bk_printf("respData:");
        for(int i=0;i<hal_ble_env.adv.respDataLen;i++)
        {
            bk_printf("%02x",hal_ble_env.adv.respData[i]);
        }
        bk_printf("\r\n");
    }
}

void at_ble_cmd_init(void)
{
    if(g_env_param.sysstore)
    {
        hal_ble_env.adv.channel_map=g_env_param.ble_param.channel_map;
        hal_ble_env.adv.interval_min=g_env_param.ble_param.interval_min;
        hal_ble_env.adv.interval_max=g_env_param.ble_param.interval_max;
        hal_ble_env.adv.adv_type=g_env_param.ble_param.adv_type;
        hal_ble_env.adv.advDataLen=g_env_param.ble_param.advDataLen;
        memcpy(hal_ble_env.adv.advData,g_env_param.ble_param.advData,g_env_param.ble_param.advDataLen);

        hal_ble_env.adv.uuid_len=g_env_param.ble_param.uuid_len;
        hal_ble_env.adv.manufacturer_data_len=g_env_param.ble_param.manufacturer_data_len;
        memcpy(hal_ble_env.adv.manufacturer_data,g_env_param.ble_param.manufacturer_data,g_env_param.ble_param.manufacturer_data_len);
        memcpy(hal_ble_env.adv.uuid,g_env_param.ble_param.uuid,g_env_param.ble_param.uuid_len);

        hal_ble_env.adv.respDataLen=g_env_param.ble_param.respDataLen;
        memcpy(hal_ble_env.adv.respData,g_env_param.ble_param.respData,g_env_param.ble_param.respDataLen);

        hal_ble_env.scan.scan_type=g_env_param.ble_param.scan_type;
        hal_ble_env.scan.own_addr_type=g_env_param.ble_param.own_addr_type;
        hal_ble_env.scan.filter_policy=g_env_param.ble_param.filter_policy;
        hal_ble_env.scan.scan_intvl=g_env_param.ble_param.scan_intvl;
        hal_ble_env.scan.scan_wd=g_env_param.ble_param.scan_wd;

        hal_ble_env.conn.con_interval = g_env_param.ble_param.con_interval;
        hal_ble_env.conn.con_latency = g_env_param.ble_param.con_latency;
        hal_ble_env.conn.sup_to = g_env_param.ble_param.sup_to;

        hal_ble_env.sdp_conn_idx=0xff;
        hal_ble_env.scan.scan_state = 0;
        hal_ble_env.connidx=0xff;
        hal_ble_env.conn.con_state = 0;

        memcpy(hal_ble_env.ble_device_name,g_env_param.ble_param.ble_device_name,strlen(g_env_param.ble_param.ble_device_name));
    }
    atsvr_register_commands(at_ble_cmds_table, sizeof(at_ble_cmds_table) / sizeof(struct _atsvr_command));
}
