#include "sockets.h"
#include "_atsvr_func.h"
#include "_at_server.h"
#include "atsvr_comm.h"
#include "string.h"
#include "sockets.h"
#include "mem_pub.h"
#include "BkDriverFlash.h"
#include "sys.h"
#include "manual_ps_pub.h"
#include "atsvr_misc.h"
#include "atsvr_port.h"
#include "atsvr_net_cmd.h"
#include "network_interface.h"
#include "atsvr_param_check.h"
#include "atsvr_error.h"
#include "network_app.h"
#include "at_server.h"
#include "atsvr_wlan.h"
//#include "lwip/netdb.h"
#if CFG_USE_TCPUDP
beken_queue_t network_msg_que;

#define MAX_ITEM_COUNT      (15)
#define MAX_SOCKET_LINK     (6)
ATNetwork at_network_mg[6];
ThreadParams g_thread_manage[MAX_THREAD_NUM];

int g_thread_index = 0;

int network_creat_thread(beken_thread_function_t       callback, IN void* para, char *threadname)
{
    strcpy(g_thread_manage[g_thread_index].thread_name,threadname);
    g_thread_manage[g_thread_index].user_arg =  para;
    g_thread_manage[g_thread_index].stack_size = 4096;
    g_thread_manage[g_thread_index].priority = THD_APPLICATION_PRIORITY;
    g_thread_manage[g_thread_index].thread_func = callback;

    if(HAL_ThreadCreate( &g_thread_manage[g_thread_index]) ==ATSVR_RET_SUCCESS){
        g_thread_index++;
        return kNoErr;
    }

    return kGeneralErr;
}

int network_stop_thread(int linkid)
{
    char linkname[64];
    int i = 0;
    sprintf(linkname,"NET_LINK%d",linkid);

    for( i = 0; i < g_thread_index; i++)
    {
        if((strcmp(g_thread_manage[i].thread_name,linkname)==0) && (g_thread_manage[i].thread_id !=0)) {
            //rtos_thread_join(g_thread_manage[i].pthreadID);
            at_network_mg[linkid].threadrunflag= FALSE;
            // rtos_delete_thread(g_thread_manage[i].thread_id);
            ATSVRLOG("\r\nDELETE THREAD SUCCESS(%s) \r\n",linkname);
            break;
        }
    }
    if(i >= g_thread_index)
        return kGeneralErr;

    return kNoErr;
}

int network_connected_num()
{
    int connectcnt = 0;
    for(int i = 0; i< MAX_SOCKET_LINK; i++ )
    {
        if(at_network_mg[i].handle > 0){
            connectcnt++;
        }
    }

    return connectcnt;
}

int isconnected_network(int linkid)
{
   return at_network_mg[linkid].handle ? 1:0;
}

int network_check_connect_type(int linkid,NETWORK_TYPE contype)
{
     return (at_network_mg[linkid].type == contype) ? 1: 0;
}

int network_link_status()
{
    int n = 0;
    char resultbuf[200]={0};

    if(network_connected_num()) {
        for(int i = 0; i< MAX_SOCKET_LINK; i++ )
        {
            if(isconnected_network(i)) {
                n = snprintf(resultbuf,sizeof(resultbuf),"+BIPSTATUS:%d,connected, %d, %s, %d,%d,%d\r\n",
                i,at_network_mg[i].type,at_network_mg[i].host,at_network_mg[i].port,at_network_mg[i].localport,at_network_mg[i].is_server);
                atsvr_output_msg(resultbuf,n);
            }
        }
    } else {
        n = snprintf(resultbuf,sizeof(resultbuf),"+BIPSTATUS:ALL disconnect\r\n");
        atsvr_output_msg(resultbuf,n);
    }

    return kNoErr;
}

int network_passthrough_send_msg(unsigned char* sendbuff,int len)
{
    unsigned int writelen = 0;
    for(int i = 0; i< MAX_SOCKET_LINK; i++)
    {
        if(at_network_mg[i].handle > 0){
            ATSVRLOG("network_passthrough_send_msg len:%d\n",len);
            at_network_mg[i].netwrite(&at_network_mg[i],sendbuff,len,3000,&writelen);
            break;
        }
    }

    return writelen;
}

int network_tcp_send_msg(int linkid, unsigned char* sendbuff,int len)
{
    unsigned int writelen = 0;

    if(at_network_mg[linkid].handle >= 0) {
        sendbuff[len]='\0';
        at_network_mg[linkid].netwrite(&at_network_mg[linkid],sendbuff,len,3000,&writelen);
        ATSVRLOG("linkid:%d,send :%s,writelen:%d,len=%d\n",linkid,sendbuff,writelen,len);
    }

    return writelen;
}

int network_udp_send_msg(int linkid, unsigned char* sendbuff,int len,char*remoteip, int port)
{
    unsigned int writelen = 0;

    if(at_network_mg[linkid].handle >= 0){
        sendbuff[len]='\0';
        writelen = HAL_UDP_WriteTo(at_network_mg[linkid].handle, sendbuff, len, remoteip, port);
        ATSVRLOG("linkid:%d,send :%s\n",linkid,sendbuff);
    }

    return writelen;
}

#if 1
int network_set_keep_alive(uintptr_t socket_fd,int keepalive)
{
    int opt = 1;
    int ret = 0;
    int fd = socket_fd - LWIP_SOCKET_FD_SHIFT;

    /*****close keepalive******/
    if(keepalive == 0){
        opt = 1;
        return kNoErr;
    }

    // open tcp keepalive
    ret = setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &opt, sizeof(int));
    if(ret < 0)
    {
        return kGeneralErr;
    }

    opt = keepalive;
    ret = setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &opt, sizeof(int));
    if(ret < 0)
    {
        return kGeneralErr;
    }

    opt = 1;  // 1s second for intval
    ret = setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &opt, sizeof(int));
    if(ret < 0)
    {
        return kGeneralErr;
    }

    opt = 3;  // 3 times
    ret = setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &opt, sizeof(int));
    if(ret < 0)
    {
        return kGeneralErr;
    }

    return kNoErr;
}
#endif

extern int wlan_get_sta_localip(char * localip);
void setup_msg_network(ATNetwork *pATNetwork , pm_socket_info_t msg)
{
    POINTER_SANITY_CHECK_RTN(pATNetwork);
    strncpy((char *)pATNetwork->host,msg.remoteip,16);
    pATNetwork->port = msg.port;
    pATNetwork->type = msg.type;
    pATNetwork->is_server = 0;
    pATNetwork->keepalive = msg.keepalive;
    pATNetwork->udpmode = msg.udpmode;

    /*if(pATNetwork->type == NETWORK_TLS || pATNetwork->type == NETWORK_DTLS){
    pATNetwork->ssl_connect_params.psk =  g_env_param.deviceinfo.psk_secret;
    pATNetwork->ssl_connect_params.psk_length = strlen(g_env_param.deviceinfo.psk_secret);
    }*/

    strcpy(pATNetwork->localip,msg.localip);
    pATNetwork->localport = msg.localport;
    if(judge_the_string_is_ipv4_string(pATNetwork->localip)!=0)
    {
        wlan_get_sta_localip(pATNetwork->localip);
    }

    if(pATNetwork->localport != 0 && (judge_the_string_is_ipv4_string(pATNetwork->localip)==0))
    {
        pATNetwork->is_server = 1;
    }

    ATSVRLOG("setup_msg_network host %s,port:%d \r\n",pATNetwork->host,pATNetwork->port);
}

static void network_datarev_thread( beken_thread_arg_t data )
{
    #define REV_NETWORK_DATA_LEN    2048
    int ret;
    unsigned char revbuf[REV_NETWORK_DATA_LEN];
    int timeout = 1000;
    size_t readlen;
    int linkid = *(int*)data;
    //int linkid = (int)threadpara->data;
    ATSVRLOG("enter link %d rev thread\r\n",linkid);

    while ( at_network_mg[linkid].threadrunflag == TRUE){
        if(at_network_mg[linkid].is_connected(&at_network_mg[linkid]) > 0){
            memset(revbuf,0,REV_NETWORK_DATA_LEN);
            if((ret = at_network_mg[linkid].netread(&at_network_mg[linkid],revbuf,REV_NETWORK_DATA_LEN,timeout,&readlen)) == ATSVR_RET_SUCCESS){
                int n = 0;
                char resultbuf[200];
                //n = snprintf(resultbuf,sizeof(resultbuf),"+IPD,%d:%s\r\n",readlen,revbuf);
                n = snprintf(resultbuf,sizeof(resultbuf),"+IPD(%d),%d:",linkid,readlen);
                atsvr_output_msg((char *)resultbuf,n);
                atsvr_output_msg((char *)revbuf,readlen);
            }
            else if(at_network_mg[linkid].type == NETWORK_TCP ||  at_network_mg[linkid].type == NETWORK_TLS){
                if(ret == ATSVR_ERR_TCP_READ_FAIL || ret == ATSVR_ERR_TCP_PEER_SHUTDOWN ||  ret == ATSVR_ERR_SSL_READ){
                    at_network_mg[linkid].disconnect(&at_network_mg[linkid]);
                }
            }
        }else{
            at_network_mg[linkid].netconnect(&at_network_mg[linkid]);
            if(at_network_mg[linkid].is_connected(&at_network_mg[linkid]) > 0){
                struct sockaddr_in loc_addr;
                socklen_t len =sizeof(loc_addr);
                memset(&loc_addr,0,len);
                if(getsockname(at_network_mg[linkid].handle,(struct sockaddr *)&loc_addr,&len)==0){
                    at_network_mg[linkid].localport = ntohs(loc_addr.sin_port);
                }
                if(at_network_mg[linkid].type == NETWORK_TCP ||  at_network_mg[linkid].type == NETWORK_TLS){
                    network_set_keep_alive(at_network_mg[linkid].handle,at_network_mg[linkid].keepalive);
                }
                if(g_atsvr_status.network_transfer_mode != ATSVR_PASSTHROUGH_MODE){
                    atsvr_cmd_rsp_ok();
                }
            }else{
                if(g_atsvr_status.network_transfer_mode != ATSVR_PASSTHROUGH_MODE){
                    atsvr_cmd_rsp_error();
                    goto end;
                }
            }
            rtos_thread_msleep(200);
        }
    }

    if(at_network_mg[linkid].handle){
        at_network_mg[linkid].disconnect((ATNetwork *)&at_network_mg[linkid]);
        ATSVRLOG("\r\n network_datarev_thread(NET_LINK%d) exit \r\n",linkid);
        at_network_mg[linkid].handle = 0;
    }

end:
    rtos_delete_thread(NULL);
}

static void network_msg_thread( beken_thread_arg_t data )
{
    rtos_delay_milliseconds( 1000 );
    UINT32 msg_timeout_ms = BEKEN_WAIT_FOREVER;
    int ret;
    pm_socket_info_t msg;
    char threadname[64];

    while (1)
    {
        ret = rtos_pop_from_queue(&network_msg_que, &msg, msg_timeout_ms);

        if(kNoErr == ret)
        {
            ATSVRLOG("rev: msg.type:%d,linkid:%d\r\n",msg.type,msg.linkid);
            setup_msg_network(&at_network_mg[msg.linkid],msg);
            interace_network_init(&at_network_mg[msg.linkid]);
            sprintf(threadname,"NET_LINK%d",msg.linkid);
            at_network_mg[msg.linkid].threadrunflag = TRUE;
            network_creat_thread(network_datarev_thread,&msg.linkid,threadname);

            #if 0
            switch (msg.type)
            {
                case NETWORK_TCP:
                    network_creat_thread(network_tcp_datarev_thread,msg.linkid,threadname);
                    break;

                case NETWORK_UDP:
                    network_creat_thread(network_udp_datarev_thread,msg.linkid,threadname);
                    break;

                case NETWORK_TLS:
                    network_creat_thread(network_tls_datarev_thread,msg.linkid,threadname);
                    break;

                case NETWORK_DTLS:
                    break;

                default:
                    break;
            }
            #endif
        }else{
            goto exit;
        }
    }

exit:
    if(network_msg_que)
    {
        rtos_deinit_queue(&network_msg_que);
    }
    rtos_delete_thread( NULL );
}

int create_network_task()
{
    memset(g_thread_manage,0,sizeof(g_thread_manage));

    int ret = rtos_init_queue(&network_msg_que,
                            "network_queue",
                            sizeof(pm_socket_info_t),
                            MAX_ITEM_COUNT);

    if (kNoErr != ret){
        ATSVRLOG("atsvr_network_task_init ceate queue failed\r\n");
        return kGeneralErr;
    }

    rtos_create_thread(NULL,
                        THD_APPLICATION_PRIORITY,
                        "net_task",
                        (beken_thread_function_t)network_msg_thread,
                        4096,
                        (beken_thread_arg_t)0);

    /*rtos_create_thread(NULL,
    THD_APPLICATION_PRIORITY,
    "net_rev",
    (beken_thread_function_t)network_datarev_thread,
    4096,
    (beken_thread_arg_t)0);*/

    return kNoErr;
}
#endif
