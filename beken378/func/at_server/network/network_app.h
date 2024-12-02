#ifndef __NETWORK_APP_H_
#define __NETWORK_APP_H_
#include "typedef.h"
#include "network_interface.h"

#define  MAX_THREAD_NUM           10

#ifndef TRUE
#define  TRUE           1
#endif

#ifndef FALSE
#define  FALSE           0
#endif
#if CFG_USE_TCPUDP
extern int network_stop_thread(int linkid);
extern int network_creat_thread(beken_thread_function_t       callback, IN void* para, char *threadname);
extern int network_connected_num();
extern int isconnected_network(int linkid);
int network_check_connect_type(int linkid,NETWORK_TYPE contype);
extern int network_tcp_send_msg(int linkid, unsigned char* sendbuff,int len);
int network_udp_send_msg(int linkid, unsigned char* sendbuff,int len,char*remoteip, int port);
extern int network_passthrough_send_msg(unsigned char* sendbuff,int len);
#endif
#endif
