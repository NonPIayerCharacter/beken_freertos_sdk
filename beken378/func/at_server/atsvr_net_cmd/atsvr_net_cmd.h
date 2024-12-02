#ifndef __ATSVR_NET_CNMD_H_
#define __ATSVR_NET_CNMD_H_

#include "network_interface.h"
#if CFG_USE_TCPUDP
typedef struct _pm_socket_info
{
	int linkid;
	char remoteip[16];
	int port;
	NETWORK_TYPE type;
	int keepalive;
	int udpmode;
	char localip[16];
	unsigned int localport;
	int linktype; //0:clinet 1:server
} pm_socket_info_t;

void _atsvr_net_cmd_init(_atsvr_env_t *env);
void _atsvr_net_cmd_deinit(_atsvr_env_t *env);

extern beken_queue_t network_msg_que;
#endif
#endif