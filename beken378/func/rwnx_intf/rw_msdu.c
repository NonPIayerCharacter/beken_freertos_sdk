#include "include.h"
#include "doubly_list.h"
#include "rw_msdu.h"
#include "rw_pub.h"
#include "str_pub.h"
#include "mem_pub.h"
#include "txu_cntrl.h"
#include "rxu_cntrl.h"
#include "llc.h"

#include "lwip/pbuf.h"
#ifdef CFG_WFA_CERTIFICATION
#include "prot/ip4.h"
#include "prot/ip6.h"
#include "prot/ethernet.h"
#endif

#if CFG_WIFI_TX_KEYDATA_USE_LOWEST_RATE
#include "lwip/prot/ip4.h"
#include "lwip/prot/ethernet.h"
#include "lwip/prot/ip.h"
#include "lwip/prot/udp.h"
#include "lwip/prot/dhcp.h"
#endif

#include "arm_arch.h"
#if CFG_GENERAL_DMA
#include "general_dma_pub.h"
#endif
#include "param_config.h"
#include "fake_clock_pub.h"
#include "power_save_pub.h"
#include "ps.h"
#if CFG_TX_BUFING
#include "rw_tx_buffering.h"
#endif
#include "includes.h"
#include "common.h"
#include "list.h"
#include "l2_packet/l2_packet.h"

#if CFG_WIFI_TX_KEYDATA_USE_LOWEST_RATE
#ifndef ETH_TYPE_EAPOL
#define ETH_TYPE_EAPOL	   0x888E
#endif
#endif

void ethernetif_input(int iface, struct pbuf *p);
UINT32 rwm_transfer_node(MSDU_NODE_T *node, u16 flag);
extern int bmsg_ps_handler_rf_ps_mode_real_wakeup(void);
UINT8 rwn_mgmt_is_valid_sta(struct sta_info_tag *sta);

LIST_HEAD_DEFINE(msdu_rx_list);


UINT8 g_tid = 0xFF;

void rwm_push_rx_list(MSDU_NODE_T *node)
{
    GLOBAL_INT_DECLARATION();

    GLOBAL_INT_DISABLE();
    list_add_tail(&node->hdr, &msdu_rx_list);
    GLOBAL_INT_RESTORE();
}

MSDU_NODE_T *rwm_pop_rx_list(void)
{
    LIST_HEADER_T *tmp;
    LIST_HEADER_T *pos;
    MSDU_NODE_PTR node;

    GLOBAL_INT_DECLARATION();

    GLOBAL_INT_DISABLE();

    node = NULLPTR;
    list_for_each_safe(pos, tmp, &msdu_rx_list)
    {
        list_del(pos);
        node = list_entry(pos, MSDU_NODE_T, hdr);

        break;
    }

    GLOBAL_INT_RESTORE();

    return node;
}

void rwm_flush_rx_list(void)
{
    MSDU_NODE_T *node_ptr;

    while(1)
    {
        node_ptr = rwm_pop_rx_list();
        if(node_ptr)
        {
            os_free(node_ptr);
        }
        else
        {
            break;
        }
    }
}

void rwm_tx_confirm(void *param)
{
	struct txdesc *txdesc = (struct txdesc *)param;

	if(txdesc && txdesc->host.msdu_node)
	{
#if CFG_NX_SOFTWARE_TX_RETRY
		if(rwn_check_sw_tx_retry(txdesc))
		{
			return;
		}
#endif
		if(txdesc->host.callback)
		{
			(*txdesc->host.callback)(txdesc->host.param);
		}
		os_null_printf("flush_desc:0x%x\r\n", txdesc->host.msdu_node);

		os_free(txdesc->host.msdu_node);
		txdesc->host.msdu_node = NULL;
		txdesc->status = TXDESC_STA_IDLE;
	}
}

void rwm_tx_msdu_renew(UINT8 *buf, UINT32 len, UINT8 *orig_addr)
{
#if CFG_GENERAL_DMA && (CFG_SOC_NAME != SOC_BK7231N)
    gdma_memcpy((void *)((UINT32)orig_addr + CFG_MSDU_RESV_HEAD_LEN), buf, len);
#else
    os_memmove((void *)((UINT32)orig_addr + CFG_MSDU_RESV_HEAD_LEN), buf, len);
#endif
}

UINT8 *rwm_get_msdu_content_ptr(MSDU_NODE_T *node)
{
    return (UINT8 *)((UINT32)node->msdu_ptr + CFG_MSDU_RESV_HEAD_LEN);
}

void rwm_txdesc_copy(struct txdesc *dst_local, ETH_HDR_PTR eth_hdr_ptr)
{
    struct hostdesc *host_ptr;

    host_ptr = &dst_local->host;

    os_memcpy(&host_ptr->eth_dest_addr, &eth_hdr_ptr->e_dest, sizeof(host_ptr->eth_dest_addr));
    os_memcpy(&host_ptr->eth_src_addr, &eth_hdr_ptr->e_src, sizeof(host_ptr->eth_src_addr));
}

void * rwm_raw_frame_prepare_space(uint8_t *buffer, int length)
{
	MSDU_NODE_T *node;
	uint8_t *pkt = buffer;

	node = rwm_tx_node_alloc(length);
	if (node == NULL) {
		bk_printf("rwm_raw_frame_prepare_space failed\r\n");
		goto prepare_exit;
	}

	rwm_tx_msdu_renew(pkt, length, node->msdu_ptr);
prepare_exit:

	return node;
}

int rwm_raw_frame_destroy_space(void *node)
{
	if(node)
	{
		rwm_node_free((MSDU_NODE_T *)node);
	}

	return 0;
}

int rwm_raw_frame_with_cb(uint8_t *buffer, int len, void *cb, void *param)
{
	int ret = 0;
	MSDU_NODE_T *node;
	UINT8 *content_ptr;
	UINT32 queue_idx = AC_VI;
	struct txdesc *txdesc_new;
	struct umacdesc *umac;

	/*the input parameter :buffer is the node pointer*/
	node = (MSDU_NODE_T *)buffer;
	content_ptr = rwm_get_msdu_content_ptr(node);

	txdesc_new = tx_txdesc_prepare(queue_idx);
	if(txdesc_new == NULL || TXDESC_STA_USED == txdesc_new->status) {
		rwm_node_free(node);

		os_printf("rwm_raw_frame_with_cb failed\r\n");
		ret = -1;
		goto exit;
	}

	txdesc_new->status = TXDESC_STA_USED;
	txdesc_new->host.flags = TXU_CNTRL_MGMT;
	txdesc_new->host.orig_addr = (UINT32)node->msdu_ptr;
	txdesc_new->host.packet_addr = (UINT32)content_ptr;
#if CFG_NX_SOFTWARE_TX_RETRY
	// record total retry times, used for rate contrl in low mac
	txdesc_new->host.status_desc_addr = 0;
	txdesc_new->host.access_category = queue_idx;
	txdesc_new->host.flags |= TXU_CNTRL_EN_SW_RETRY_CHECK;
	txdesc_new->lmac.hw_desc->thd.statinfo = 0;
#else
	txdesc_new->host.status_desc_addr = (UINT32)content_ptr;
#endif
	txdesc_new->host.packet_len = len;
	txdesc_new->host.tid = 0xff;
	txdesc_new->host.callback = (mgmt_tx_cb_t)cb;
#if CFG_BK_AWARE
	txdesc_new->host.param = txdesc_new;
#else
	txdesc_new->host.param = param;
#endif
	txdesc_new->host.msdu_node = (void *)node;

	umac = &txdesc_new->umac;
	umac->payl_len = len;
	umac->head_len = 0;
	umac->tail_len = 0;
	umac->hdr_len_802_2 = 0;

	umac->buf_control = &txl_buffer_control_24G;

	txdesc_new->lmac.agg_desc = NULL;
	txdesc_new->lmac.hw_desc->cfm.status = 0;

	ps_set_data_prevent();
	nxmac_pwr_mgt_setf(0);

	bmsg_ps_handler_rf_ps_mode_real_wakeup();

	txl_cntrl_push(txdesc_new, queue_idx);

	ret = len;

exit:
	return ret;
}

MSDU_NODE_T *rwm_tx_node_alloc(UINT32 len)
{
    UINT8 *buff_ptr;
    MSDU_NODE_T *node_ptr = 0;
#if (CFG_SUPPORT_RTT) && (CFG_SOC_NAME == SOC_BK7221U)
    extern void *dtcm_malloc(size_t size);
    node_ptr = (MSDU_NODE_T *)dtcm_malloc(sizeof(MSDU_NODE_T)
                                        + CFG_MSDU_RESV_HEAD_LEN
                                        + len
                                        + CFG_MSDU_RESV_TAIL_LEN);
#elif (CFG_OS_FREERTOS) && (CFG_SOC_NAME == SOC_BK7221U)
    node_ptr = (MSDU_NODE_T *)pvPortMalloc(sizeof(MSDU_NODE_T)
                                        + CFG_MSDU_RESV_HEAD_LEN
                                        + len
                                        + CFG_MSDU_RESV_TAIL_LEN);
#else
    node_ptr = (MSDU_NODE_T *)os_malloc(sizeof(MSDU_NODE_T)
                                        + CFG_MSDU_RESV_HEAD_LEN
                                        + len
                                        + CFG_MSDU_RESV_TAIL_LEN);
#endif

    if(NULL == node_ptr)
    {
        goto alloc_exit;
    }

    buff_ptr = (UINT8 *)((UINT32)node_ptr + sizeof(MSDU_NODE_T));

    node_ptr->msdu_ptr = buff_ptr;
    node_ptr->len = len;

alloc_exit:
    return node_ptr;
}

void rwm_node_free(MSDU_NODE_T *node)
{
    ASSERT(node);
    os_free(node);
}

UINT8 *rwm_rx_buf_alloc(UINT32 len)
{
    return (UINT8 *)os_malloc(len);
}

UINT32 rwm_get_rx_valid(void)
{
    UINT32 count = 0;
    LIST_HEADER_T *tmp;
    LIST_HEADER_T *pos;
    LIST_HEADER_T *head = &msdu_rx_list;

    GLOBAL_INT_DECLARATION();

    GLOBAL_INT_DISABLE();
    list_for_each_safe(pos, tmp, head)
    {
        count ++;
    }
    GLOBAL_INT_RESTORE();

    return ((count >= MSDU_RX_MAX_CNT) ? 0 : 1);
}

UINT32 rwm_get_rx_valid_node_len(void)
{
    UINT32 len = 0;
    LIST_HEADER_T *tmp;
    LIST_HEADER_T *pos;
    MSDU_NODE_PTR node;

    GLOBAL_INT_DECLARATION();

    GLOBAL_INT_DISABLE();

    node = NULLPTR;
    list_for_each_safe(pos, tmp, &msdu_rx_list)
    {
        node = list_entry(pos, MSDU_NODE_T, hdr);
        len = node->len;
        break;
    }

    GLOBAL_INT_RESTORE();

    return len;
}

#if !CFG_RWNX_QOS_MSDU
UINT8 rwm_get_tid()
{
    return g_tid;
}

void rwm_set_tid(UINT8 tid)
{
    if (0xFF == tid)
    {
        g_tid = tid;
    }
    else
    {
        g_tid = tid & MAC_QOSCTRL_UP_MSK;
    }
}
#endif // !CFG_RWNX_QOS_MSDU

#if CFG_USE_P2P_PS
void rwm_p2p_ps_change_ind_handler(void *msg)
{
    struct ke_msg *msg_ptr = (struct ke_msg *)msg;
#if CFG_WIFI_P2P
    struct mm_p2p_vif_ps_change_ind *ind;
#endif

    if(!msg_ptr || !msg_ptr->param)
        return;

#if CFG_WIFI_P2P
    ind = (struct mm_p2p_vif_ps_change_ind *)msg_ptr->param;
    if (ind->ps_state == PS_MODE_OFF)
    {
        rwm_trigger_tx_bufing_stop(TX_BUFING_SRC_P2P_PS, ind->vif_index);
    }
    else if (ind->ps_state == PS_MODE_ON)
    {
        rwm_trigger_tx_bufing_start(TX_BUFING_SRC_P2P_PS, ind->vif_index);
    }
#endif
}
#endif
#if CFG_USE_AP_PS
void rwm_msdu_ps_change_ind_handler(void *msg)
{
    struct ke_msg *msg_ptr = (struct ke_msg *)msg;
    struct mm_ps_change_ind *ind;

    if(!msg_ptr || !msg_ptr->param)
        return;

    ind = (struct mm_ps_change_ind *)msg_ptr->param;
    if (ind->ps_state == PS_MODE_OFF)
    {
        rwm_trigger_tx_bufing_stop(TX_BUFING_SRC_STA_PS, ind->sta_idx);
    }
    else if(ind->ps_state == PS_MODE_ON)
    {
        rwm_trigger_tx_bufing_start(TX_BUFING_SRC_STA_PS, ind->sta_idx);
    }
}
#endif

void rwm_msdu_init(void)
{
#if CFG_TX_BUFING
    rwm_tx_bufing_init();
#endif
#if !CFG_RWNX_QOS_MSDU
    g_tid = 0xFF;
#endif
}

#ifdef CFG_WFA_CERTIFICATION
/*
 * IEEE802.11-2016: Table 10-1—UP-to-AC mappings
 */
uint8_t ipv4_ieee8023_dscp(UINT8 *buf)
{
	uint8_t tos;
	struct ip_hdr *hdr = (struct ip_hdr *)buf;

	tos = IPH_TOS(hdr);

	return (tos & 0xfc) >> 5;
}

/* extract flow control field */
uint8_t ipv6_ieee8023_dscp(UINT8 *buf)
{
	uint8_t tos;
	struct ip6_hdr *hdr = (struct ip6_hdr *)buf;

	tos = IP6H_FL(hdr);

	return (tos & 0xfc) >> 5;
}
#endif

/*
 * get user priority from @buf.
 * ipv4 dscp/tos, ipv6 flow control. for eapol packets, disable qos.
 */
uint8_t classify8021d(UINT8 *buf)
{
#ifdef CFG_WFA_CERTIFICATION
	struct eth_hdr *ethhdr = (struct eth_hdr *)buf;

	switch (PP_HTONS(ethhdr->type)) {
	case ETHTYPE_IP:
		return ipv4_ieee8023_dscp(ethhdr + 1);
	case ETHTYPE_IPV6:
		return ipv6_ieee8023_dscp(ethhdr + 1);
	case ETH_P_PAE:
		return 7;	/* TID7 highest user priority */
	default:
		return 0;
	}
#else
	return 4;		// TID4: mapped to AC_VI
#endif
}

static bool tx_use_low_rate_once = false;
UINT32 rwm_transfer(UINT8 vif_idx, UINT8 *buf, UINT32 len, int sync, void *args)
{
    UINT32 ret = 0;
    MSDU_NODE_T *node;
    ETH_HDR_PTR eth_hdr_ptr;

    ret = RW_FAILURE;
#ifdef CFG_USE_APP_DEMO_VIDEO_TRANSFER
#if CFG_OS_FREERTOS
extern size_t xPortGetFreeHeapSize( void );
#define MEMORY_LIMIT 7000
    if (xPortGetFreeHeapSize() < MEMORY_LIMIT) {
        os_printf("%s, %d, no mem!\n", __func__, __LINE__);
        goto tx_exit;
    }
#else
   /* Add other OS support */
#endif
#endif
    node = rwm_tx_node_alloc(len);
    if(NULL == node)
    {
        #if NX_POWERSAVE
        txl_cntrl_dec_pck_cnt();
        #endif
#if CFG_SUPPORT_RTT
#if !defined(PKG_NETUTILS_IPERF)
        os_printf("rwm_transfer no node\r\n");
#endif
#else
#if !defined(CFG_IPERF_TEST_ACCEL) || (CFG_IPERF_TEST_ACCEL==0)
        os_printf("rwm_transfer no node\r\n");
#endif
#endif
        goto tx_exit;
    }
    rwm_tx_msdu_renew(buf, len, node->msdu_ptr);

    eth_hdr_ptr = (ETH_HDR_PTR)buf;
    node->vif_idx = vif_idx;
	node->sync = sync;
	node->args = args;
    node->sta_idx = rwm_mgmt_tx_get_staidx(vif_idx,
                             &eth_hdr_ptr->e_dest);
#if CFG_TX_BUFING
    if (rwm_check_tx_bufing(node))
    {
        rwm_tx_bufing_save_data(node);
        return ret;
    }
#endif

    if ( true == tx_use_low_rate_once )
    {
        tx_use_low_rate_once = false;
        /// use RETRY_IMMEDIATELY to transfer at a low rate
        rwm_transfer_node(node, TXU_CNTRL_RETRY_IMMEDIATELY);
    }
    else
    {
        rwm_transfer_node(node, 0);
    }

tx_exit:
    return ret;
}

void ieee80211_data_tx_cb(void *param)
{
	struct txdesc *txdesc_new = (struct txdesc *)param;
	MSDU_NODE_T *node = (MSDU_NODE_T *)txdesc_new->host.msdu_node;
	struct tx_hd *txhd = &txdesc_new->lmac.hw_desc->thd;
	struct ieee80211_tx_cb *cb = (struct ieee80211_tx_cb *)node->args;
	uint32_t status = txhd->statinfo;
	struct l2_packet_node *item, *n;
	struct l2_packet_head *l2_packet = get_l2_packet_entity();
	bool set = false;

	if (0 == node) {
		os_printf("zero_node\r\n");
		return;
	}

	rtos_lock_mutex(&l2_packet->l2_mutex);
	dl_list_for_each_safe(item, n, &l2_packet->head_list, struct l2_packet_node, list) {
		if (item->cb.l2_tag == cb->l2_tag) {
			if (status & FRAME_SUCCESSFUL_TX_BIT /*DESC_DONE_SW_TX_BIT*/)
				cb->result = RW_SUCCESS;
			else
				cb->result = RW_FAILURE;
			rtos_set_semaphore(&item->cb.sema);
			set = true;
			break;
		}
	}
	rtos_unlock_mutex(&l2_packet->l2_mutex);

	if (!set)
		os_printf("XXX: dobule callback for node %p\n", node);
}


#if CFG_RWNX_QOS_MSDU
int sta_11n_nss(uint8_t *mcs_set)
{
	int i;
	int nss = 0;	/* spartial stream num */

	// Go through the MCS map to find out one valid mcs
	for (i = 0; i < 4; i++) {	// 4 == sizeof(rc_ss->rate_map.ht)
		if (mcs_set[i] != 0)
			nss++;
	}

	return nss;
}

int qos_need_enabled(struct sta_info_tag *sta)
{
	if(rwn_mgmt_is_valid_sta(sta) == 0)
		return 0;
#if CFG_TKIP_SW_CRYPT
	if(sta->sta_sec_info.cur_key == NULL)
		return 0;
	struct key_info_tag *key = *(sta->sta_sec_info.cur_key);
	if ((NULL != key) && (key->cipher == MAC_RSNIE_CIPHER_TKIP))
		return 0; /* disable QOS for TKIP */
#endif
	if (!(sta->info.capa_flags & STA_QOS_CAPA))
		return 0;

	return 1;
}
#endif

#if CFG_WIFI_TX_KEYDATA_USE_LOWEST_RATE
// EAPoL, DHCP, ARP
bool rwm_use_lowest_rate(ETH_HDR_T *eth)
{
	bool use = false;

	switch (htons(eth->e_proto)) {
	case ETHTYPE_ARP:
		/* fall through */
	case ETHTYPE_EAPOL:
		use = true;
		break;

	case ETHTYPE_IP: {
		struct ip_hdr *ip = (struct ip_hdr *)(eth + 1);

		switch (IPH_PROTO(ip)) {
		case IP_PROTO_UDP: {
			struct udp_hdr *udp = (struct udp_hdr *)((uint8_t *)ip + IPH_HL(ip) * 4);
			if (ntohs(udp->dest) == DHCP_SERVER_PORT)
				use = true;
		}	break;
		}
	}	break;
	}

	return use;
}
#endif

UINT32 rwm_transfer_node(MSDU_NODE_T *node, u16 flag)
{
    UINT8 tid;
    UINT32 ret = 0;
    UINT8 *content_ptr;

    UINT32 queue_idx;

    ETH_HDR_PTR eth_hdr_ptr;
    struct txdesc *txdesc_new;
#if CFG_RWNX_QOS_MSDU
	struct sta_info_tag *sta;
	struct vif_info_tag *vif;
#endif

    if(!node) {
        goto tx_exit;
    }

    content_ptr = rwm_get_msdu_content_ptr(node);
    eth_hdr_ptr = (ETH_HDR_PTR)content_ptr;

#if CFG_RWNX_QOS_MSDU
	vif = rwm_mgmt_vif_idx2ptr(node->vif_idx);
	if (NULL == vif)
	{
		os_printf("%s: vif is NULL!\r\n", __func__);
		goto tx_exit;
	}
	if (likely(vif->active)) {
		sta = &sta_info_tab[vif->u.sta.ap_id];
		if (qos_need_enabled(sta)) {
			int i;
			tid = classify8021d((UINT8 *)eth_hdr_ptr);
			/* check admission ctrl */
			for (i = mac_tid2ac[tid]; i >= 0; i--)
				if (!(vif->bss_info.edca_param.acm & BIT(i)))
					break;
			if (i < 0)
				goto tx_exit;
			queue_idx = i;	/* AC_* */
		} else {
			/*
			 * non-WMM STA
			 *
			 * CWmin 15, CWmax 1023, AIFSN 2, TXOP 0. set these values when joining with this BSS.
			 */
			tid = 0xFF;
			queue_idx = AC_VI;
		}
	} else {
		tid = 0xFF;
	    queue_idx = AC_VI;
	}
#else /* !CFG_RWNX_QOS_MSDU */
    tid = rwm_get_tid();

    queue_idx = AC_VI;
#endif /* CFG_RWNX_QOS_MSDU */

    txdesc_new = tx_txdesc_prepare(queue_idx);
    if(TXDESC_STA_USED == txdesc_new->status)
    {
#if CFG_SUPPORT_RTT
#if !defined(PKG_NETUTILS_IPERF)
        os_printf("rwm_transfer no txdesc \r\n");

#endif
#else
#if !defined(CFG_IPERF_TEST_ACCEL) || (CFG_IPERF_TEST_ACCEL==0)
        os_printf("rwm_transfer no txdesc \r\n");
#endif
#endif
        goto tx_exit;
    }

    txdesc_new->status = TXDESC_STA_USED;
    rwm_txdesc_copy(txdesc_new, eth_hdr_ptr);

    txdesc_new->host.flags            = flag;
#if NX_AMSDU_TX
    txdesc_new->host.orig_addr[0]     = (UINT32)node->msdu_ptr;
    txdesc_new->host.packet_addr[0]   = (UINT32)content_ptr + 14;
    txdesc_new->host.packet_len[0]    = node->len - 14;
    txdesc_new->host.packet_cnt       = 1;
#else
    txdesc_new->host.orig_addr        = (UINT32)node->msdu_ptr;
    txdesc_new->host.packet_addr      = (UINT32)content_ptr + 14;
    txdesc_new->host.packet_len       = node->len - 14;
#endif
#if CFG_NX_SOFTWARE_TX_RETRY
    // record total retry times, used for rate contrl in low mac
    txdesc_new->host.status_desc_addr = 0;
    txdesc_new->host.access_category = queue_idx;
    txdesc_new->host.flags |= TXU_CNTRL_EN_SW_RETRY_CHECK;
    txdesc_new->lmac.hw_desc->thd.statinfo = 0;
#else
    txdesc_new->host.status_desc_addr = (UINT32)content_ptr + 14;
#endif
    txdesc_new->host.ethertype        = eth_hdr_ptr->e_proto;

#if CFG_WIFI_TX_KEYDATA_USE_LOWEST_RATE
    if (rwm_use_lowest_rate(eth_hdr_ptr))
        txdesc_new->host.flags |= TXU_CNTRL_LOWEST_RATE;
#endif

    txdesc_new->host.tid              = tid;

    txdesc_new->host.vif_idx          = node->vif_idx;
    txdesc_new->host.staid            = node->sta_idx;
	txdesc_new->host.msdu_node        = (void *)node;

	if (node->sync)
	{
		txdesc_new->host.callback		  = (mgmt_tx_cb_t)ieee80211_data_tx_cb;
		txdesc_new->host.param			  = (void *)txdesc_new;
	}
	else
	{
		txdesc_new->host.callback = 0;
	}

    txdesc_new->lmac.agg_desc = NULL;
    txdesc_new->lmac.hw_desc->cfm.status = 0;

    txu_cntrl_push(txdesc_new, queue_idx);
    return ret;

tx_exit:
    if (NULL != node)
        rwm_node_free(node);
#if NX_POWERSAVE
    txl_cntrl_dec_pck_cnt();
#endif
    return ret;
}

UINT32 rwm_get_rx_free_node(struct pbuf **p_ret, UINT32 len)
{
    struct pbuf *p;

    p = pbuf_alloc(PBUF_RAW, len, PBUF_RAM);
    *p_ret = p;

    return RW_SUCCESS;
}

static const uint8_t rfc1042_header[6] = { 0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00 };
static const uint8_t bridge_tunnel_header[6] = { 0xaa, 0xaa, 0x03, 0x00, 0xf8, 0x00 };
void ethernetif_input_amsdu(RW_RXIFO_PTR rx_info, struct pbuf *p)
{
    struct pbuf *pbuf = NULL;
    uint32_t du_len;
    struct mac_addr temp_mac;

    struct llc_snap *llc_snap;
    struct ethernet_hdr *eth_hdr;
    struct amsdu_hdr *amsdu_subfrm_hdr = (struct amsdu_hdr *)p->payload;
    uint32_t mpdu_end = (uint32_t)p->payload + p->len;
    uint16_t msdu_len_with_padding;
    VIF_INF_PTR p_vif_entry = rwm_mgmt_vif_idx2ptr(rx_info->vif_idx);

    /*
     * format of p->payload
     ****************************************************
     *  |  SUB_FRAME  |  SUB_FRAME  |  SUB_FRAME  |
     ****************************************************
     * format of SUB_FRAME
     * 1 MESH type
     ****************************************************
     *  |  DA  |  SA  |  LENGTH  |  MESH_CONTROL  |  DATA  |  PADDING  |
     ****************************************************
     * 2 SNAP type
     ***********************************************************************
     *  |  DA  |  SA  |  LENGTH  |  LLC/SNAP  |  DATA  |  PADDING  |
     ***********************************************************************
     * 3 RAW type (should not happen actually)
     ****************************************************
     *  |  DA  |  SA  |  LENGTH  |  DATA  |  PADDING  |
     ****************************************************
     */

    //os_printf("%s amsdu_len=%d\n", __FUNCTION__, p->len);
    while ((uint32_t)amsdu_subfrm_hdr < mpdu_end)
    {
        //calculate msdu_len_with_padding first
        du_len = ntohs(amsdu_subfrm_hdr->len);
        msdu_len_with_padding = sizeof(struct amsdu_hdr) + du_len + sizeof(uint32_t) - 1;
        msdu_len_with_padding = msdu_len_with_padding & ~(sizeof(uint32_t) - 1);
        //os_printf("%s amsdu_subframe_len=%d,msdu_len_with_padding=%d\n", __FUNCTION__, du_len, msdu_len_with_padding);

        //ieee802.11 amsdu_hdr to ieee802.3 ethernet_hdr
#if 0//(RW_MESH_EN)
        if ((p_vif_entry->type == VIF_MESH_POINT) && (rx_info->dst_idx != INVALID_STA_IDX))
        {
            /*
                ****************************************************
                *  |  DA  |  SA  |  LENGTH  |  MESH_CONTROL  |  DATA  |  PADDING  |
                ****************************************************
                * ==>
                ****************************************************
                *  DA  |  SA  |  ETHERTYPE  |  MESH_CONTROL  |  DATA  |  PADDING  |
                ****************************************************
                *  Keep subframe as mesh frame, since amsdu_hdr=ethernet_hdr
                *  set eth_hdr->len as ethertype like rxu_cntrl_mac2eth_update
                */
            eth_hdr = (struct ethernet_hdr *)amsdu_subfrm_hdr;
            llc_snap = (struct llc_snap *)((uint8_t *)amsdu_subfrm_hdr + sizeof(struct ethernet_hdr) + rx_status->mesh_ctrl_len);
            eth_hdr->len = llc_snap->proto_id;
            du_len += sizeof(struct ethernet_hdr);
        }
        else
#endif //(RW_MESH_EN)
        {
            llc_snap = (struct llc_snap *)(amsdu_subfrm_hdr + 1);

            if ((!memcmp(llc_snap, &rfc1042_header, sizeof(rfc1042_header))
                 //&& (llc_snap->ether_type != RX_ETH_PROT_ID_AARP) - Appletalk depracated ?
                 && (llc_snap->proto_id != RX_ETH_PROT_ID_IPX))
                || (!memcmp(llc_snap, &bridge_tunnel_header, sizeof(bridge_tunnel_header))))
            {
                /*
                    ****************************************************
                    *  |  DA  |  SA  |  LENGTH  |  LLC/SNAP  |  DATA  |  PADDING  |
                    ****************************************************
                    * ==>
                    ****************************************************
                    *  |  DA  |  SA  |  DATA  |  PADDING  |
                    ****************************************************
                    */
                eth_hdr = (struct ethernet_hdr *)((uint8_t *)amsdu_subfrm_hdr + sizeof(struct llc_snap_short) + sizeof(amsdu_subfrm_hdr->len));
                du_len += sizeof(struct ethernet_hdr) - sizeof(struct llc_snap_short) - sizeof(amsdu_subfrm_hdr->len);
                MAC_ADDR_CPY(&eth_hdr->sa, &amsdu_subfrm_hdr->sa);
                MAC_ADDR_CPY(&eth_hdr->da, &amsdu_subfrm_hdr->da);
            }
            else
            {
                /*
                    ****************************************************
                    *  |  DA  |  SA  |  LENGTH  |  DATA  |  PADDING  |
                    ****************************************************
                    * ==>
                    ****************************************************
                    *  |  DA  |  SA  |  DATA  |  PADDING  |
                    ****************************************************
                    */
                eth_hdr = (struct ethernet_hdr *)((uint8_t *)amsdu_subfrm_hdr + sizeof(amsdu_subfrm_hdr->len));
                du_len += sizeof(struct ethernet_hdr) - sizeof(amsdu_subfrm_hdr->len);
                MAC_ADDR_CPY(&temp_mac, &amsdu_subfrm_hdr->sa);
                MAC_ADDR_CPY(&eth_hdr->sa, &temp_mac);
                MAC_ADDR_CPY(&temp_mac, &amsdu_subfrm_hdr->da);
                MAC_ADDR_CPY(&eth_hdr->da, &temp_mac);
            }
        }


        // If the frame's src addr not equal to us.
        if (p_vif_entry && os_memcmp(&eth_hdr->sa, &p_vif_entry->mac_addr, ETH_ALEN))
        {
            //malloc/dma/callback
            rwm_get_rx_free_node(&pbuf, du_len);
            if (NULL == pbuf)
            {
                os_printf("%s rwm_get_rx_free_node(%d) failed\n", __FUNCTION__, du_len);
            }
            else
            {
                os_memcpy(pbuf->payload, (void *)eth_hdr, du_len);
                ethernetif_input(rx_info->vif_idx, pbuf);
            }
        }

        //next amsdu_subframe
        amsdu_subfrm_hdr = (struct amsdu_hdr *)((uint8_t *)amsdu_subfrm_hdr + msdu_len_with_padding);
    }

    pbuf_free(p);
}

UINT32 rwm_upload_data(RW_RXIFO_PTR rx_info)
{
    struct pbuf *p = (struct pbuf *)rx_info->data;
    STA_INF_PTR sta_entry;

    os_null_printf("s:%d, v:%d, d:%d, r:%d, c:%d, l:%d, %p\r\n",
                   rx_info->sta_idx,
                   rx_info->vif_idx,
                   rx_info->dst_idx,
                   rx_info->rssi,
                   rx_info->center_freq,
                   rx_info->length,
                   rx_info->data);

    sta_entry = rwm_mgmt_sta_idx2ptr(rx_info->sta_idx);
    if (sta_entry)
        sta_entry->rssi = rx_info->rssi;

    if (rx_info->rx_dmadesc_flags & RX_FLAGS_IS_AMSDU_BIT)
    {
        /* A-MSDU subframe, convert like 'rxu_cntrl_mac2eth_update()' and then pass it to lwip */
        ethernetif_input_amsdu(rx_info, p);
    }
    else
    {
        struct vif_info_tag *p_vif_entry = rwm_mgmt_vif_idx2ptr(rx_info->vif_idx);

        if (p_vif_entry)
        {
            if (p->len > sizeof(struct ethernet_hdr))
            {
                struct ethernet_hdr *eth_hdr = (struct ethernet_hdr *)p->payload;

                // If the frame's src addr not equal to us.
                if (os_memcmp(&eth_hdr->sa, &p_vif_entry->mac_addr, ETH_ALEN))
                {
                    ethernetif_input(rx_info->vif_idx, p);
                    return RW_SUCCESS;
                }
            }
        }

        // drop packet
        pbuf_free(p);
    }

    return RW_SUCCESS;
}

UINT32 rwm_uploaded_data_handle(UINT8 *upper_buf, UINT32 len)
{
    UINT32 count;
    UINT32 ret = RW_FAILURE;
    MSDU_NODE_T *node_ptr;

    node_ptr = rwm_pop_rx_list();
    if(node_ptr)
    {
        count = _MIN(len, node_ptr->len);
#if CFG_GENERAL_DMA && (CFG_SOC_NAME != SOC_BK7231N)
        gdma_memcpy(upper_buf, node_ptr->msdu_ptr, count);
#else
        os_memcpy(upper_buf, node_ptr->msdu_ptr, count);
#endif
        ret = count;

        os_free(node_ptr);
        node_ptr = NULL;
    }

    return ret;
}

///////////////////////////////////////////////////////////////////////////////
VIF_INF_PTR rwm_mgmt_vif_idx2ptr(UINT8 vif_idx)
{
    VIF_INF_PTR vif_entry = NULL;

    if(vif_idx < NX_VIRT_DEV_MAX)
        vif_entry = &vif_info_tab[vif_idx];

    return vif_entry;
}

VIF_INF_PTR rwm_mgmt_vif_type2ptr(UINT8 vif_type)
{
    VIF_INF_PTR vif_entry = NULL;
    UINT32 i;

    for(i = 0; i < NX_VIRT_DEV_MAX; i++)
    {
        vif_entry = &vif_info_tab[i];
        if(vif_entry->type == vif_type)
            break;
    }

    if(i == NX_VIRT_DEV_MAX)
        vif_entry = NULL;

    return vif_entry;
}

STA_INF_PTR rwm_mgmt_sta_idx2ptr(UINT8 staid)
{
    STA_INF_PTR sta_entry = NULL;

    if(staid < NX_REMOTE_STA_MAX)
        sta_entry = &sta_info_tab[staid];

    return sta_entry;
}

STA_INF_PTR rwm_mgmt_sta_mac2ptr(void *mac)
{
    UINT32 i;
    STA_INF_PTR sta_entry = NULL;

    for(i = 0; i < NX_REMOTE_STA_MAX; i++)
    {
        sta_entry = &sta_info_tab[i];
        if(MAC_ADDR_CMP((void *)&sta_entry->mac_addr, mac))
            break;
    }

    return sta_entry;
}

UINT8 rwm_mgmt_sta_mac2idx(void *mac)
{
    UINT32 i;
    UINT8 staid = 0xff;
    STA_INF_PTR sta_entry = NULL;

    for(i = 0; i < NX_REMOTE_STA_MAX; i++)
    {
        sta_entry = &sta_info_tab[i];
        if(MAC_ADDR_CMP((void *)&sta_entry->mac_addr, mac))
            break;
    }
    if(i < NX_REMOTE_STA_MAX)
        staid = i;

    return staid;
}

UINT8 rwm_mgmt_sta_mac2port(void *mac)
{
    UINT32 i;
    STA_INF_PTR sta_entry = NULL;

    for(i = 0; i < NX_REMOTE_STA_MAX; i++)
    {
        sta_entry = &sta_info_tab[i];
        if(MAC_ADDR_CMP((void *)&sta_entry->mac_addr, mac))
            break;
    }

	if (sta_entry)
	{
		if (sta_entry->ctrl_port_state == PORT_OPEN)
            return 1;
	}

	return 0;
}

UINT8 rwm_mgmt_vif_mac2idx(void *mac)
{
    VIF_INF_PTR vif_entry = NULL;
    UINT8 vif_idx = INVALID_VIF_IDX;
    UINT32 i;

    for(i = 0; i < NX_VIRT_DEV_MAX; i++)
    {
        vif_entry = &vif_info_tab[i];
        if(MAC_ADDR_CMP((void *)&vif_entry->mac_addr, mac))
            break;
    }

    if(i < NX_VIRT_DEV_MAX)
        vif_idx = i;

    return vif_idx;
}

UINT8 rwm_mgmt_vif_name2idx(char *name)
{
    VIF_INF_PTR vif_entry = NULL;
    struct netif *lwip_if;
    UINT8 vif_idx = 0xff;
    UINT32 i;

    for(i = 0; i < NX_VIRT_DEV_MAX; i++)
    {
        vif_entry = &vif_info_tab[i];
        if(vif_entry->priv)
        {
            lwip_if = (struct netif *)vif_entry->priv;
            if (!os_strncmp(lwip_if->hostname, name, os_strlen(lwip_if->hostname)))
            {
                break;
            }
        }
    }

    if(i < NX_VIRT_DEV_MAX)
        vif_idx = i;

    return vif_idx;
}

/**
 * set UPD_RATE flag since temprature etc. change
 */
UINT8 rwm_mgmt_update_rate(void)
{
    UINT32 i;
    struct sta_pol_tbl_cntl *pol_tbl;
    GLOBAL_INT_DECLARATION();

    GLOBAL_INT_DISABLE();
    for(i = 0; i < NX_REMOTE_STA_MAX; i++)
    {
        pol_tbl = &sta_info_tab[i].pol_tbl;
        // Keep in mind we have to update the rate, since temprature change etc.
        pol_tbl->upd_field |= CO_BIT(STA_MGMT_POL_UPD_RATE);
    }
    GLOBAL_INT_RESTORE();

	return 0;
}

UINT8 rwm_mgmt_get_hwkeyidx(UINT8 vif_idx, UINT8 staid, UINT8 key_idx)
{
    UINT8 hw_key_idx = MM_SEC_MAX_KEY_NBR + 1;
    struct key_info_tag *key = NULL;

    VIF_INF_PTR vif_entry = NULL;
    STA_INF_PTR sta_entry = NULL;

    if(staid == 0xff)   // group key
    {
        vif_entry = rwm_mgmt_vif_idx2ptr(vif_idx);
#if NX_MFP
        if (vif_entry && key_idx < MAC_DEFAULT_MFP_KEY_COUNT)
#else
        if (vif_entry && key_idx < MAC_DEFAULT_KEY_COUNT)
#endif
            key = &vif_entry->key_info[key_idx];
    }
    else
    {
        sta_entry = rwm_mgmt_sta_idx2ptr(staid);
        if(sta_entry)
            key = *(sta_entry->sta_sec_info.cur_key);
    }

    if(key && key->valid)
    {
        hw_key_idx = key->hw_key_idx;
    }

    return hw_key_idx;
}

void rwm_mgmt_set_vif_netif(struct netif *net_if)
{
    VIF_INF_PTR vif_entry = NULL;
    UINT8 vif_idx;

    if(!net_if)
        return;

    vif_idx = rwm_mgmt_vif_mac2idx(net_if->hwaddr);
    vif_entry = rwm_mgmt_vif_idx2ptr(vif_idx);

    if(vif_entry)
    {
        vif_entry->priv = net_if;
        net_if->state = (void *)vif_entry;
    }
    else
    {
        os_printf("warnning: set_vif_netif failed\r\n");
    }
}

struct netif *rwm_mgmt_get_vif2netif(UINT8 vif_idx)
{
    VIF_INF_PTR vif_entry = NULL;
    struct netif *netif = NULL;

    vif_entry = rwm_mgmt_vif_idx2ptr(vif_idx);

    if(vif_entry)
        netif = (struct netif *)vif_entry->priv;

    return netif;
}

UINT8 rwm_mgmt_get_netif2vif(struct netif *netif)
{
    UINT8 vif_idx = 0xff;
    VIF_INF_PTR vif_entry = NULL;

    if(netif && netif->state)
    {
        vif_entry = (VIF_INF_PTR)netif->state;
        vif_idx = vif_entry->index;
    }

    return vif_idx;
}

UINT8 rwm_mgmt_tx_get_staidx(UINT8 vif_idx, void *dstmac)
{
    UINT8 staid = 0xff;
    VIF_INF_PTR vif_entry = NULL;

    vif_entry = rwm_mgmt_vif_idx2ptr(vif_idx);

    if(vif_entry)
    {
        if(vif_entry->type == VIF_STA)
        {
            staid = vif_entry->u.sta.ap_id;
        }
        else if(vif_entry->type == VIF_AP)
        {
            staid = rwm_mgmt_sta_mac2idx(dstmac);
        }
    }

    if(staid == 0xff)
    {
        staid = VIF_TO_BCMC_IDX(vif_idx);
    }

    return staid;
}

u8 rwn_mgmt_is_only_sta_role_add(void)
{
    VIF_INF_PTR vif_entry = (VIF_INF_PTR)rwm_mgmt_is_vif_first_used();

    if(!vif_entry)
        return 0;

    if(vif_entry->type == VIF_STA)
        return 1;

    return 0;
}

#include "lwip/sockets.h"
extern uint8_t* dhcp_lookup_mac(uint8_t *chaddr);

void rwn_mgmt_show_vif_peer_sta_list(UINT8 role)
{
    struct vif_info_tag *vif = (VIF_INF_PTR)rwm_mgmt_is_vif_first_used();
    struct sta_info_tag *sta;
    UINT8 num = 0;

    while(vif) {
        if ( vif->type == role) {
            sta = (struct sta_info_tag *)co_list_pick(&vif->sta_list);
            while (sta != NULL)
            {
                UINT8 *macptr = (UINT8*)sta->mac_addr.array;
                UINT8 *ipptr = NULL;

                if(role == VIF_AP) {
                    ipptr = dhcp_lookup_mac(macptr);
                } else if (role == VIF_STA){
                    struct netif *netif = (struct netif *)vif->priv;
                    ipptr = (UINT8 *)inet_ntoa(netif->gw);
                }

                os_printf("%d: mac:%02x-%02x-%02x-%02x-%02x-%02x, ip:%s\r\n", num++,
                    macptr[0], macptr[1], macptr[2],
                    macptr[3], macptr[4], macptr[5],ipptr);

                sta = (struct sta_info_tag *)co_list_next(&sta->list_hdr);
            }
        }
        vif = (VIF_INF_PTR) rwm_mgmt_next(vif);
    }
}

UINT8 rwn_mgmt_if_ap_stas_empty()
{
    struct vif_info_tag *vif = (VIF_INF_PTR)rwm_mgmt_is_vif_first_used();
    UINT8 role = VIF_AP;

    while(vif) {
        if ( vif->type == role) {
            if(co_list_is_empty(&vif->sta_list))
                {
                return 1;
            }
        }
        vif = (VIF_INF_PTR) rwm_mgmt_next(vif);
    }
    return 0;
}

UINT8 rwn_mgmt_is_valid_sta(struct sta_info_tag *sta)
{
    uint8_t sta_idx = 0xff;
    if(sta)
    {
        sta_idx = CO_GET_INDEX(sta, sta_info_tab);
        if(sta_idx < STA_MAX)
        {
            if((sta->inst_nbr != 0xff) && (sta->staid == sta_idx))
            {
                return 1;
            }
        }
    }
    return 0;
}

#if CFG_NX_SOFTWARE_TX_RETRY
UINT32 rwn_check_sw_tx_retry(struct txdesc *txdesc)
{
    #define PKTS_STATUS_NULL             0  // null
    #define PKTS_STATUS_TX_DONE          1  // done but no sw retry
    #define PKTS_STATUS_TX_DROP          2  // droped but other case
    #define PKTS_STATUS_TX_RETRY_DONE    3  // done with sw retry
    #define PKTS_STATUS_TX_RETRY_FAIL    4  // droped with sw retry
    #define PKTS_STATUS_TX_RETRYING      5  // sw retry txing
    uint32_t pkt_status = PKTS_STATUS_NULL;

    if(txdesc->host.flags & TXU_CNTRL_EN_SW_RETRY_CHECK)
    {
        uint32_t cfm_status = txdesc->lmac.hw_desc->cfm.status;
        uint32_t txstatus = txdesc->lmac.hw_desc->thd.statinfo;
        uint32_t tx_acked = ((txstatus & (DESC_DONE_TX_BIT | FRAME_SUCCESSFUL_TX_BIT)) == (DESC_DONE_TX_BIT | FRAME_SUCCESSFUL_TX_BIT));
        uint32_t retry_cnt = ((txstatus & NUM_MPDU_RETRIES_MSK) >> NUM_MPDU_RETRIES_OFT);
        uint32_t retry_limit = ((txstatus & RETRY_LIMIT_REACHED_BIT) != 0);
        uint32_t failures = retry_cnt + retry_limit;
        uint32_t last_retry = (uint32_t)txdesc->host.status_desc_addr;
        uint32_t total_retry;

        total_retry = last_retry + failures;

        #define SW_RETRY_LIMITED_COUNT      (30)
        if(tx_acked)
        {
            total_retry += 1; // total send times
            if(last_retry)
                pkt_status = PKTS_STATUS_TX_RETRY_DONE;
            else
            {
                pkt_status = PKTS_STATUS_TX_DONE;
            }
        }
        else if(total_retry < SW_RETRY_LIMITED_COUNT)
        {
            uint32_t need_drop = 0;
            uint32_t sw_done = ((cfm_status & TX_STATUS_SW_RETRY_REQUIRED) != 0);
            if(txl_check_reset())
                need_drop |= (1<<0);
            else if(sw_done)
            {
                need_drop |= (1<<1); // drop mac flush pkts
            }
            if(need_drop == 0)
            {
                uint8_t access_category = txdesc->host.access_category;

                pkt_status = PKTS_STATUS_TX_RETRYING;

                // record total retry times, used for rate contrl in low mac
                txdesc->host.status_desc_addr = (UINT32)total_retry;
                txdesc->host.flags |= TXU_CNTRL_RETRY;
                txdesc->lmac.hw_desc->cfm.status = 0;
                txdesc->status = TXDESC_STA_USED;
                txdesc->lmac.hw_desc->thd.statinfo = 0;
                txu_cntrl_push(txdesc, access_category);
                txl_cntrl_inc_pck_cnt();
                //os_printf("txing: 0x%08x-0x%08x, %d, %d,\r\n", txstatus, cfm_status, pkt_status, total_retry);
                return 1;
            }
            else
            {
                pkt_status = PKTS_STATUS_TX_DROP;
            }
        }
        else
        {
            pkt_status = PKTS_STATUS_TX_RETRY_FAIL;
        }
        pkt_status = pkt_status;
        //os_printf("txend: 0x%08x-0x%08x, %d, %d\r\n", txstatus, cfm_status, pkt_status, total_retry);
    }

    return 0;
}
#endif

/**
 * next package will be sent at a low rate for once
 * Notice that only packages sent by rwm_transfer() can use this function and
 * it won't work only if CFG_NX_SOFTWARE_TX_RETRY or CFG_LOW_VOLTAGE_PS is on for now.
*/
void rwn_set_tx_low_rate_once(void)
{
    tx_use_low_rate_once = true;
}

// eof

