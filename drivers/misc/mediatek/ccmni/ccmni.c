/*****************************************************************************
 *
 * Filename:
 * ---------
 *   ccmni.c
 *
 * Project:
 * --------
 *   
 *
 * Description:
 * ------------
 *   Cross Chip Modem Network Interface
 *
 * Author:
 * -------
 *   Anny.Hu(mtk80401)
 *
 ****************************************************************************/
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/ipv6.h>
#include <net/ipv6.h>
#include <linux/skbuff.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/version.h>
#include <linux/sockios.h>
#include "ccmni.h"


ccmni_ctl_block_t *ccmni_ctl_blk[MAX_MD_NUM];
unsigned int ccmni_debug_enable = 0;



/********************internal function*********************/
static int get_ccmni_idx_from_ch(int md_id, int ch)
{
	ccmni_ctl_block_t *ctlb = ccmni_ctl_blk[md_id];
	unsigned int i;

	for (i = 0; i < ctlb->ccci_ops->ccmni_num; i++) {
		if (ctlb->ccmni_inst[i]) {
			if ((ctlb->ccmni_inst[i]->ch.rx == ch) || (ctlb->ccmni_inst[i]->ch.tx_ack == ch)) {
				return i;
			} 
		} else {
			CCMNI_ERR_MSG(md_id, "invalid ccmni instance(ccmni%d): ch=%d\n", i, ch); 
		}
	}

	CCMNI_ERR_MSG(md_id, "invalid ccmni rx channel(%d)\n", ch); 
	return -1;
}

static void ccmni_make_etherframe(void *_eth_hdr, unsigned char *mac_addr, unsigned int packet_type)
{
	struct ethhdr *eth_hdr = _eth_hdr;

	memcpy(eth_hdr->h_dest,   mac_addr, sizeof(eth_hdr->h_dest));
	memset(eth_hdr->h_source, 0, sizeof(eth_hdr->h_source));
	if(packet_type == 0x60) {
		eth_hdr->h_proto = __constant_cpu_to_be16(ETH_P_IPV6);
	} else {
		eth_hdr->h_proto = __constant_cpu_to_be16(ETH_P_IP);
	}
}

static inline int is_ack_skb(struct sk_buff *skb)
{
	u32 packet_type;
    struct tcphdr *tcph;

    packet_type = skb->data[0] & 0xF0;
    if (packet_type == IPV6_VERSION) {
        struct ipv6hdr *iph = (struct ipv6hdr *)skb->data;
        u32 total_len = sizeof(struct ipv6hdr) + ntohs(iph->payload_len);
        if (total_len <= 128 - sizeof(struct ccci_header)) {
            u8 nexthdr = iph->nexthdr;
            __be16 frag_off;
            u32 l4_off = ipv6_skip_exthdr(skb, sizeof(struct ipv6hdr), &nexthdr, &frag_off);
            tcph = (struct tcphdr *)(skb->data + l4_off);
            if (nexthdr == IPPROTO_TCP && 
	    	!tcph->syn && !tcph->fin && !tcph->rst && 
	    	((total_len - l4_off) == (tcph->doff << 2))) {
                return 1;
            }
        }
    } else if (packet_type == IPV4_VERSION) {
        struct iphdr *iph = (struct iphdr *)skb->data;
        if (ntohs(iph->tot_len) <= 128 - sizeof(struct ccci_header)) {
            tcph = (struct tcphdr *)(skb->data + (iph->ihl << 2));
            if (iph->protocol == IPPROTO_TCP && 
	    	!tcph->syn && !tcph->fin && !tcph->rst && 
	    	(ntohs(iph->tot_len) == (iph->ihl << 2) + (tcph->doff << 2))) {
                return 1;
            }
        }
    }

    return 0;
}


/********************internal debug function*********************/
#if 0
static void ccmni_dbg_skb(struct sk_buff *skb){
    CCMNI_DBG_MSG(-1, "[SKB] head=0x%x end=0x%x data=0x%x tail=0x%x len=0x%x data_len=0x%x\n",\
        (unsigned int)skb->head, (unsigned int)skb->end, \
        (unsigned int)skb->data, (unsigned int)skb->tail,\
        (unsigned int)skb->len,  (unsigned int)skb->data_len);
}

static void ccmni_dbg_skb_addr(struct sk_buff *skb, int idx){
    ccmni_dbg_skb(skb);
    CCMNI_DBG_MSG(-1, "[SKB] idx=%d addr=0x%x size=%d L2_addr=0x%x L3_addr=0x%x L4_addr=0x%x\n", idx, \
		(unsigned int)skb->data, skb->len, (unsigned int)skb_mac_header(skb), \
        (unsigned int)skb_network_header(skb), (unsigned int)skb_transport_header(skb));
}

static void ccmni_dbg_eth_header(struct ethhdr *ethh){
    if (ethh != NULL) {
    	 CCMNI_DBG_MSG(-1, "[SKB] ethhdr: proto=0x%04x dest_mac=%02x:%02x:%02x:%02x:%02x:%02x src_mac=%02x:%02x:%02x:%02x:%02x:%02x\n",\
        	ethh->h_proto, ethh->h_dest[0], ethh->h_dest[1], ethh->h_dest[2], ethh->h_dest[3], ethh->h_dest[4], ethh->h_dest[5], \
        	ethh->h_source[0], ethh->h_source[1], ethh->h_source[2], ethh->h_source[3], ethh->h_source[4], ethh->h_source[5]);
    }
}

static void ccmni_dbg_ip_header(struct iphdr *iph){
    if (iph != NULL) {
    	 CCMNI_DBG_MSG(-1, "[SKB] iphdr: ihl=0x%02x ver=0x%02x tos=0x%02x tot_len=0x%04x id=0x%04x frag_off=0x%04x ttl=0x%02x proto=0x%02x check=0x%04x saddr=0x%08x daddr=0x%08x\n",\
        	iph->ihl, iph->version, iph->tos, iph->tot_len, \
        	iph->id, iph->frag_off, iph->ttl, iph->protocol, iph->check, iph->saddr, iph->daddr);
    }
}

static void ccmni_dbg_tcp_header(struct tcphdr *tcph){
    if (tcph != NULL) {
    	 CCMNI_DBG_MSG(-1, "[SKB] tcp_hdr: src=0x%04x dest=0x%04x seq=0x%08x ack_seq=0x%08x urg=%d ack=%d psh=%d rst=%d syn=%d fin=%d\n",\
	        ntohl(tcph->source), ntohl(tcph->dest), tcph->seq, tcph->ack_seq, tcph->urg, \
	        tcph->ack, tcph->psh, tcph->rst, tcph->syn, tcph->fin);
    }
}

static void ccmni_dbg_skb_header(struct sk_buff *skb){
    struct ethhdr *ethh;
    struct iphdr  *iph;
    struct ipv6hdr  *ipv6h;
    struct tcphdr  *tcph;
    struct tcphdr _tcph;
    unsigned int protoff;

    ethh = (struct ethhdr *)skb->data;
    if(NULL != ethh) {
    	ccmni_dbg_eth_header(ethh);
    }

	if (skb->protocol == htons(ETH_P_IP)) {
		iph = ip_hdr(skb);
		ccmni_dbg_ip_header(iph);
	} else if (skb->protocol == htons(ETH_P_IPV6)) {
		ipv6h = ipv6_hdr(skb);
	}
	
	if(NULL == iph)
		return;

    switch (iph->protocol) {
        case IPPROTO_UDP:
        {
            struct udphdr *udph;
            udph = (struct udphdr *)skb_transport_header(skb);
             CCMNI_DBG_MSG(-1, "[SKB] UDP: source=0x%x) dest=0x%x len=%d check=0x%04x\n",\
                    ntohs(udph->source), ntohs(udph->dest), ntohs(udph->len), udph->check);
            break;
        }

		case IPPROTO_TCP:
		{			
			if (skb->protocol == htons(ETH_P_IP)) {
				protoff = iph->ihl*4;
				tcph = skb_header_pointer(skb, protoff, sizeof(_tcph), &_tcph);
			} else if (skb->protocol == htons(ETH_P_IPV6)) {
				tcph = tcp_hdr(skb);
			}
			ccmni_dbg_tcp_header(tcph);
            break;
        }
        default:
            break;
    }
}
#endif


/********************netdev register function********************/
static int ccmni_open(struct net_device *dev)
{
	ccmni_instance_t *ccmni = (ccmni_instance_t *)netdev_priv(dev);
	ccmni_ctl_block_t *ccmni_ctl = ccmni_ctl_blk[ccmni->md_id];

	if (unlikely(ccmni_ctl == NULL)) {
		CCMNI_ERR_MSG(ccmni->md_id, "CCMNI%d open: MD%d ctlb is NULL\n", ccmni->index, ccmni->md_id);
		return -1;
	}
	
	CCMNI_INF_MSG(ccmni->md_id, "CCMNI%d open: cnt=%d, md_ability=0x%X\n", ccmni->index, \
		atomic_read(&ccmni->usage), ccmni_ctl->ccci_ops->md_ability);
	netif_start_queue(dev);
	if(unlikely(ccmni_ctl->ccci_ops->md_ability & MODEM_CAP_NAPI)) {
		napi_enable(&ccmni->napi);
		napi_schedule(&ccmni->napi);
	}
	atomic_inc(&ccmni->usage);
	
	return 0;
}

static int ccmni_close(struct net_device *dev)
{
	ccmni_instance_t *ccmni = (ccmni_instance_t *)netdev_priv(dev);
	ccmni_ctl_block_t *ccmni_ctl = ccmni_ctl_blk[ccmni->md_id];
	
	if (unlikely(ccmni_ctl == NULL)) {
		CCMNI_ERR_MSG(ccmni->md_id, "CCMNI%d close: MD%d ctlb is NULL\n", ccmni->index, ccmni->md_id);
		return -1;
	}

	atomic_dec(&ccmni->usage);
	CCMNI_INF_MSG(ccmni->md_id, "CCMNI%d close: cnt=%d\n", ccmni->index, atomic_read(&ccmni->usage));
	netif_stop_queue(dev);
	if(unlikely(ccmni_ctl->ccci_ops->md_ability & MODEM_CAP_NAPI))
		napi_disable(&ccmni->napi);
	
	return 0;
}

static int ccmni_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	int ret;
	int skb_len = skb->len;
	int tx_ch, ccci_tx_ch;
	ccmni_instance_t *ccmni = (ccmni_instance_t *)netdev_priv(dev);
	ccmni_ctl_block_t *ctlb = ccmni_ctl_blk[ccmni->md_id];
	unsigned int is_ack = 0;

	CCMNI_DBG_MSG(ccmni->md_id, "[TX]CCMNI%d head_len=%d,len=%d\n", ccmni->index, skb_headroom(skb), skb->len);

	if(unlikely(skb->len > CCMNI_MTU)) { //???why dev->mtu, if dev->mtu is changed by upper layer
		CCMNI_ERR_MSG(ccmni->md_id, "CCMNI%d write fail: len(0x%x)>MTU(0x%x, 0x%x)\n", ccmni->index, skb->len, CCMNI_MTU, dev->mtu);
		dev_kfree_skb(skb);
		dev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}
	
	if(unlikely(skb_headroom(skb) < sizeof(struct ccci_header))) {
		CCMNI_ERR_MSG(ccmni->md_id, "CCMNI%d write fail: header room(%d) < ccci_header(%d)\n", \
			ccmni->index, skb_headroom(skb), dev->hard_header_len);
		dev_kfree_skb(skb);
		dev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}
	
	ccci_tx_ch = tx_ch = ccmni->ch.tx;
	if (ctlb->ccci_ops->md_ability & MODEM_CAP_DATA_ACK_DVD) {
		if(ccmni->ch.rx == CCCI_CCMNI1_RX || ccmni->ch.rx == CCCI_CCMNI2_RX) {
			is_ack = is_ack_skb(skb);
			if(is_ack) {
				ccci_tx_ch = (ccmni->ch.tx==CCCI_CCMNI1_TX)?CCCI_CCMNI1_DL_ACK:CCCI_CCMNI2_DL_ACK;
				//tx_ch = ctlb->ccmni_inst[1]->ch.tx;
			} else {
				ccci_tx_ch = ccmni->ch.tx;
				//tx_ch = ctlb->ccmni_inst[0]->ch.tx;
			}
		}
	}
#if 0
	struct ccci_header *ccci_h;
	ccci_h = (struct ccci_header*)skb_push(skb, sizeof(struct ccci_header));
	ccci_h->channel = ccci_tx_ch;
	ccci_h->data[0] = 0;
	ccci_h->data[1] = skb->len; // as skb->len already included ccci_header after skb_push

	if (ctlb->ccci_ops->md_ability & MODEM_CAP_CCMNI_SEQNO) {
		ccci_h->reserved = ccmni->tx_seq_num[is_ack]++;
	} else {
		ccci_h->reserved = 0 ;
	}

	CCMNI_DBG_MSG(ccmni->md_id, "[TX]CCMNI%d: 0x%08X, 0x%08X, %08X, 0x%08X\n", 
		ccmni->index, ccci_h->data[0], ccci_h->data[1], ccci_h->channel, ccci_h->reserved);
#endif

	ret = ctlb->ccci_ops->send_pkt(ccmni->md_id, ccci_tx_ch, skb);
	if(ret == CCMNI_ERR_MD_NO_READY || ret == CCMNI_ERR_TX_INVAL) {
		dev_kfree_skb(skb);
		dev->stats.tx_dropped++;
		ccmni->tx_busy_cnt = 0;
		CCMNI_ERR_MSG(ccmni->md_id, "[TX]CCMNI%d send pkt fail: %d\n", ccmni->index, ret);
		return NETDEV_TX_OK;
	} else if (ret == CCMNI_ERR_TX_BUSY) {
#if 0
		skb_pull(skb, sizeof(struct ccci_header)); // undo header, in next retry, we'll reserve header again
#endif
		goto tx_busy;
	}
	dev->stats.tx_packets++;
	dev->stats.tx_bytes += skb_len;
	ccmni->tx_busy_cnt = 0;

	return NETDEV_TX_OK;

tx_busy:
	if(unlikely(!(ctlb->ccci_ops->md_ability & MODEM_CAP_TXBUSY_STOP))) {
		if((++ccmni->tx_busy_cnt)%20000 == 0)
			CCMNI_ERR_MSG(ccmni->md_id, "[TX]CCMNI%d TX busy: retry_times=%ld\n", ccmni->index, ccmni->tx_busy_cnt);
	} else {
		ccmni->tx_busy_cnt++;
	}
	return NETDEV_TX_BUSY;
}

static int ccmni_change_mtu(struct net_device *dev, int new_mtu)
{
	ccmni_instance_t *ccmni = (ccmni_instance_t *)netdev_priv(dev);
	
	if (new_mtu > CCMNI_MTU) {
		CCMNI_ERR_MSG(ccmni->md_id, "CCMNI%d new mtu siz(%d) > MTU(%d)\n", ccmni->index, new_mtu, CCMNI_MTU);
		return -EINVAL;
	} else {
		dev->mtu = new_mtu;
		CCMNI_INF_MSG(ccmni->md_id, "CCMNI%d change mtu_siz=%d\n", ccmni->index, new_mtu);
		return 0;
	}
}

static void ccmni_tx_timeout(struct net_device *dev)
{
	ccmni_instance_t *ccmni = (ccmni_instance_t *)netdev_priv(dev);
	
	CCMNI_INF_MSG(ccmni->md_id, "ccmni%d_tx_timeout: usage_cnt=%d\n", ccmni->index, atomic_read(&ccmni->usage));

	dev->stats.tx_errors++;
	if(atomic_read(&ccmni->usage) > 0)
		netif_wake_queue(dev);
}

static int ccmni_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	int md_id_irat, usage_cnt;
	ccmni_instance_t *ccmni_irat, *ccmni_tmp;
	ccmni_instance_t *ccmni = (ccmni_instance_t *)netdev_priv(dev);
	ccmni_ctl_block_t *ctlb = NULL;

	switch(cmd) {
		case SIOCSTXQSTATE:
			if(!ifr->ifr_ifru.ifru_ivalue) {
				if(atomic_read(&ccmni->usage) > 0) {
					atomic_dec(&ccmni->usage);
					netif_stop_queue(dev);
					dev->watchdog_timeo = 60*HZ; // stop queue won't stop Tx watchdog (ndo_tx_timeout)
					
					ctlb = ccmni_ctl_blk[ccmni->md_id];
					ccmni_tmp = ctlb->ccmni_inst[ccmni->index];
					if (ccmni_tmp != ccmni) { //iRAT ccmni
						usage_cnt = atomic_read(&ccmni->usage);
						atomic_set(&ccmni_tmp->usage, usage_cnt);
					}
				}
			} else {
				if(atomic_read(&ccmni->usage) <=0 ) {
					if(netif_running(dev) && netif_queue_stopped(dev))
						netif_wake_queue(dev);
					dev->watchdog_timeo = CCMNI_NETDEV_WDT_TO;
					atomic_inc(&ccmni->usage);
					
					ctlb = ccmni_ctl_blk[ccmni->md_id];
					ccmni_tmp = ctlb->ccmni_inst[ccmni->index];
					if (ccmni_tmp != ccmni) { //iRAT ccmni
						usage_cnt = atomic_read(&ccmni->usage);
						atomic_set(&ccmni_tmp->usage, usage_cnt);
					}
				}
			}
			CCMNI_INF_MSG(ccmni->md_id, "SIOCSTXQSTATE: set CCMNI%d tx_state=%d, usage_cnt=%d\n", \
				ccmni->index, ifr->ifr_ifru.ifru_ivalue, atomic_read(&ccmni->usage));
			break;
			
		case SIOCCCMNICFG:
			md_id_irat = ifr->ifr_ifru.ifru_ivalue;
			if (md_id_irat < 0 && md_id_irat >= MAX_MD_NUM) {
				CCMNI_ERR_MSG(ccmni->md_id, "SIOCSCCMNICFG: CCMNI%d invalid md_id(%d)\n", \
					ccmni->index, ifr->ifr_ifru.ifru_ivalue);
				return -EINVAL;
			}
			if (md_id_irat == ccmni->md_id) {
				CCMNI_INF_MSG(ccmni->md_id, "SIOCCCMNICFG: CCMNI%d iRAT on the same MD%d, usage_cnt=%d\n", \
					ccmni->index, ifr->ifr_ifru.ifru_ivalue, atomic_read(&ccmni->usage));
				break;
			}
			
			ctlb = ccmni_ctl_blk[md_id_irat];
			ccmni_irat = ctlb->ccmni_inst[ccmni->index];
			usage_cnt = atomic_read(&ccmni->usage);
			atomic_set(&ccmni_irat->usage, usage_cnt);
			memcpy(netdev_priv(dev), ccmni_irat, sizeof(ccmni_instance_t));
			
			CCMNI_INF_MSG(ccmni->md_id, "SIOCCCMNICFG: CCMNI%d iRAT from MD%d to MD%d, usage_cnt=%d\n", \
				ccmni->index, ccmni->md_id, ifr->ifr_ifru.ifru_ivalue, atomic_read(&ccmni->usage));
			break;
			
		default:
			CCMNI_ERR_MSG(ccmni->md_id, "CCMNI%d: unknown ioctl cmd=%x\n", ccmni->index, cmd);
			break;
	}

	return 0;
}

static const struct net_device_ops ccmni_netdev_ops = 
{
	.ndo_open		= ccmni_open,
	.ndo_stop		= ccmni_close,
	.ndo_start_xmit	= ccmni_start_xmit,
	.ndo_tx_timeout	= ccmni_tx_timeout,
	.ndo_do_ioctl   = ccmni_ioctl,
	.ndo_change_mtu = ccmni_change_mtu,
};

static int ccmni_napi_poll(struct napi_struct *napi ,int budget)
{
	ccmni_instance_t *ccmni = (ccmni_instance_t *)netdev_priv(napi->dev);
	int md_id = ccmni->md_id;
	ccmni_ctl_block_t *ctlb = ccmni_ctl_blk[md_id];
	
	del_timer(&ccmni->timer);

	if (ctlb->ccci_ops->napi_poll)
		return ctlb->ccci_ops->napi_poll(md_id, ccmni->ch.rx, napi, budget);
	else
		return 0;
}

static void ccmni_napi_poll_timeout(unsigned long data)
{
	ccmni_instance_t *ccmni = (ccmni_instance_t *)data;
	CCMNI_ERR_MSG(ccmni->md_id, "CCMNI%d lost NAPI polling\n", ccmni->index);
}


/********************ccmni driver register  ccci function********************/
static inline int ccmni_inst_init(int md_id, ccmni_instance_t * ccmni, struct net_device *dev)
{
	ccmni_ctl_block_t *ctlb = ccmni_ctl_blk[md_id];
	struct ccmni_ch channel;
	int ret = 0;
	
	ret = ctlb->ccci_ops->get_ccmni_ch(md_id, ccmni->index, &channel);
	if (ret) {
		CCMNI_ERR_MSG(md_id, "get ccmni%d channel fail\n", ccmni->index);
		return ret;
	}
			
	ccmni->dev = dev;
	ccmni->ctlb = ctlb;
	ccmni->md_id = md_id;

	//ccmni tx/rx channel setting 
	ccmni->ch.rx = channel.rx;
	ccmni->ch.rx_ack = channel.rx_ack;
	ccmni->ch.tx = channel.tx;
	ccmni->ch.tx_ack = channel.tx_ack;
			
	//register napi device
	if(dev && (ctlb->ccci_ops->md_ability & MODEM_CAP_NAPI)) {
		init_timer(&ccmni->timer);
		ccmni->timer.function = ccmni_napi_poll_timeout;
		ccmni->timer.data = (unsigned long)ccmni;
		netif_napi_add(dev, &ccmni->napi, ccmni_napi_poll, ctlb->ccci_ops->napi_poll_weigh);
	}
	
	atomic_set(&ccmni->usage, 0);
	spin_lock_init(&ccmni->spinlock);

	return ret;
}

static int ccmni_init(int md_id, ccmni_ccci_ops_t *ccci_info)
{
	int i = 0, j = 0, ret = 0;
	ccmni_ctl_block_t *ctlb = NULL;
	ccmni_ctl_block_t * ctlb_irat_src = NULL;
	ccmni_instance_t  *ccmni = NULL;
	ccmni_instance_t  *ccmni_irat_src = NULL;
	struct net_device *dev = NULL;

	if(unlikely(ccci_info->md_ability & MODEM_CAP_CCMNI_DISABLE)) {
		CCMNI_ERR_MSG(md_id, "no need init ccmni: md_ability=0x%08X\n", ccci_info->md_ability);
		return 0;
	}

	ctlb = kzalloc(sizeof(ccmni_ctl_block_t), GFP_KERNEL);
	if (unlikely(ctlb == NULL)) {		
		CCMNI_ERR_MSG(md_id, "alloc ccmni ctl struct fail\n");
		return -ENOMEM;
	}

	ctlb->ccci_ops = kzalloc(sizeof(ccmni_ccci_ops_t), GFP_KERNEL);
	if (unlikely(ctlb->ccci_ops == NULL)) {		
		CCMNI_ERR_MSG(md_id, "alloc ccmni_ccci_ops struct fail\n");
		ret = -ENOMEM;
		goto alloc_mem_fail;
	}
	
	ccmni_ctl_blk[md_id] = ctlb;

	memcpy(ctlb->ccci_ops, ccci_info, sizeof(ccmni_ccci_ops_t));

#if 0
	ctlb->ccci_ops->ccmni_ver = ccci_info->ccmni_ver;
	ctlb->ccci_ops->ccmni_num = ccci_info->ccmni_num;
	ctlb->ccci_ops->md_ability = ccci_info->md_ability;
	ctlb->ccci_ops->irat_md_id = ccci_info->irat_md_id;
	ctlb->ccci_ops->send_pkt = ccci_info->send_pkt;
	ctlb->ccci_ops->get_ccmni_ch = ccci_info->get_ccmni_ch;
	ctlb->ccci_ops->napi_poll_weigh = ccci_info->napi_poll_weigh;
	ctlb->ccci_ops->napi_poll = ccci_info->napi_poll;
	memcpy(ctlb->ccci_ops->name, ccci_info->name, sizeof(ccci_info->name));
#endif

	CCMNI_INF_MSG(md_id, "ccmni_init: ccmni_num=%d, md_ability=0x%08x, irat_en=%08x, irat_md_id=%d, send_pkt=%p, get_ccmni_ch=%p, name=%s\n", 
					ctlb->ccci_ops->ccmni_num, ctlb->ccci_ops->md_ability, (ctlb->ccci_ops->md_ability & MODEM_CAP_CCMNI_IRAT), \
					ctlb->ccci_ops->irat_md_id, ctlb->ccci_ops->send_pkt, ctlb->ccci_ops->get_ccmni_ch, ctlb->ccci_ops->name);

	if ((ctlb->ccci_ops->md_ability & MODEM_CAP_CCMNI_IRAT) == 0) {
		for (i = 0; i < ctlb->ccci_ops->ccmni_num; i++) {
			//allocate netdev
			dev = alloc_etherdev(sizeof(ccmni_instance_t));
			if (unlikely(dev == NULL)) {
				CCMNI_ERR_MSG(md_id, "alloc netdev fail\n");
				ret = -ENOMEM;
				goto alloc_netdev_fail;
			}

			//init net device
			dev->header_ops = NULL;
			dev->mtu = CCMNI_MTU;
			dev->tx_queue_len = CCMNI_TX_QUEUE;
			dev->watchdog_timeo = CCMNI_NETDEV_WDT_TO;
			dev->flags = IFF_NOARP & /* ccmni is a pure IP device */ 
					(~IFF_BROADCAST & ~IFF_MULTICAST);	/* ccmni is P2P */
			dev->features = NETIF_F_VLAN_CHALLENGED; /* not support VLAN */
			dev->addr_len = ETH_ALEN; /* ethernet header size */
			dev->destructor = free_netdev;
			dev->hard_header_len += sizeof(struct ccci_header); /* reserve Tx CCCI header room */
			dev->netdev_ops = &ccmni_netdev_ops;
			random_ether_addr((u8 *) dev->dev_addr);
			
			sprintf(dev->name, "%s%d", ctlb->ccci_ops->name, i);
			CCMNI_INF_MSG(md_id, "register netdev name: %s\n", dev->name);

			//init private structure of netdev
			ccmni = netdev_priv(dev);	
			ccmni->index = i;
			ret = ccmni_inst_init(md_id, ccmni, dev);
			if (ret) {
				CCMNI_ERR_MSG(md_id, "initial ccmni instance fail\n");
				goto alloc_netdev_fail;
			}
			ctlb->ccmni_inst[i] = ccmni;
			
			//register net device
			ret = register_netdev(dev);
			if (ret) {
				CCMNI_ERR_MSG(md_id, "CCMNI%d register netdev fail: %d\n", i, ret);
				goto alloc_netdev_fail;
			}
			
			CCMNI_INF_MSG(ccmni->md_id, "CCMNI%d=%p, ctlb=%p, ctlb_ops=%p, dev=%p\n", 
				i, ccmni, ccmni->ctlb, ccmni->ctlb->ccci_ops, ccmni->dev);
		}
	} else {
		if (ctlb->ccci_ops->irat_md_id < 0 || ctlb->ccci_ops->irat_md_id > MAX_MD_NUM) {
			CCMNI_ERR_MSG(md_id, "md%d IRAT fail because invalid irat md(%d)\n", md_id, ctlb->ccci_ops->irat_md_id);
			ret = -EINVAL;
			goto alloc_mem_fail;
		}
		
		ctlb_irat_src = ccmni_ctl_blk[ctlb->ccci_ops->irat_md_id];
		if (!ctlb_irat_src) {
			CCMNI_ERR_MSG(md_id, "md%d IRAT fail because irat md%d ctlb is NULL\n", md_id, ctlb->ccci_ops->irat_md_id);
			ret = -EINVAL;
			goto alloc_mem_fail;
		}
		
		if (unlikely(ctlb->ccci_ops->ccmni_num != ctlb_irat_src->ccci_ops->ccmni_num)) {
			CCMNI_ERR_MSG(md_id, "IRAT fail because number of src(%d) and dest(%d) ccmni isn't equal\n", \
				ctlb_irat_src->ccci_ops->ccmni_num, ctlb->ccci_ops->ccmni_num);
			ret = -EINVAL;
			goto alloc_mem_fail;
		}
		
		for (i = 0; i < ctlb->ccci_ops->ccmni_num; i++) {
			ccmni = kzalloc(sizeof(ccmni_instance_t), GFP_KERNEL);
			ccmni_irat_src = kzalloc(sizeof(ccmni_instance_t), GFP_KERNEL);
			if (unlikely(ccmni == NULL || ccmni_irat_src == NULL)) {		
				CCMNI_ERR_MSG(md_id, "alloc ccmni instance fail\n");
				ret = -ENOMEM;
				goto alloc_mem_fail;
			}
			
			// inital irat ccmni instance
			ccmni->index = i;
			dev = ctlb_irat_src->ccmni_inst[i]->dev;
			ret = ccmni_inst_init(md_id, ccmni, dev);
			if (ret) {
				CCMNI_ERR_MSG(md_id, "initial ccmni instance fail\n");
				goto alloc_mem_fail;
			}
			ctlb->ccmni_inst[i] = ccmni;
			
			// inital irat source ccmni instance
			memcpy(ccmni_irat_src, ctlb_irat_src->ccmni_inst[i], sizeof(ccmni_instance_t));
			ctlb_irat_src->ccmni_inst[i] = ccmni_irat_src;
			CCMNI_INF_MSG(ccmni->md_id, "[IRAT]CCMNI%d=%p, ctlb=%p, ctlb_ops=%p, dev=%p\n", 
				i, ccmni, ccmni->ctlb, ccmni->ctlb->ccci_ops, ccmni->dev);
		}
	}
	
	snprintf(ctlb->wakelock_name, sizeof(ctlb->wakelock_name), "ccmni_md%d", (md_id+1));	
	wake_lock_init(&ctlb->ccmni_wakelock, WAKE_LOCK_SUSPEND, ctlb->wakelock_name);
	
	return 0;
	
alloc_netdev_fail:
	if (dev) {
		free_netdev(dev);
		ctlb->ccmni_inst[i] = NULL;
	}

	for (j = i-1; j >= 0; j--) {
		ccmni = ctlb->ccmni_inst[j];
		unregister_netdev(ccmni->dev);
		//free_netdev(ccmni->dev);
		ctlb->ccmni_inst[j] = NULL;
	}
	
alloc_mem_fail:
	if (ctlb->ccci_ops) {
		kfree(ctlb->ccci_ops);
	}
	kfree(ctlb);
	ccmni_ctl_blk[md_id] = NULL;
	return ret;
}

static void ccmni_exit(int md_id)
{
	int i = 0;
	ccmni_ctl_block_t *ctlb = NULL;
	ccmni_instance_t  *ccmni = NULL;

	CCMNI_INF_MSG(md_id, "ccmni_exit\n");
	
	ctlb = ccmni_ctl_blk[md_id];
	if (ctlb) {
		if (ctlb->ccci_ops == NULL) {
			goto ccmni_exit_ret;
		}
		
		for (i = 0; i < ctlb->ccci_ops->ccmni_num; i++) {
			ccmni = ctlb->ccmni_inst[i];
			if (ccmni) {
				CCMNI_INF_MSG(md_id, "ccmni_exit: unregister ccmni%d dev\n", i);
				unregister_netdev(ccmni->dev);
				//free_netdev(ccmni->dev);
				ctlb->ccmni_inst[i] = NULL;
			}
		}
		
		kfree(ctlb->ccci_ops);
		
ccmni_exit_ret:
		kfree(ctlb);
		ccmni_ctl_blk[md_id] = NULL;
	}
}

static int ccmni_rx_callback(int md_id, int rx_ch, struct sk_buff *skb, void *priv_data)
{
	ccmni_ctl_block_t *ctlb = ccmni_ctl_blk[md_id];
//	struct ccci_header *ccci_h = (struct ccci_header*)skb->data;
	ccmni_instance_t *ccmni = NULL;
	struct net_device *dev = NULL;
	int pkt_type, skb_len, ccmni_idx;

	if (unlikely(ctlb == NULL || ctlb->ccci_ops == NULL)) {
		CCMNI_ERR_MSG(md_id, "invalid CCMNI ctrl/ops struct for RX_CH(%d)\n", rx_ch);
		dev_kfree_skb(skb);
		return -1;
	}
	
#if 0
	CCMNI_INF_MSG(md_id, "[RX]-1: 0x%08X, 0x%08X, %08X, 0x%08X\n", 
					ccci_h->data[0], ccci_h->data[1], ccci_h->channel, ccci_h->reserved);

	if (ctlb->ccci_ops->md_ability & MODEM_CAP_CCMNI_SEQNO) {
		if(unlikely(ccmni->rx_seq_num!=0 && (ccci_h->reserved - ccmni->rx_seq_num)!=1)) {
			CCMNI_ERR_MSG(ccmni->md_id, "CCMNI%d rx packet seqno error: %d->%d(exp)\n", 
				ccmni->index, ccci_h->reserved, (ccmni->rx_seq_num+1));
		}
		ccmni->rx_seq_num = ccci_h->reserved;
	}
	CCMNI_INF_MSG(md_id, "[RX]-2: 0x%08X, 0x%08X, %08X, 0x%08X\n", 
				ccci_h->data[0], ccci_h->data[1], ccci_h->channel, ccci_h->reserved);
	ch = ccci_h->channel & 0xFF;
#endif

	ccmni_idx = get_ccmni_idx_from_ch(md_id, rx_ch);
	if (unlikely(ccmni_idx < 0)) {
		CCMNI_ERR_MSG(md_id, "CCMNI rx(%d) skb ch error\n", rx_ch);
		dev_kfree_skb(skb);
		return -1;
	}
	ccmni = ctlb->ccmni_inst[ccmni_idx];
	dev = ccmni->dev;
	
	CCMNI_DBG_MSG(md_id, "[RX]CCMNI%d(rx_ch=%d) receive data_len=%d\n", ccmni_idx, rx_ch, skb->len);
	
//	skb_pull(skb, sizeof(struct ccci_header));
	pkt_type = skb->data[0] & 0xF0;
	ccmni_make_etherframe(skb->data-ETH_HLEN, dev->dev_addr, pkt_type);
    skb_set_mac_header(skb, -ETH_HLEN);
	skb->dev = dev;
	if(pkt_type == 0x60) {            
		skb->protocol  = htons(ETH_P_IPV6);
	} else {
		skb->protocol  = htons(ETH_P_IP);
	}
	skb->ip_summed = CHECKSUM_NONE;
	skb_len = skb->len;
	
	if(likely(ctlb->ccci_ops->md_ability & MODEM_CAP_NAPI)) {
		netif_receive_skb(skb);
	} else {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0))
		if(!in_interrupt()) {
			netif_rx_ni(skb);
		} else {
			netif_rx(skb);
		}
#else
		netif_rx(skb);
#endif
	}
	dev->stats.rx_packets++;
	dev->stats.rx_bytes += skb_len;

	wake_lock_timeout(&ctlb->ccmni_wakelock, HZ);
	
	return 0;
}


static void ccmni_md_state_callback(int md_id, int rx_ch, MD_STATE state)
{
	ccmni_ctl_block_t *ctlb = ccmni_ctl_blk[md_id];
	ccmni_instance_t  *ccmni = NULL;
	int ccmni_idx = 0;
	
	if (unlikely(ctlb == NULL)) {
		CCMNI_ERR_MSG(md_id, "invalid ccmni ctrl struct when rx_ch=%d md_sta=%d\n", rx_ch, state);
		return;
	}
	
	ccmni_idx = get_ccmni_idx_from_ch(md_id, rx_ch);
 	if (unlikely(ccmni_idx < 0)) {
		CCMNI_ERR_MSG(md_id, "get error ccmni index when md_sta=%d\n", state);
		return;
 	}
		
	ccmni = ctlb->ccmni_inst[ccmni_idx];

	if (state != TX_IRQ) {
		CCMNI_INF_MSG(md_id, "ccmni_md_state_callback: CCMNI%d, md_sta=%d, usage=%d\n", \
			ccmni_idx, state, atomic_read(&ccmni->usage));
	}
		
	switch(state) {
		case READY:
			netif_carrier_on(ccmni->dev);
			ccmni->tx_seq_num[0] = 0;
			ccmni->tx_seq_num[1] = 0;
			ccmni->rx_seq_num = 0;
			break;
			
		case EXCEPTION:
		case RESET:
			netif_carrier_off(ccmni->dev);
			break;

		case RX_IRQ:
			mod_timer(&ccmni->timer, jiffies+HZ);
			napi_schedule(&ccmni->napi);
			wake_lock_timeout(&ctlb->ccmni_wakelock, HZ);
			break;
			
		case TX_IRQ:
			if(netif_running(ccmni->dev) && netif_queue_stopped(ccmni->dev) && atomic_read(&ccmni->usage)>0) {
				netif_wake_queue(ccmni->dev);
				//ccmni->flags &= ~PORT_F_RX_FULLED;
				CCMNI_INF_MSG(md_id, "ccmni_md_state_callback: CCMNI%d, md_sta=%d, usage=%d\n", \
					ccmni_idx, state, atomic_read(&ccmni->usage));
			}
			break;
			
		case TX_FULL:
			netif_stop_queue(ccmni->dev);
			//ccmni->flags |= PORT_F_RX_FULLED; // for convenient in traffic log
			break;
		default:
			break;
	}
}

static void ccmni_dump(int md_id, int rx_ch, unsigned int flag)
{
	ccmni_ctl_block_t *ctlb = ccmni_ctl_blk[md_id];
	ccmni_instance_t *ccmni = NULL;
	int ccmni_idx = 0;
	
	if (ctlb == NULL)
		return;
		
	ccmni_idx = get_ccmni_idx_from_ch(md_id, rx_ch);
	if (unlikely(ccmni_idx < 0)) {
		CCMNI_ERR_MSG(md_id, "CCMNI rx(%d) skb ch error\n", rx_ch);
		return;
	}	
	
	ccmni = ctlb->ccmni_inst[ccmni_idx];
	if (unlikely(ccmni == NULL))
		return;
	
	if ((ccmni->dev->stats.rx_packets == 0) && (ccmni->dev->stats.tx_packets == 0))
		return;

	CCMNI_INF_MSG(md_id, "CCMNI%d(%d), rx=(%ld,%ld), tx=(%ld, %ld), tx_busy=%ld\n", \
				ccmni->index, atomic_read(&ccmni->usage), ccmni->dev->stats.rx_packets, ccmni->dev->stats.rx_bytes, \
				ccmni->dev->stats.tx_packets, ccmni->dev->stats.tx_bytes, ccmni->tx_busy_cnt);
	
	return;
}


struct ccmni_dev_ops ccmni_ops = {
	.skb_alloc_size = 1600,
	.init = &ccmni_init,
	.rx_callback = &ccmni_rx_callback,
	.md_state_callback = &ccmni_md_state_callback,
	.exit = ccmni_exit,
	.dump = ccmni_dump,
};

