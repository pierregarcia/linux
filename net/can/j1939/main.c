/*
 * Copyright (c) 2010-2011 EIA Electronics
 *
 * Authors:
 * Kurt Van Dijck <kurt.van.dijck@eia.be>
 * Pieter Beyens <pieter.beyens@eia.be>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the version 2 of the GNU General Public License
 * as published by the Free Software Foundation
 */

/*
 * Core of can-j1939 that links j1939 to CAN.
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/socket.h>
#include <linux/list.h>
#include <linux/if_arp.h>
#include <net/tcp_states.h>

#include <linux/can.h>
#include <linux/can/core.h>
#include "j1939-priv.h"

MODULE_DESCRIPTION("PF_CAN SAE J1939");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("EIA Electronics (Kurt Van Dijck & Pieter Beyens)");

const char j1939_procname[] = "can-j1939";
struct proc_dir_entry *j1939_procdir;

static struct {
	struct notifier_block notifier;
} s;

/* LOWLEVEL CAN interface */

/* CAN_HDR: #bytes before can_frame data part */
#define CAN_HDR	(offsetof(struct can_frame, data))
/* CAN_FTR: #bytes beyond data part */
#define CAN_FTR	(sizeof(struct can_frame)-CAN_HDR-\
		sizeof(((struct can_frame *)0)->data))

static void j1939_recv_ecu_flags(struct sk_buff *skb, void *data)
{
	struct j1939_priv *priv = data;
	struct j1939_sk_buff_cb *cb = (void *)skb->cb;
	struct addr_ent *paddr;

	if (!priv)
		return;
	write_lock_bh(&priv->lock);
	if (j1939_address_is_unicast(cb->src.addr)) {
		paddr = &priv->ents[cb->src.addr];
		paddr->rxtime = ktime_get();
		if (0x0ee00 == cb->pgn) {
			/* do not touch many things for Address claims */
		} else if (paddr->ecu) {
			paddr->ecu->rxtime = paddr->rxtime;
			cb->src.flags = paddr->ecu->flags;
		} else {
			if (!paddr->flags)
				paddr->flags |= ECUFLAG_REMOTE;
			cb->src.flags = paddr->flags;
		}
	}

	if (j1939_address_is_unicast(cb->dst.addr)) {
		paddr = &priv->ents[cb->dst.addr];
		if (paddr->ecu)
			cb->dst.flags = paddr->ecu->flags;
		else
			cb->dst.flags = paddr->flags ?: ECUFLAG_REMOTE;
	}
	write_unlock_bh(&priv->lock);
}

/* lowest layer */
static void j1939_can_recv(struct sk_buff *skb, void *data)
{
	int orig_len;
	struct j1939_sk_buff_cb *sk_addr;
	struct can_frame *msg;
	uint8_t saved_cb[sizeof(skb->cb)];

	BUILD_BUG_ON(sizeof(*sk_addr) > sizeof(skb->cb));
	/*
	 * get a pointer to the header of the skb
	 * the skb payload (pointer) is moved, so that the next skb_data
	 * returns the actual payload
	 */
	msg = (void *)skb->data;
	orig_len = skb->len;
	skb_pull(skb, CAN_HDR);
	/* fix length, set to dlc, with 8 maximum */
	skb_trim(skb, min_t(uint8_t, msg->can_dlc, 8));

	/* set addr */
	sk_addr = (struct j1939_sk_buff_cb *)skb->cb;
	memcpy(saved_cb, sk_addr, sizeof(saved_cb));
	memset(sk_addr, 0, sizeof(*sk_addr));
	if (skb->dev)
		sk_addr->ifindex = skb->dev->ifindex;
	sk_addr->priority = (msg->can_id & 0x1c000000) >> 26;
	sk_addr->src.addr = msg->can_id & 0xff;
	sk_addr->pgn = (msg->can_id & 0x3ffff00) >> 8;
	if (pgn_is_pdu1(sk_addr->pgn)) {
		/* Type 1: with destination address */
		sk_addr->dst.addr = sk_addr->pgn & 0xff;
		/* normalize pgn: strip dst address */
		sk_addr->pgn &= 0x3ff00;
	} else {
		/* set broadcast address */
		sk_addr->dst.addr = J1939_NO_ADDR;
	}
	j1939_recv_ecu_flags(skb, data);

	if (j1939_recv_address_claim(skb))
		goto done;
	if (j1939_recv_promisc(skb))
		goto done;
	if (j1939_recv_transport(skb))
		goto done;
	j1939_recv(skb);
done:
	/* restore the original skb, should always work */
	skb_push(skb, CAN_HDR);
	/* no safety check, it just restores the skbuf's contents */
	__skb_trim(skb, orig_len);
	memcpy(sk_addr, saved_cb, sizeof(saved_cb));
}

static int j1939_send_can(struct sk_buff *skb)
{
	int ret, dlc;
	canid_t canid;
	struct j1939_sk_buff_cb *sk_addr;
	struct net_device *netdev = NULL;
	struct can_frame *msg;

	dlc = skb->len;
	if (dlc > 8)
		return -EMSGSIZE;
	ret = pskb_expand_head(skb, SKB_DATA_ALIGN(CAN_HDR),
			CAN_FTR + (8-dlc), GFP_ATOMIC);
	if (ret < 0)
		return ret;

	msg = (void *)skb_push(skb, CAN_HDR);
	BUG_ON(!msg);
	/* make it a full can frame */
	skb_put(skb, CAN_FTR + (8 - dlc));

	sk_addr = (struct j1939_sk_buff_cb *)skb->cb;
	canid = CAN_EFF_FLAG |
		(sk_addr->src.addr & 0xff) |
		((sk_addr->priority & 0x7) << 26);
	if (pgn_is_pdu1(sk_addr->pgn))
		canid |= ((sk_addr->pgn & 0x3ff00) << 8) |
			((sk_addr->dst.addr & 0xff) << 8);
	else
		canid |= ((sk_addr->pgn & 0x3ffff) << 8);

	msg->can_id = canid;
	msg->can_dlc = dlc;

	/* set net_device */
	ret = -ENODEV;
	if (!skb->dev) {
		if (!sk_addr->ifindex)
			goto failed;
		netdev = dev_get_by_index(&init_net, sk_addr->ifindex);
		if (!netdev)
			goto failed;
		skb->dev = netdev;
	}

	/* fix the 'always free' policy of can_send */
	skb = skb_get(skb);
	ret = can_send(skb, 1);
	if (!ret) {
		/* free when can_send succeeded */
		kfree_skb(skb);
		/* is this necessary ? */
		ret = RESULT_STOP;
	}
failed:
	if (netdev)
		dev_put(netdev);
	return ret;
}

int j1939_send(struct sk_buff *skb)
{
	struct j1939_sk_buff_cb *cb = (void *)skb->cb;
	struct j1939_priv *priv;
	struct addr_ent *paddr;
	struct j1939_ecu *ecu;
	int ret = 0;

	/* apply sanity checks */
	cb->pgn &= (pgn_is_pdu1(cb->pgn)) ? 0x3ff00 : 0x3ffff;
	if (cb->priority > 7)
		cb->priority = 6;

	/* verify source */
	if (!cb->ifindex)
		return -ENETUNREACH;

	priv = j1939_priv_find(cb->ifindex);
	if (!priv)
		return -ENETUNREACH;

	sock_hold(skb->sk);

	read_lock_bh(&priv->lock);
	/* verify source */
	if (cb->src.name) {
		ecu = j1939_ecu_find_by_name(cb->src.name, cb->ifindex);
		cb->src.flags = ecu ? ecu->flags : 0;
		if (ecu)
			put_j1939_ecu(ecu);
	} else if (j1939_address_is_unicast(cb->src.addr)) {
		paddr = &priv->ents[cb->src.addr];
		cb->src.flags = paddr->flags;
	} else if (cb->src.addr == J1939_IDLE_ADDR) {
		/* allow always */
		cb->src.flags = ECUFLAG_LOCAL;
	} else {
		/* J1939_NO_ADDR */
		cb->src.flags = 0;
	}
	if (cb->src.flags & ECUFLAG_REMOTE) {
		ret = -EREMOTE;
		goto done;
	} else if (!(cb->src.flags & ECUFLAG_LOCAL)) {
		ret = -EADDRNOTAVAIL;
		goto done;
	}

	/* verify destination */
	if (cb->dst.name) {
		ecu = j1939_ecu_find_by_name(cb->dst.name, cb->ifindex);
		if (!ecu) {
			ret = -EADDRNOTAVAIL;
			goto done;
		}
		cb->dst.flags = ecu->flags;
		put_j1939_ecu(ecu);
	} else if (cb->dst.addr == J1939_IDLE_ADDR) {
		/* not a valid destination */
		ret = -EADDRNOTAVAIL;
		goto done;
	} else if (j1939_address_is_unicast(cb->dst.addr)) {
		paddr = &priv->ents[cb->dst.addr];
		cb->dst.flags = paddr->flags;
	} else {
		cb->dst.flags = 0;
	}

	ret = j1939_send_transport(skb);
	if (unlikely(ret))
		goto done;
	ret = j1939_send_address_claim(skb);
	if (unlikely(ret))
		goto done;
	ret = j1939_send_can(skb);
done:
	/* don't mark as failed, it can't be better */
	if (ret == RESULT_STOP)
		ret = 0;
	read_unlock_bh(&priv->lock);
	sock_put(skb->sk);
	put_j1939_priv(priv);
	return ret;
}
EXPORT_SYMBOL_GPL(j1939_send);

/* send a packet of max 8 bytes, without testing metadata */
int j1939_send_normalized_pkt(struct sk_buff *skb)
{
	int ret;

	ret = j1939_send_address_claim(skb);
	if (unlikely(ret))
		return ret;
	return j1939_send_can(skb);
}


/* NETDEV MANAGEMENT */

/* values for can_rx_(un)register */
#define J1939_CAN_ID	CAN_EFF_FLAG
#define J1939_CAN_MASK	(CAN_EFF_FLAG | CAN_RTR_FLAG)

int netdev_enable_j1939(struct net_device *netdev)
{
	int ret;
	struct j1939_priv *priv;
	struct dev_rcv_lists *can_ml_priv;

	BUG_ON(!netdev);
	if (netdev->type != ARPHRD_CAN)
		return -EAFNOSUPPORT;

	/* create/stuff j1939_priv */
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	tasklet_init(&priv->ac_task, j1939_priv_ac_task, (unsigned long)priv);
	rwlock_init(&priv->lock);
	INIT_LIST_HEAD(&priv->ecus);
	INIT_LIST_HEAD(&priv->flist);
	priv->ifindex = netdev->ifindex;
	kref_init(&priv->kref);

	/* link into netdev */
	spin_lock(&can_rcvlists_lock);
	can_ml_priv = netdev->ml_priv;
	if (!can_ml_priv->j1939_priv)
		can_ml_priv->j1939_priv = priv;
	ret = (can_ml_priv->j1939_priv == priv) ? 0 : -EBUSY;
	spin_unlock(&can_rcvlists_lock);
	if (ret < 0)
		goto fail_register;

	/* add CAN handler */
	ret = can_rx_register(netdev, J1939_CAN_ID, J1939_CAN_MASK,
			j1939_can_recv, priv, "j1939");
	if (ret < 0)
		goto fail_can_rx;
	return 0;

fail_can_rx:
	netdev_disable_j1939(netdev);
	return ret;

fail_register:
	kfree(priv);
	return ret;
}

int netdev_disable_j1939(struct net_device *netdev)
{
	struct dev_rcv_lists *can_ml_priv;
	struct j1939_priv *priv;
	struct j1939_ecu *ecu;

	BUG_ON(!netdev);

	/* unlink from netdev */
	spin_lock(&can_rcvlists_lock);
	can_ml_priv = netdev->ml_priv;
	priv = can_ml_priv->j1939_priv;
	can_ml_priv->j1939_priv = NULL;
	spin_unlock(&can_rcvlists_lock);
	if (!priv)
		return 0;
	can_rx_unregister(netdev, J1939_CAN_ID, J1939_CAN_MASK,
			j1939_can_recv, priv);

	/* cleanup priv */
	write_lock_bh(&priv->lock);
	while (!list_empty(&priv->ecus)) {
		ecu = list_first_entry(&priv->ecus, struct j1939_ecu, list);
		write_unlock_bh(&priv->lock);
		j1939_ecu_unregister(ecu);
		write_lock_bh(&priv->lock);
	}
	write_unlock_bh(&priv->lock);

	/* final put */
	put_j1939_priv(priv);
	/* notify j1939 sockets */
	j1939sk_netdev_event(netdev->ifindex, EHOSTDOWN);
	return 0;
}


static int j1939_notifier(struct notifier_block *nb,
			unsigned long msg, void *data)
{
	struct net_device *netdev = (struct net_device *)data;

	if (!net_eq(dev_net(netdev), &init_net))
		return NOTIFY_DONE;

	if (netdev->type != ARPHRD_CAN)
		return NOTIFY_DONE;

	switch (msg) {
	case NETDEV_UNREGISTER:
		netdev_disable_j1939(netdev);
		j1939sk_netdev_event(netdev->ifindex, ENODEV);
		break;

	case NETDEV_DOWN:
		j1939sk_netdev_event(netdev->ifindex, ENETDOWN);
		break;
	}

	return NOTIFY_DONE;
}

/* MODULE interface */

static __init int j1939_module_init(void)
{
	int ret;

	pr_info("can: SAE J1939\n");

	/* create /proc/net/can directory */
	j1939_procdir = proc_mkdir(j1939_procname, init_net.proc_net);
	if (!j1939_procdir)
		return -EINVAL;

	s.notifier.notifier_call = j1939_notifier;
	register_netdevice_notifier(&s.notifier);

	ret = j1939sk_module_init();
	if (ret < 0)
		goto fail_sk;
	ret = j1939tp_module_init();
	if (ret < 0)
		goto fail_tp;
	return 0;

	j1939tp_module_exit();
fail_tp:
	j1939sk_module_exit();
fail_sk:
	unregister_netdevice_notifier(&s.notifier);
	proc_net_remove(&init_net, j1939_procname);
	return ret;
}

static __exit void j1939_module_exit(void)
{
	struct net_device *netdev;

	/* shutdown j1939 for all netdevs */
	for_each_netdev(&init_net, netdev) {
		if (netdev->type != ARPHRD_CAN)
			continue;
		/* netdev disable only disables when j1939 active */
		netdev_disable_j1939(netdev);
	}

	j1939tp_module_exit();
	j1939sk_module_exit();

	unregister_netdevice_notifier(&s.notifier);

	proc_net_remove(&init_net, j1939_procname);
}

module_init(j1939_module_init);
module_exit(j1939_module_exit);
