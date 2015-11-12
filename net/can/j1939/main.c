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
MODULE_ALIAS("can-proto-" __stringify(CAN_J1939));

const char j1939_procname[] = "can-j1939";
struct proc_dir_entry *j1939_procdir;

/* LOWLEVEL CAN interface */

/* CAN_HDR: #bytes before can_frame data part */
#define CAN_HDR	(offsetof(struct can_frame, data))
/* CAN_FTR: #bytes beyond data part */
#define CAN_FTR	(sizeof(struct can_frame)-CAN_HDR-\
		sizeof(((struct can_frame *)0)->data))

static unsigned int padding = 0;

module_param_named(padding, padding, uint, 0644);

MODULE_PARM_DESC(padding, "Pad all packets to 8 bytes, and stuff with 0xff");

static void j1939_recv_ecu_flags(struct sk_buff *skb, void *data)
{
	struct j1939_priv *priv = data;
	struct j1939_sk_buff_cb *cb = (void *)skb->cb;
	struct addr_ent *paddr;

	if (!priv)
		return;
	write_lock_bh(&priv->lock);
	if (j1939_address_is_unicast(cb->srcaddr)) {
		paddr = &priv->ents[cb->srcaddr];
		paddr->rxtime = ktime_get();
		if (0x0ee00 == cb->pgn) {
			/* do not touch many things for Address claims */
		} else if (paddr->ecu) {
			paddr->ecu->rxtime = paddr->rxtime;
			cb->srcflags = paddr->ecu->flags;
		} else {
			if (!paddr->flags)
				paddr->flags |= ECUFLAG_REMOTE;
			cb->srcflags = paddr->flags;
		}
	}

	if (j1939_address_is_unicast(cb->dstaddr)) {
		paddr = &priv->ents[cb->dstaddr];
		if (paddr->ecu)
			cb->dstflags = paddr->ecu->flags;
		else
			cb->dstflags = paddr->flags ?: ECUFLAG_REMOTE;
	}
	write_unlock_bh(&priv->lock);
}

/* lowest layer */
static void j1939_can_recv(struct sk_buff *iskb, void *data)
{
	struct sk_buff *skb;
	struct j1939_sk_buff_cb *sk_addr;
	struct can_frame *cf;

	BUILD_BUG_ON(sizeof(*sk_addr) > sizeof(skb->cb));

	/* create a copy of the skb
	 * j1939 only delivers the real data bytes,
	 * the header goes into sockaddr.
	 * j1939 may not touch the incoming skb in such way
	 */
	skb = skb_clone(iskb, GFP_ATOMIC);
	/*
	 * get a pointer to the header of the skb
	 * the skb payload (pointer) is moved, so that the next skb_data
	 * returns the actual payload
	 */
	cf = (void *)skb->data;
	skb_pull(skb, CAN_HDR);
	/* fix length, set to dlc, with 8 maximum */
	skb_trim(skb, min_t(uint8_t, cf->can_dlc, 8));

	/* set addr */
	sk_addr = (struct j1939_sk_buff_cb *)skb->cb;
	memset(sk_addr, 0, sizeof(*sk_addr));
	/* save incoming socket, without assigning the skb to it */
	sk_addr->insock = iskb->sk;
	sk_addr->priority = (cf->can_id & 0x1c000000) >> 26;
	sk_addr->srcaddr = cf->can_id & 0xff;
	sk_addr->pgn = (cf->can_id & 0x3ffff00) >> 8;
	if (pgn_is_pdu1(sk_addr->pgn)) {
		/* Type 1: with destination address */
		sk_addr->dstaddr = sk_addr->pgn & 0xff;
		/* normalize pgn: strip dst address */
		sk_addr->pgn &= 0x3ff00;
	} else {
		/* set broadcast address */
		sk_addr->dstaddr = J1939_NO_ADDR;
	}
	j1939_recv_ecu_flags(skb, data);

	if (j1939_recv_address_claim(skb))
		goto done;
	if (j1939_recv_promisc(skb))
		goto done;
	if (j1939_recv_transport(skb))
		/* this means the transport layer processed the message */
		goto done;
	j1939_recv(skb);
done:
	kfree_skb(skb);
}

int j1939_send_can(struct sk_buff *skb)
{
	int ret, dlc;
	canid_t canid;
	struct j1939_sk_buff_cb *sk_addr;
	struct can_frame *cf;

	ret = j1939_fixup_address_claim(skb);
	if (unlikely(ret))
		goto failed;
	dlc = skb->len;
	if (dlc > 8) {
		ret = -EMSGSIZE;
		goto failed;
	}

	/* re-claim the CAN_HDR from the SKB */
	cf = (void *)skb_push(skb, CAN_HDR);
	BUG_ON(!cf);
	/* make it a full can frame again */
	skb_put(skb, CAN_FTR + (8 - dlc));

	sk_addr = (struct j1939_sk_buff_cb *)skb->cb;
	canid = CAN_EFF_FLAG |
		(sk_addr->srcaddr & 0xff) |
		((sk_addr->priority & 0x7) << 26);
	if (pgn_is_pdu1(sk_addr->pgn))
		canid |= ((sk_addr->pgn & 0x3ff00) << 8) |
			((sk_addr->dstaddr & 0xff) << 8);
	else
		canid |= ((sk_addr->pgn & 0x3ffff) << 8);

	cf->can_id = canid;
	if (padding) {
		memset(cf->data + dlc, 0xff, 8 - dlc);
		cf->can_dlc = 8;
	} else
		cf->can_dlc = dlc;

	return can_send(skb, 1);
failed:
	consume_skb(skb);
	return ret;
}

int j1939_send(struct sk_buff *skb)
{
	struct j1939_sk_buff_cb *cb = (void *)skb->cb;
	struct j1939_priv *priv;
	struct j1939_ecu *ecu;
	int ret = 0;

	/* apply sanity checks */
	cb->pgn &= (pgn_is_pdu1(cb->pgn)) ? 0x3ff00 : 0x3ffff;
	if (cb->priority > 7)
		cb->priority = 6;

	/* verify source */
	priv = dev_j1939_priv(skb->dev);
	if (!priv)
		return -ENETUNREACH;

	read_lock_bh(&priv->lock);
	/* verify source */
	if (cb->srcname) {
		ecu = j1939_ecu_find_by_name(cb->srcname, skb->dev->ifindex);
		if (ecu) {
			cb->srcflags = ecu->flags;
			put_j1939_ecu(ecu);
		}
	} else if (j1939_address_is_unicast(cb->srcaddr))
		cb->srcflags = priv->ents[cb->srcaddr].flags;
	else if (cb->srcaddr == J1939_IDLE_ADDR)
		/* allow always */
		cb->srcflags = ECUFLAG_LOCAL;
	else
		cb->srcflags = 0;

	if (cb->srcflags & ECUFLAG_REMOTE) {
		ret = -EREMOTE;
		goto done;
	} else if (!(cb->srcflags & ECUFLAG_LOCAL)) {
		ret = -EADDRNOTAVAIL;
		goto done;
	}

	/* verify destination */
	if (cb->dstname) {
		ecu = j1939_ecu_find_by_name(cb->dstname, skb->dev->ifindex);
		if (!ecu) {
			ret = -EADDRNOTAVAIL;
			goto done;
		}
		cb->dstflags = ecu->flags;
		put_j1939_ecu(ecu);
	} else if (j1939_address_is_unicast(cb->dstaddr)) {
		cb->dstflags = priv->ents[cb->dstaddr].flags;
	} else if (cb->dstaddr == J1939_IDLE_ADDR) {
		/* not a valid destination */
		ret = -EADDRNOTAVAIL;
		goto done;
	} else
		cb->dstflags = 0;

	if (skb->len > 8)
		ret = j1939_send_transport(skb);
	else
		ret = j1939_send_can(skb);
done:
	read_unlock_bh(&priv->lock);
	put_j1939_priv(priv);
	return ret;
}
EXPORT_SYMBOL_GPL(j1939_send);

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
	priv->ifindex = netdev->ifindex;
	kref_init(&priv->kref);

	/* link into netdev */
	can_ml_priv = netdev->ml_priv;
	if (!can_ml_priv->j1939_priv)
		can_ml_priv->j1939_priv = priv;
	ret = (can_ml_priv->j1939_priv == priv) ? 0 : -EBUSY;
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
	can_ml_priv = netdev->ml_priv;
	priv = can_ml_priv->j1939_priv;
	can_ml_priv->j1939_priv = NULL;
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


static int j1939_netdev_notify(struct notifier_block *nb,
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
		j1939tp_rmdev_notifier(netdev);
		j1939sk_netdev_event(netdev->ifindex, ENODEV);
		break;

	case NETDEV_DOWN:
		j1939sk_netdev_event(netdev->ifindex, ENETDOWN);
		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block j1939_netdev_notifier = {
	.notifier_call = j1939_netdev_notify,
};

/* MODULE interface */

static __init int j1939_module_init(void)
{
	int ret;

	pr_info("can: SAE J1939\n");

	/* create /proc/net/can directory */
	j1939_procdir = proc_mkdir(j1939_procname, init_net.proc_net);
	if (!j1939_procdir)
		return -EINVAL;

	register_netdevice_notifier(&j1939_netdev_notifier);

	ret = can_proto_register(&j1939_can_proto);
	if (ret < 0) {
		pr_err("can: registration of j1939 protocol failed\n");
		goto fail_sk;
	}
	ret = j1939tp_module_init();
	if (ret < 0)
		goto fail_tp;
	return 0;

	j1939tp_module_exit();
fail_tp:
	can_proto_unregister(&j1939_can_proto);
fail_sk:
	unregister_netdevice_notifier(&j1939_netdev_notifier);
	proc_remove(j1939_procdir);
	j1939_procdir = NULL;
	return ret;
}

static __exit void j1939_module_exit(void)
{
	struct net_device *netdev;

	/* shutdown j1939 for all netdevs */
	rcu_read_lock();
	for_each_netdev_rcu(&init_net, netdev) {
		if (netdev->type != ARPHRD_CAN)
			continue;
		/* netdev disable only disables when j1939 active */
		netdev_disable_j1939(netdev);
	}
	rcu_read_unlock();

	j1939tp_module_exit();

	can_proto_unregister(&j1939_can_proto);

	unregister_netdevice_notifier(&j1939_netdev_notifier);

	proc_remove(j1939_procdir);
	j1939_procdir = NULL;
}

module_init(j1939_module_init);
module_exit(j1939_module_exit);
