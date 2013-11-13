/*
 * Copyright (c) 2010-2011 EIA Electronics
 *
 * Authors:
 * Kurt Van Dijck <kurt.van.dijck@eia.be>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the version 2 of the GNU General Public License
 * as published by the Free Software Foundation
 */

/*
 * j1939-bus.c - bus for j1939 remote devices
 * Since rtnetlink, no real bus is used.
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/workqueue.h>

#include "j1939-priv.h"

#define ecu_dbg(ecu, fmt, ...) \
	pr_debug("j1939-%i,%016llx,%02x: " fmt, (ecu)->parent->ifindex, \
		(ecu)->name, (ecu)->sa, ##__VA_ARGS__)
#define ecu_alert(ecu, fmt, ...) \
	pr_alert("j1939-%i,%016llx,%02x: " fmt, (ecu)->parent->ifindex, \
		(ecu)->name, (ecu)->sa, ##__VA_ARGS__)

/*
 * iterate over ECU's,
 * and register flagged ecu's on their claimed SA
 */
void j1939_priv_ac_task(unsigned long val)
{
	struct j1939_priv *priv = (void *)val;
	struct j1939_ecu *ecu;

	write_lock_bh(&priv->lock);
	list_for_each_entry(ecu, &priv->ecus, list) {
		/* next 2 (read & set) could be merged into xxx? */
		if (!atomic_read(&ecu->ac_delay_expired))
			continue;
		atomic_set(&ecu->ac_delay_expired, 0);
		if (j1939_address_is_unicast(ecu->sa))
			ecu->parent->ents[ecu->sa].ecu = ecu;
	}
	write_unlock_bh(&priv->lock);
}
/*
 * device interface
 */
static void cb_put_j1939_priv(struct kref *kref)
{
	struct j1939_priv *priv =
		container_of(kref, struct j1939_priv, kref);

	tasklet_disable_nosync(&priv->ac_task);
	kfree(priv);
}

void put_j1939_priv(struct j1939_priv *segment)
{
	kref_put(&segment->kref, cb_put_j1939_priv);
}

/*
 * ECU device interface
 */
static enum hrtimer_restart j1939_ecu_timer_handler(struct hrtimer *hrtimer)
{
	struct j1939_ecu *ecu =
		container_of(hrtimer, struct j1939_ecu, ac_timer);

	atomic_set(&ecu->ac_delay_expired, 1);
	tasklet_schedule(&ecu->parent->ac_task);
	return HRTIMER_NORESTART;
}

static void cb_put_j1939_ecu(struct kref *kref)
{
	struct j1939_ecu *ecu =container_of(kref, struct j1939_ecu, kref);

	kfree(ecu);
}
void put_j1939_ecu(struct j1939_ecu *ecu)
{
	kref_put(&ecu->kref, cb_put_j1939_ecu);
}

struct j1939_ecu *j1939_ecu_get_register(name_t name, int ifindex, int flags,
		int return_existing)
{
	struct j1939_priv *parent;
	struct j1939_ecu *ecu, *dut;

	if (!ifindex || !name) {
		pr_alert("%s(%i, %016llx) invalid\n",
				__func__, ifindex, (long long)name);
		return ERR_PTR(-EINVAL);
	}

	parent = j1939_priv_find(ifindex);
	if (!parent) {
		pr_alert("%s %i: segment not found\n", __func__, ifindex);
		return ERR_PTR(-EINVAL);
	}
	if (return_existing) {
		read_lock_bh(&parent->lock);
		/* test for existing name */
		list_for_each_entry(dut, &parent->ecus, list) {
			if (dut->name == name) {
				get_j1939_ecu(dut);
				read_unlock_bh(&parent->lock);
				return dut;
			}
		}
		read_unlock_bh(&parent->lock);
	}
	/* alloc */
	ecu = kzalloc(sizeof(*ecu), gfp_any());
	if (!ecu)
		/* should we look for an existing ecu */
		return ERR_PTR(-ENOMEM);
	kref_init(&ecu->kref);
	ecu->sa = J1939_IDLE_ADDR;
	ecu->name = name;
	ecu->flags = flags;

	hrtimer_init(&ecu->ac_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ecu->ac_timer.function = j1939_ecu_timer_handler;
	INIT_LIST_HEAD(&ecu->list);

	/* first add to internal list */
	write_lock_bh(&parent->lock);
	/* test for duplicate name */
	list_for_each_entry(dut, &parent->ecus, list) {
		if (dut->name == ecu->name)
			goto duplicate;
	}
	get_j1939_ecu(ecu);
	/* a ref to parent is held */
	ecu->parent = parent;
	list_add_tail(&ecu->list, &parent->ecus);
	write_unlock_bh(&parent->lock);
	ecu_dbg(ecu, "register\n");
	return ecu;

duplicate:
	get_j1939_ecu(dut);
	write_unlock_bh(&parent->lock);
	put_j1939_priv(parent);
	if (return_existing)
		return dut;
	ecu_alert(ecu, "duplicate name\n");
	put_j1939_ecu(ecu);
	return ERR_PTR(-EEXIST);
}

void j1939_ecu_unregister(struct j1939_ecu *ecu)
{
	BUG_ON(!ecu);
	ecu_dbg(ecu, "unregister\n");
	hrtimer_try_to_cancel(&ecu->ac_timer);

	write_lock_bh(&ecu->parent->lock);
	j1939_ecu_remove_sa_locked(ecu);
	list_del_init(&ecu->list);
	write_unlock_bh(&ecu->parent->lock);
	/* put segment, reverting the effect done by ..._register() */
	put_j1939_priv(ecu->parent);
	put_j1939_ecu(ecu);
}

struct j1939_ecu *j1939_ecu_find_by_addr(int sa, int ifindex)
{
	struct j1939_ecu *ecu;
	struct j1939_priv *parent;

	if (!j1939_address_is_unicast(sa))
		return NULL;
	parent = j1939_priv_find(ifindex);
	if (!parent)
		return NULL;
	read_lock_bh(&parent->lock);
	ecu = parent->ents[sa].ecu;
	if (ecu)
		get_j1939_ecu(ecu);
	read_unlock_bh(&parent->lock);
	put_j1939_priv(parent);
	return ecu;
}

int j1939_name_to_sa(uint64_t name, int ifindex)
{
	struct j1939_ecu *ecu;
	struct j1939_priv *parent;
	int sa;

	if (!name)
		return J1939_IDLE_ADDR;
	parent = j1939_priv_find(ifindex);
	if (!parent)
		return J1939_IDLE_ADDR;

	sa = J1939_IDLE_ADDR;
	read_lock_bh(&parent->lock);
	list_for_each_entry(ecu, &parent->ecus, list) {
		if (ecu->name == name) {
			if ((sa == J1939_IDLE_ADDR) &&
			    (parent->ents[ecu->sa].ecu == ecu))
				/* ecu's SA is registered */
				sa = ecu->sa;
			break;
		}
	}
	read_unlock_bh(&parent->lock);
	put_j1939_priv(parent);
	return sa;
}

struct j1939_ecu *j1939_ecu_find_priv_default_tx(int ifindex,
		name_t *name, uint8_t *addr)
{
	struct j1939_ecu *ecu;
	struct j1939_priv *parent;
	struct addr_ent *paddr;
	int j;

	if (ifindex <= 0)
		return ERR_PTR(-EINVAL);
	parent = j1939_priv_find(ifindex);
	if (!parent)
		return ERR_PTR(-ENETUNREACH);
	read_lock_bh(&parent->lock);
	list_for_each_entry(ecu, &parent->ecus, list) {
		if (ecu->flags & ECUFLAG_LOCAL) {
			get_j1939_ecu(ecu);
			if (name)
				*name = ecu->name;
			if (addr)
				*addr = ecu->sa;
			goto found;
		}
	}
	ecu = NULL;
	for (j = 0, paddr = parent->ents; j < J1939_IDLE_ADDR; ++j, ++paddr) {
		if (paddr->ecu)
			continue;
		if (paddr->flags & ECUFLAG_LOCAL) {
			if (name)
				*name = 0;
			if (addr)
				*addr = j;
			goto found;
		}
	}
	ecu = ERR_PTR(-EHOSTDOWN);
found:
	read_unlock_bh(&parent->lock);
	put_j1939_priv(parent);
	return ecu;
}

/* ecu lookup helper */
static struct j1939_ecu *_j1939_ecu_find_by_name(name_t name,
		struct j1939_priv *priv)
{
	struct j1939_ecu *ecu;

	read_lock_bh(&priv->lock);
	list_for_each_entry(ecu, &priv->ecus, list) {
		if (ecu->name == name) {
			get_j1939_ecu(ecu);
			goto found_on_intf;
		}
	}
	ecu = NULL;
found_on_intf:
	read_unlock_bh(&priv->lock);
	return ecu;
}

/* ecu lookup by name */
struct j1939_ecu *j1939_ecu_find_by_name(name_t name, int ifindex)
{
	struct j1939_ecu *ecu;
	struct j1939_priv *priv;
	struct net_device *netdev;

	if (!name)
		return NULL;
	if (ifindex) {
		priv = j1939_priv_find(ifindex);
		if (!priv)
			return NULL;
		ecu = _j1939_ecu_find_by_name(name, priv);
		put_j1939_priv(priv);
		return ecu;
	}

	/* iterate netdevices */
	for_each_netdev(&init_net, netdev) {
		priv = dev_j1939_priv(netdev);
		if (!priv)
			continue;
		ecu = _j1939_ecu_find_by_name(name, priv);
		put_j1939_priv(priv);
		if (ecu)
			goto found;
	}
	ecu = NULL;
found:
	return ecu;
}
