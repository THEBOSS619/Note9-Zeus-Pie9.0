// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal global scheduler boost code adapted from Samsung eHMP/EMS
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 * Copyright (C) 2019 Diep Quynh Nguyen <remilia.1505@gmail.com>
 *
 */

#include <linux/sched.h>
#include "sched.h"

static struct plist_head global_boost_list = PLIST_HEAD_INIT(global_boost_list);

static DEFINE_SPINLOCK(global_boost_lock);

static int global_boost_value(void)
{
	if (plist_head_empty(&global_boost_list))
		return 0;

	return plist_last(&global_boost_list)->prio;
}

int global_boost(void)
{
	u64 now = ktime_to_us(ktime_get());

	/* booting boost duration = 40s */
	if (now < 40 * USEC_PER_SEC)
		return 1;

	return global_boost_value() > 0;
}

void global_boost_update_request(struct global_boost_request *req, u32 new_value)
{
	unsigned long flags;

	if (req->node.prio == new_value)
		return;

	spin_lock_irqsave(&global_boost_lock, flags);

	/*
	 * If the request already added to the list updates the value, remove
	 * the request from the list and add it again.
	 */
	if (req->active)
		plist_del(&req->node, &global_boost_list);
	else
		req->active = 1;

	plist_node_init(&req->node, new_value);
	plist_add(&req->node, &global_boost_list);

	spin_unlock_irqrestore(&global_boost_lock, flags);
}