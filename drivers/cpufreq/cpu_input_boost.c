// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018-2019 Sultan Alsawaf <sultan@kerneltoast.com>.
 * Copyright (C) 2019 Danny Lin <danny@kdrag0n.dev>.
 */

/*
Improved by THEBOSS619 
*/

#define pr_fmt(fmt) "cpu_input_boost: " fmt

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/fb.h>
#include <linux/input.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include "../../kernel/sched/sched.h"
#include <linux/kthread.h>
#include <linux/version.h>

#define ST_TA "top-app"
#define ST_FG "foreground"
#define ST_BG "background"
#define ST_ROOT "/"

unsigned long last_input_jiffies;

static __read_mostly unsigned int input_boost_freq_lp = CONFIG_INPUT_BOOST_FREQ_LP;
static __read_mostly unsigned int input_boost_freq_hp = CONFIG_INPUT_BOOST_FREQ_PERF;
static __read_mostly unsigned int input_boost_return_freq_lp = CONFIG_REMOVE_INPUT_BOOST_FREQ_LP;
static __read_mostly unsigned int input_boost_return_freq_hp = CONFIG_REMOVE_INPUT_BOOST_FREQ_PERF;
static __read_mostly unsigned int input_boost_awake_return_freq_lp = CONFIG_AWAKE_REMOVE_INPUT_BOOST_FREQ_LP;
static __read_mostly unsigned int input_boost_awake_return_freq_hp = CONFIG_AWAKE_REMOVE_INPUT_BOOST_FREQ_PERF;
static __read_mostly unsigned int general_boost_freq_lp = CONFIG_GENERAL_BOOST_FREQ_LP;
static __read_mostly unsigned int general_boost_freq_hp = CONFIG_GENERAL_BOOST_FREQ_PERF;
static __read_mostly unsigned short input_boost_duration = CONFIG_INPUT_BOOST_DURATION_MS;
static __read_mostly unsigned short wake_boost_duration = CONFIG_WAKE_BOOST_DURATION_MS;

module_param(input_boost_freq_lp, uint, 0644);
module_param(input_boost_freq_hp, uint, 0644);
module_param_named(remove_input_boost_freq_lp, input_boost_return_freq_lp, uint, 0644);
module_param_named(remove_input_boost_freq_perf, input_boost_return_freq_hp, uint, 0644);
module_param(input_boost_awake_return_freq_lp, uint, 0644);
module_param(input_boost_awake_return_freq_hp, uint, 0644);
module_param(general_boost_freq_lp, uint, 0644);
module_param(general_boost_freq_hp, uint, 0644);
module_param(input_boost_duration, short, 0644);
module_param(wake_boost_duration, short, 0644);

#ifdef CONFIG_DYNAMIC_STUNE_BOOST
static __read_mostly int input_stune_boost = CONFIG_INPUT_STUNE_BOOST;
static __read_mostly int max_stune_boost = CONFIG_MAX_STUNE_BOOST;
static __read_mostly int general_stune_boost = CONFIG_GENERAL_STUNE_BOOST;
static __read_mostly int display_stune_boost = CONFIG_DISPLAY_BOOST_STUNE_LEVEL;
static __read_mostly int display_bg_stune_boost = CONFIG_BG_DISPLAY_BOOST_STUNE_LEVEL;
static __read_mostly int suspend_ta_stune_boost = CONFIG_SUSPEND_BOOST_STUNE_LEVEL;
static __read_mostly int suspend_fg_stune_boost = CONFIG_SUSPEND_BOOST_STUNE_LEVEL;
static __read_mostly int suspend_bg_stune_boost = CONFIG_SUSPEND_BOOST_STUNE_LEVEL;
static __read_mostly int suspend_root_stune_boost = CONFIG_ROOT_SUSPEND_BOOST_STUNE_LEVEL;

module_param_named(dynamic_stune_boost, input_stune_boost, int, 0644);
module_param(max_stune_boost, int, 0644);
module_param(general_stune_boost, int, 0644);
module_param(display_stune_boost, int, 0644);
module_param(display_bg_stune_boost, int, 0644);
module_param(suspend_ta_stune_boost, int, 0644);
module_param(suspend_fg_stune_boost, int, 0644);
module_param(suspend_bg_stune_boost, int, 0644);
module_param(suspend_root_stune_boost, int, 0644);
#endif

/* The sched_param struct is located elsewhere in newer kernels */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
#include <uapi/linux/sched/types.h>
#endif

/* Available bits for boost_drv state */
enum {
	SCREEN_AWAKE,
	INPUT_BOOST,
	MAX_BOOST,
	GENERAL_BOOST
};

struct boost_drv {
	struct kthread_worker worker;
	struct task_struct *worker_thread;
	struct kthread_work input_boost;
	struct delayed_work input_unboost;
	struct kthread_work max_boost;
	struct delayed_work max_unboost;
	struct kthread_work general_boost;
	struct delayed_work general_unboost;
	struct notifier_block cpu_notif;
	struct notifier_block fb_notif;
	wait_queue_head_t boost_waitq;
	atomic_long_t max_boost_expires;
	atomic_t max_boost_dur;
	atomic64_t general_boost_expires;
	atomic_t general_boost_dur;
	unsigned long state;

	bool input_stune_active;
	int input_stune_slot;
	bool max_stune_active;
	int max_stune_slot;
	bool general_stune_active;
	int general_stune_slot;
	bool display_stune_active;
	int display_stune_slot;
	bool display_bg_stune_active;
	int display_bg_stune_slot;
	int ta_stune_boost_default;
	int fg_stune_boost_default;
	int bg_stune_boost_default;
	int root_stune_boost_default;
	bool bg_stune_default_set;
	unsigned long last_input_jiffies;
};

static void input_unboost_worker(struct work_struct *work);
static void max_unboost_worker(struct work_struct *work);

static struct boost_drv boost_drv_g __read_mostly = {
	.input_unboost = __DELAYED_WORK_INITIALIZER(boost_drv_g.input_unboost,
						    input_unboost_worker, 0),
	.max_unboost = __DELAYED_WORK_INITIALIZER(boost_drv_g.max_unboost,
						  max_unboost_worker, 0),
	.boost_waitq = __WAIT_QUEUE_HEAD_INITIALIZER(boost_drv_g.boost_waitq)
};

static u32 get_boost_freq(struct boost_drv *b, u32 cpu, u32 state)
{
	if (state & INPUT_BOOST) {
		if (cpumask_test_cpu(cpu, cpu_lp_mask))
			return input_boost_freq_lp;

		if (cpu_rq(cpu)->nr_running > 1)
			return input_boost_freq_hp;

		return 0;
	}

	if (cpumask_test_cpu(cpu, cpu_lp_mask))
		return general_boost_freq_lp;

	if (cpu_rq(cpu)->nr_running > 1)
		return general_boost_freq_hp;

	return 0;
}

static u32 get_min_freq(struct boost_drv *b, u32 cpu, u32 state)
{
	if (state & SCREEN_AWAKE) {
		if (cpumask_test_cpu(cpu, cpu_lp_mask))
			return input_boost_awake_return_freq_lp;

		return input_boost_awake_return_freq_hp;
	}

	if (cpumask_test_cpu(cpu, cpu_lp_mask))
		return input_boost_return_freq_lp;

	return input_boost_return_freq_hp;
}

static u32 get_boost_state(struct boost_drv *b)
{
	return atomic_read(&b->state);
}

static void set_boost_bit(struct boost_drv *b, u32 state)
{
	atomic_or(state, &b->state);
}

static void clear_boost_bit(struct boost_drv *b, u32 state)
{
	atomic_andnot(state, &b->state);
}

static void update_online_cpu_policy(void)
{
	unsigned int cpu;

	/* Only one CPU from each cluster needs to be updated */
	get_online_cpus();
	cpu = cpumask_first_and(cpu_lp_mask, cpu_online_mask);
	cpufreq_update_policy(cpu);
	cpu = cpumask_first_and(cpu_perf_mask, cpu_online_mask);
	cpufreq_update_policy(cpu);
	put_online_cpus();
}

static void update_stune_boost(struct boost_drv *b, bool *active, char *st,
			    int value, int *slot)
{
	if (value && !*active)
		*active = !do_stune_boost(st, value, slot);
}

static void clear_stune_boost(struct boost_drv *b, bool *active, char *st,
			      int slot)
{
	if (*active)
		*active = reset_stune_boost(st, slot);
}

bool cpu_input_boost_within_input(unsigned long timeout_ms)
{
	struct boost_drv *b = &boost_drv_g;

	return time_before(jiffies, b->last_input_jiffies +
			   msecs_to_jiffies(timeout_ms));
}

static void unboost_all_cpus(struct boost_drv *b)
{
	if (!cancel_delayed_work_sync(&b->input_unboost) &&
		!cancel_delayed_work_sync(&b->general_unboost) &&
		!cancel_delayed_work_sync(&b->max_unboost))
		return;

	clear_boost_bit(b, INPUT_BOOST | MAX_BOOST | GENERAL_BOOST);
	update_online_cpu_policy();

	clear_stune_boost(b, &b->input_stune_active, ST_TA, b->input_stune_slot);
	clear_stune_boost(b, &b->max_stune_active, ST_TA, b->max_stune_slot);
	clear_stune_boost(b, &b->general_stune_active, ST_TA, b->general_stune_slot);
}

static void __cpu_input_boost_kick(struct boost_drv *b)
{
	if (test_bit(SCREEN_AWAKE, &b->state))
		return;

	set_bit(INPUT_BOOST, &b->state);
	if (!mod_delayed_work(system_unbound_wq, &b->input_unboost,
			      msecs_to_jiffies(input_boost_duration)))
		wake_up(&b->boost_waitq);
}

void cpu_input_boost_kick(void)
{
	struct boost_drv *b = &boost_drv_g;

	__cpu_input_boost_kick(b);
}

static void __cpu_input_boost_kick_max(struct boost_drv *b,
				       unsigned int duration_ms)
{
	unsigned long boost_jiffies = msecs_to_jiffies(duration_ms);
	unsigned long curr_expires, new_expires;

	do {
		curr_expires = atomic_long_read(&b->max_boost_expires);
		new_expires = jiffies + boost_jiffies;

		/* Skip this boost if there's a longer boost in effect */
		if (time_after(curr_expires, new_expires))
			return;
	} while (atomic_long_cmpxchg(&b->max_boost_expires, curr_expires,
				     new_expires) != curr_expires);

	set_bit(MAX_BOOST, &b->state);
	if (!mod_delayed_work(system_unbound_wq, &b->max_unboost,
			      boost_jiffies))
		wake_up(&b->boost_waitq);
}

void cpu_input_boost_kick_max(unsigned int duration_ms)
{
	struct boost_drv *b = &boost_drv_g;

	if (test_bit(SCREEN_AWAKE, &b->state))
		return;

	energy_aware_enable = false;
	__cpu_input_boost_kick_max(b, duration_ms);
}

void cpu_input_boost_kick_wake(void)
{
	struct boost_drv *b = &boost_drv_g;

	__cpu_input_boost_kick_max(b, wake_boost_duration);
}

static void __cpu_input_boost_kick_general(struct boost_drv *b,
	unsigned int duration_ms)
{
	unsigned long curr_expires, new_expires;

	do {
		curr_expires = atomic64_read(&b->general_boost_expires);
		new_expires = jiffies + msecs_to_jiffies(duration_ms);

		/* Skip this boost if there's a longer boost in effect */
		if (time_after(curr_expires, new_expires))
			return;
	} while (atomic64_cmpxchg(&b->general_boost_expires, curr_expires,
		new_expires) != curr_expires);

	atomic_set(&b->general_boost_dur, duration_ms);
	kthread_queue_work(&b->worker, &b->general_boost);
}

void cpu_input_boost_kick_general(unsigned int duration_ms)
{
	struct boost_drv *b = boost_drv_g;
	u32 state;

	if (!b)
		return;

	state = get_boost_state(b);

	if (!(state & SCREEN_AWAKE))
		return;

	__cpu_input_boost_kick_general(b, duration_ms);
}

static void input_boost_worker(struct kthread_work *work)
{
	struct boost_drv *b = container_of(work, typeof(*b), input_boost);

	if (!cancel_delayed_work_sync(&b->input_unboost)) {
		set_boost_bit(b, INPUT_BOOST);
		update_online_cpu_policy();

		update_stune_boost(b, &b->input_stune_active, ST_TA, input_stune_boost,
				   &b->input_stune_slot);
	}

	queue_delayed_work(system_power_efficient_wq, &b->input_unboost,
		msecs_to_jiffies(input_boost_duration));
}

static void input_unboost_worker(struct work_struct *work)
{
	struct boost_drv *b = container_of(to_delayed_work(work),
					   typeof(*b), input_unboost);

	clear_boost_bit(b, INPUT_BOOST);
	update_online_cpu_policy();

	clear_stune_boost(b, &b->input_stune_active, ST_TA, b->input_stune_slot);
}

static void max_boost_worker(struct kthread_work *work)
{
	struct boost_drv *b = container_of(work, typeof(*b), max_boost);

	if (!cancel_delayed_work_sync(&b->max_unboost)) {
		set_boost_bit(b, MAX_BOOST);
		update_online_cpu_policy();

		update_stune_boost(b, &b->max_stune_active, ST_TA, max_stune_boost,
				   &b->max_stune_slot);
	}

	queue_delayed_work(system_power_efficient_wq, &b->max_unboost,
		msecs_to_jiffies(atomic_read(&b->max_boost_dur)));

}

static void max_unboost_worker(struct work_struct *work)
{
	struct boost_drv *b = container_of(to_delayed_work(work),
					   typeof(*b), max_unboost);

	energy_aware_enable = true;
	clear_boost_bit(b, MAX_BOOST);
	wake_up(&b->boost_waitq);

	clear_stune_boost(b, &b->max_stune_active, ST_TA, b->max_stune_slot);
}

static int cpu_thread(void *data)
{
	static const struct sched_param sched_max_rt_prio = {
		.sched_priority = MAX_RT_PRIO - 1
	};
	struct boost_drv *b = data;
	unsigned long old_state = 0;

	sched_setscheduler_nocheck(current, SCHED_FIFO, &sched_max_rt_prio);

	while (1) {
		bool should_stop = false;
		unsigned long curr_state;

		wait_event(b->boost_waitq,
			(curr_state = READ_ONCE(b->state)) != old_state ||
			(should_stop = kthread_should_stop()));

		if (should_stop)
			break;

		old_state = curr_state;
		update_online_cpu_policy();
	}
	return 0;
}

static void general_boost_worker(struct kthread_work *work)
{
	struct boost_drv *b = container_of(work, typeof(*b), general_boost);

	if (!cancel_delayed_work_sync(&b->general_unboost)) {
		set_boost_bit(b, GENERAL_BOOST);
		update_online_cpu_policy();

		update_stune_boost(b, &b->general_stune_active, ST_TA,
				   general_stune_boost, &b->general_stune_slot);
	}

	queue_delayed_work(system_power_efficient_wq, &b->general_unboost,
		msecs_to_jiffies(atomic_read(&b->general_boost_dur)));

}

static void general_unboost_worker(struct work_struct *work)
{
	struct boost_drv *b = container_of(to_delayed_work(work),
					   typeof(*b), general_unboost);

	clear_boost_bit(b, GENERAL_BOOST);
	update_online_cpu_policy();

	clear_stune_boost(b, &b->general_stune_active, ST_TA, b->general_stune_slot);
}

static int cpu_notifier_cb(struct notifier_block *nb, unsigned long action,
			   void *data)
{
	struct boost_drv *b = container_of(nb, typeof(*b), cpu_notif);
	struct cpufreq_policy *policy = data;
	u32 boost_freq, min_freq, state;

	if (action != CPUFREQ_ADJUST)
		return NOTIFY_OK;

	/* Unboost when the screen is off */
	if (test_bit(SCREEN_AWAKE, &b->state)) {
		policy->min = policy->cpuinfo.min_freq;
		clear_stune_boost(b, &b->input_stune_active, ST_TA, b->input_stune_slot);
		clear_stune_boost(b, &b->max_stune_active, ST_TA, b->max_stune_slot);
		clear_stune_boost(b, &b->general_stune_active, ST_TA, b->general_stune_slot);
		return NOTIFY_OK;
	}

	/* Boost CPU to max frequency for max boost */
	if (test_bit(MAX_BOOST, &b->state)) {
		policy->min = policy->max;
		update_stune_boost(b, &b->input_stune_active, ST_TA, input_stune_boost,
				   &b->input_stune_slot);
		update_stune_boost(b, &b->max_stune_active, ST_TA, max_stune_boost,
				   &b->max_stune_slot);
		update_stune_boost(b, &b->general_stune_active, ST_TA,
				   general_stune_boost, &b->general_stune_slot);
		return NOTIFY_OK;
	}

	/* CPU boost is disabled. Don't apply boost */
	boost_freq = get_boost_freq(b, policy->cpu, state);
	if (boost_freq == 0)
		return NOTIFY_OK;

	/*
	 * Boost to policy->max if the boost frequency is higher. When
	 * unboosting, set policy->min to the absolute min freq for the CPU.
	 */
	if (test_bit(INPUT_BOOST, GENERAL_BOOST, &b->state)) {
		policy->min = min(policy->max, boost_freq);
		update_stune_boost(b, &b->input_stune_active, ST_TA, input_stune_boost,
				   &b->input_stune_slot);
		update_stune_boost(b, &b->max_stune_active, ST_TA, max_stune_boost,
				   &b->max_stune_slot);
		update_stune_boost(b, &b->general_stune_active, ST_TA,
				   general_stune_boost, &b->general_stune_slot);
	} else {
		min_freq = get_min_freq(b, policy->cpu, state);
		policy->min = max(policy->cpuinfo.min_freq, min_freq);
		clear_stune_boost(b, &b->input_stune_active, ST_TA, b->input_stune_slot);
		clear_stune_boost(b, &b->max_stune_active, ST_TA, b->max_stune_slot);
		clear_stune_boost(b, &b->general_stune_active, ST_TA, b->general_stune_slot);
	}

	return NOTIFY_OK;
}

static int fb_notifier_cb(struct notifier_block *nb,
			  unsigned long action, void *data)
{
	struct boost_drv *b = container_of(nb, typeof(*b), fb_notif);
	struct fb_event *evdata = data;
	int *blank = ((struct fb_event *)data)->data;

	/* Parse framebuffer blank events as soon as they occur */
	if (action != FB_EARLY_EVENT_BLANK)
		return NOTIFY_OK;

	/* Boost when the screen turns on and unboost when it turns off */
	if (*blank == FB_BLANK_UNBLANK) {
		if (b->ta_stune_boost_default != INT_MIN)
			set_stune_boost(ST_TA, b->ta_stune_boost_default, NULL);
		if (b->fg_stune_boost_default != INT_MIN)
			set_stune_boost(ST_FG, b->fg_stune_boost_default, NULL);
		if (!b->bg_stune_default_set) {
			set_stune_boost(ST_BG, suspend_bg_stune_boost, NULL);
			b->bg_stune_default_set = true;
		clear_bit(SCREEN_AWAKE, &b->state);
		}
		if (b->root_stune_boost_default != INT_MIN)
			set_stune_boost(ST_ROOT, b->root_stune_boost_default, NULL);

		update_stune_boost(b, &b->display_stune_active, ST_TA,
			           display_stune_boost, &b->display_stune_slot);
		update_stune_boost(b, &b->display_bg_stune_active, ST_BG,
			           display_bg_stune_boost, &b->display_bg_stune_slot);
		__cpu_input_boost_kick_max(b, wake_boost_duration);
		disable_schedtune_boost(0);
#ifdef CONFIG_CPU_INPUT_BOOST_DEBUG
		pr_info("kicked max wake boost due to unblank event\n");
#endif
	} else {
		disable_schedtune_boost(1);
		clear_stune_boost(b, &b->display_stune_active, ST_TA,
				  b->display_stune_slot);
		clear_stune_boost(b, &b->display_bg_stune_active, ST_BG,
				  b->display_bg_stune_slot);
		unboost_all_cpus(b);

		set_stune_boost(ST_TA, suspend_ta_stune_boost,
				&b->ta_stune_boost_default);
		set_stune_boost(ST_FG, suspend_fg_stune_boost,
				&b->fg_stune_boost_default);
		set_stune_boost(ST_ROOT, suspend_root_stune_boost,
				&b->root_stune_boost_default);
		set_bit(SCREEN_AWAKE, &b->state);
#ifdef CONFIG_CPU_INPUT_BOOST_DEBUG
		pr_info("cleared all boosts due to blank event\n");
#endif
	}

	return NOTIFY_OK;
}

static void cpu_input_boost_input_event(struct input_handle *handle,
					unsigned int type, unsigned int code,
					int value)
{
	struct boost_drv *b = handle->handler->private;

	__cpu_input_boost_kick(b);

	if (type == EV_KEY && code == KEY_POWER && value == 1 &&
	    !test_bit(SCREEN_AWAKE, &b->state))
		__cpu_input_boost_kick_max(b, wake_boost_duration);

	last_input_jiffies = jiffies;
	b->last_input_jiffies = jiffies;
}

static int cpu_input_boost_input_connect(struct input_handler *handler,
					 struct input_dev *dev,
					 const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpu_input_boost_handle";

	ret = input_register_handle(handle);
	if (ret)
		goto free_handle;

	ret = input_open_device(handle);
	if (ret)
		goto unregister_handle;

	return 0;

unregister_handle:
	input_unregister_handle(handle);
free_handle:
	kfree(handle);
	return ret;
}

static void cpu_input_boost_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id cpu_input_boost_ids[] = {
	/* Multi-touch touchscreen */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
			BIT_MASK(ABS_MT_POSITION_X) |
			BIT_MASK(ABS_MT_POSITION_Y) }
	},
	/* Touchpad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_KEYBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
		.absbit = { [BIT_WORD(ABS_X)] =
			BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) }
	},
	/* Keypad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) }
	},
	{ }
};

static struct input_handler cpu_input_boost_input_handler = {
	.event		= cpu_input_boost_input_event,
	.connect	= cpu_input_boost_input_connect,
	.disconnect	= cpu_input_boost_input_disconnect,
	.name		= "cpu_input_boost_handler",
	.id_table	= cpu_input_boost_ids
};

static int __init cpu_input_boost_init(void)
{
	struct boost_drv *b = &boost_drv_g;
	struct task_struct *thread;
	int ret, i;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 2 };
	cpumask_t sys_bg_mask;

	kthread_init_worker(&b->worker);
	b->worker_thread = kthread_run(kthread_worker_fn, &b->worker,
				       "cpu_input_boost_thread");
	if (IS_ERR(b->worker_thread)) {
		ret = PTR_ERR(b->worker_thread);
		pr_err("Failed to start kworker, err: %d\n", ret);
		goto free_b;
	}

	ret = sched_setscheduler(b->worker_thread, SCHED_FIFO, &param);
	if (ret)
		pr_err("Failed to set SCHED_FIFO on kworker, err: %d\n", ret);

	/* Init the cpumask: 1-3 inclusive */
	for (i = 1; i <= 3; i++)
		cpumask_set_cpu(i, &sys_bg_mask);

	/* Bind it to the cpumask */
	kthread_bind_mask(b->worker_thread, &sys_bg_mask);

	/* Wake it up */
	wake_up_process(b->worker_thread);

	atomic64_set(&b->max_boost_expires, 0);
	kthread_init_work(&b->input_boost, input_boost_worker);
	kthread_init_work(&b->max_boost, max_boost_worker);
	kthread_init_work(&b->general_boost, general_boost_worker);
	INIT_DELAYED_WORK(&b->general_unboost, general_unboost_worker);
	b->ta_stune_boost_default = INT_MIN;
	b->fg_stune_boost_default = INT_MIN;
	b->bg_stune_boost_default = INT_MIN;
	b->root_stune_boost_default = INT_MIN;
	b->bg_stune_default_set = false;

	b->cpu_notif.notifier_call = cpu_notifier_cb;
	b->cpu_notif.priority = INT_MAX - 2;
	ret = cpufreq_register_notifier(&b->cpu_notif, CPUFREQ_POLICY_NOTIFIER);
	if (ret) {
		pr_err("Failed to register cpufreq notifier, err: %d\n", ret);
		return ret;
	}

	cpu_input_boost_input_handler.private = b;
	ret = input_register_handler(&cpu_input_boost_input_handler);
	if (ret) {
		pr_err("Failed to register input handler, err: %d\n", ret);
		goto unregister_cpu_notif;
	}

	b->fb_notif.notifier_call = fb_notifier_cb;
	b->fb_notif.priority = INT_MAX;
	ret = fb_register_client(&b->fb_notif);
	if (ret) {
		pr_err("Failed to register fb notifier, err: %d\n", ret);
		goto unregister_handler;
	}

	thread = kthread_run_perf_critical(cpu_thread, b, "cpu_boostd");
	if (IS_ERR(thread)) {
		ret = PTR_ERR(thread);
		pr_err("Failed to start CPU boost thread, err: %d\n", ret);
		goto free_b;
	}

	return 0;

unregister_handler:
	input_unregister_handler(&cpu_input_boost_input_handler);
unregister_cpu_notif:
	cpufreq_unregister_notifier(&b->cpu_notif, CPUFREQ_POLICY_NOTIFIER);
free_b:
	kfree(b);
	return ret;
}
subsys_initcall(cpu_input_boost_init);
