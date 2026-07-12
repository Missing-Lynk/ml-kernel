// SPDX-License-Identifier: GPL-2.0
/*
 * ar_sysctl.ko - open reimplementation of /dev/ar_sysctl: a pure-software
 * priority / suspend-resume arbiter for the MPP sub-services. No hardware.
 *
 * ABI recovered byte-exact from the vendor .ko disassembly. magic 'S',
 * 7 ioctls nr 20..26, 40-byte msg {char name[32]; int priority(<32); int event}.
 * 32 priority slots (SYSCTL_MAX_PRIORITY, proven 3 ways). status:
 * 0=RUNNING 1=SUSPEND 2=BUSY 3=ERROR. The vendor runs a worker kthread that
 * walks the 32 slots dispatching the pending event; here suspend/resume flip
 * status directly (the dispatch ordering is observable only to the closed
 * services, which gate on status + the per-node event writeback).
 */
#define pr_fmt(fmt) "ar_sysctl: " fmt

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/string.h>

#define SYSCTL_MAX_PRIORITY	32

/* status word */
enum { ST_RUNNING = 0, ST_SUSPEND = 1, ST_BUSY = 2, ST_ERROR = 3 };
/* event codes: 3 = blocking suspend, 5 = EVENT_EXIT */
enum { EV_BLOCK_SUSPEND = 3, EV_EXIT = 5 };

struct sysctl_msg {		/* 40 bytes (0x28) */
	char	name[32];
	__s32	priority;	/* must be < 32 */
	__s32	event;
};

struct sysctl_node {
	struct sysctl_msg	msg;
	struct file		*owner;	/* for crash cleanup */
	bool			active;
	struct list_head	link;	/* in g_prio[priority] */
};

static DEFINE_MUTEX(g_lock);
static struct list_head g_prio[SYSCTL_MAX_PRIORITY];
static int g_status = ST_RUNNING;
static int g_active_cnt;

static struct sysctl_node *node_find(const char *name)
{
	int i;
	struct sysctl_node *n;

	for (i = 0; i < SYSCTL_MAX_PRIORITY; i++) {
		list_for_each_entry(n, &g_prio[i], link)
			if (!strncmp(n->msg.name, name, sizeof(n->msg.name)))
				return n;
	}
	return NULL;
}

static long sc_register(struct file *f, struct sysctl_msg *m)
{
	struct sysctl_node *n;

	if (m->priority < 0 || m->priority >= SYSCTL_MAX_PRIORITY)
		return -EINVAL;
	n = node_find(m->name);
	if (!n) {
		n = kzalloc(sizeof(*n), GFP_KERNEL);
		if (!n)
			return -ENOMEM;
		list_add_tail(&n->link, &g_prio[m->priority]);
	} else if (n->msg.priority != m->priority) {
		/* re-register with a new priority: move to the matching bucket so the
		 * node lives in g_prio[priority] (node_find scans all buckets, but the
		 * priority-ordered walk in suspend dispatch relies on correct bucketing).
		 */
		list_move_tail(&n->link, &g_prio[m->priority]);
	}
	n->msg = *m;
	n->owner = f;
	if (!n->active) {
		n->active = true;
		g_active_cnt++;
	}
	return 0;
}

static long sc_unregister(struct sysctl_msg *m)
{
	struct sysctl_node *n = node_find(m->name);

	if (!n)
		return -EINVAL;
	if (n->active) {
		n->active = false;
		g_active_cnt--;
	}
	return 0;
}

/* nr22 get-element and nr23 event_done both look up + write back. event==EXIT
 * unlinks.
 */
static long sc_lookup_writeback(unsigned int nr, struct sysctl_msg *m)
{
	struct sysctl_node *n = node_find(m->name);

	if (!n)
		return -EINVAL;
	if (nr == 23 && m->event == EV_EXIT) {
		if (n->active)
			g_active_cnt--;
		list_del(&n->link);
		kfree(n);
		return 0;	/* nothing to write back */
	}
	*m = n->msg;
	return 1;		/* signal caller to copy_to_user */
}

static long ar_sysctl_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	unsigned int nr = _IOC_NR(cmd);
	struct sysctl_msg m;
	s32 v;
	long ret = 0, wb;

	if (_IOC_TYPE(cmd) != 'S')
		return -EINVAL;

	mutex_lock(&g_lock);
	switch (nr) {
	case 20: {	/* REGISTER */
		if (copy_from_user(&m, uarg, sizeof(m))) {
			ret = -EFAULT;
			break;
		}
		ret = sc_register(f, &m);
	}
	break;
	case 21: {	/* UNREGISTER */
		if (copy_from_user(&m, uarg, sizeof(m))) {
			ret = -EFAULT;
			break;
		}
		ret = sc_unregister(&m);
	}
	break;
	case 22:	/* get-element (_IOWR) */
	case 23: {	/* EVENT_DONE (_IOWR) */
		if (copy_from_user(&m, uarg, sizeof(m))) {
			ret = -EFAULT;
			break;
		}
		wb = sc_lookup_writeback(nr, &m);
		if (wb == 1 && copy_to_user(uarg, &m, sizeof(m)))
			ret = -EFAULT;
		else if (wb < 0)
			ret = wb;
	}
	break;
	case 24: {	/* SUSPEND / fast_suspend */
		if (copy_from_user(&m, uarg, sizeof(m))) {
			ret = -EFAULT;
			break;
		}
		if (g_status != ST_RUNNING || g_active_cnt == 0) {
			ret = -EBUSY;
			break;
		}
		g_status = ST_SUSPEND;
		/* event==3 is the blocking variant; here it returns once suspended */
	}
	break;
	case 25: {	/* FAST_RESUME (int) */
		if (copy_from_user(&v, uarg, sizeof(v))) {
			ret = -EFAULT;
			break;
		}
		if (g_status != ST_SUSPEND) {
			ret = -EBUSY;
			break;
		}
		g_status = ST_RUNNING;
	}
	break;
	case 26: {	/* QUERY_STATUS (_IOWR int) */
		v = g_status;
		if (copy_to_user(uarg, &v, sizeof(v)))
			ret = -EFAULT;
	}
	break;
	default: {
		ret = -EINVAL;
	}
	break;
	}
	mutex_unlock(&g_lock);
	return ret;
}

/* crash cleanup: drop any nodes still owned by the closing fd. */
static int ar_sysctl_release(struct inode *ino, struct file *f)
{
	int i;
	struct sysctl_node *n, *t;

	mutex_lock(&g_lock);
	for (i = 0; i < SYSCTL_MAX_PRIORITY; i++) {
		list_for_each_entry_safe(n, t, &g_prio[i], link) {
			if (n->owner == f) {
				if (n->active)
					g_active_cnt--;
				list_del(&n->link);
				kfree(n);
			}
		}
	}
	mutex_unlock(&g_lock);
	return 0;
}

static const struct file_operations ar_sysctl_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= ar_sysctl_ioctl,
	.compat_ioctl	= compat_ptr_ioctl,
	.release	= ar_sysctl_release,
};

static struct miscdevice ar_sysctl_dev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "ar_sysctl",
	.fops	= &ar_sysctl_fops,
};

static int __init ar_sysctl_init(void)
{
	int i;

	BUILD_BUG_ON(sizeof(struct sysctl_msg) != 40);
	for (i = 0; i < SYSCTL_MAX_PRIORITY; i++)
		INIT_LIST_HEAD(&g_prio[i]);
	return misc_register(&ar_sysctl_dev);
}

static void __exit ar_sysctl_exit(void)
{
	int i;
	struct sysctl_node *n, *t;

	misc_deregister(&ar_sysctl_dev);
	for (i = 0; i < SYSCTL_MAX_PRIORITY; i++) {
		list_for_each_entry_safe(n, t, &g_prio[i], link) {
			list_del(&n->link);
			kfree(n);
		}
	}
}

module_init(ar_sysctl_init);
module_exit(ar_sysctl_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("missinglynk (open reimpl)");
MODULE_DESCRIPTION("Artosyn /dev/ar_sysctl: 32-slot priority/suspend arbiter (open)");
