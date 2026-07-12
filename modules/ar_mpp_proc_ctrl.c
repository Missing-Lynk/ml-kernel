// SPDX-License-Identifier: GPL-2.0
/*
 * ar_mpp_proc_ctrl.ko - open reimplementation of /dev/ar_mpp_proc_ctl.
 *
 * The MPP /proc "umap" shuttle: a userspace daemon CREATEs a
 * named /proc/umap/<name> entry; reads of that proc file are answered with text
 * the daemon supplies, and writes to it are queued back to the daemon. The daemon
 * blocks in MSG_FROM_PROC_DEV to receive those, and pushes show-text via WRITE.
 *
 * ABI (recovered byte-exact from the vendor .ko disassembly): magic 'M', nr 6..9, sizes:
 *   6:IOWR 144 CREATE   7:IOR 280 MSG_FROM_PROC_DEV   8:IOR 24 WRITE   9:IOW 8 CLOSE
 *
 * The nr8/nr9 bindings are proven by the vendor .ko debug strings: the nr8
 * (24B) handler prints "==> AR_PROC_WRITE_MSG_TO_PROC_FS" and the nr9 (8B)
 * handler prints "==> AR_PROC_REQUEST_CLOSE".
 *
 * The 144/280/24-byte struct *field* layouts below are now byte-pinned from the
 * kernel copy_{from,to}_user offsets and confirmed against the userspace callers
 * (sources cited per struct). This is a debug/log shuttle, not the video data
 * path; correctness-of-shape matters more than perf.
 */
#define pr_fmt(fmt) "ar_mpp_proc_ctrl: " fmt

#include <linux/module.h>
#include <linux/build_bug.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/list.h>

#define MPP_MAGIC	'M'
#define PROC_DIR	"umap"
#define NAME_LEN	128		/* CREATE name[] field width */
#define PAYLOAD_LEN	256		/* MSG data[] payload field width */
#define SHOW_LEN	4096

struct proc_dev {
	u64			handle;	/* opaque entry handle (kernel ptr in vendor) */
	u32			key;	/* CREATE.key cookie (re: create_arg+128) */
	char			name[NAME_LEN];
	struct proc_dir_entry	*pde;
	char			show_buf[SHOW_LEN];	/* text returned to readers */
	size_t			show_len;
	/* last write from a proc writer, waiting for the daemon's MSG ioctl */
	char			msg[PAYLOAD_LEN];
	size_t			msg_len;
	bool			msg_pending;
	wait_queue_head_t	wq;
	struct mutex		lock;
	struct list_head	node;
};

/*
 * AR_PROC_REQUEST_CREATE, _IOWR('M',6,144).
 * Pinned from ar_mpp_procdev_ioctl @0x1a14 (copy_from_user 144B; key read at
 * struct+128, flags at +132; handle written back at +136 via copy_to_user
 * @0x1a8c) and consistent with the vendor userspace create call (name into
 * +0/128B; key at +128; flags=1 at +132; reads the handle from +136 after the
 * ioctl).
 */
struct create_arg {		/* 144 bytes (0x90) */
	char	name[NAME_LEN];	/* +0   IN  procfs path (vendor allows '/') */
	__u32	key;		/* +128 IN  caller key/cookie */
	__u32	flags;		/* +132 IN  =1 when caller has a show/write cb */
	__u64	handle;		/* +136 OUT entry handle */
};
static_assert(sizeof(struct create_arg) == 144);

/*
 * AR_PROC_WRITE_MSG_TO_PROC_FS, _IOR('M',8,24)  (despite _IOR, kernel reads it).
 * Pushes the text that `cat /proc/umap/<name>` will return. Pinned from kernel
 * @0x18e0-0x19b8 (pbuf@0, len@8, handle@16; copies `len` bytes from *pbuf into
 * entry->show_buf, then NUL-terminates) and ar_proc_write @0x1938-0x1948
 * (stores data ptr@+0, len@+8, handle@+16, then ioctl 0x80184d08).
 */
struct write_arg {		/* 24 bytes (0x18) */
	__u64	pbuf;		/* +0  IN  user pointer to show text */
	__u32	len;		/* +8  IN  byte count */
	__u32	_pad;		/* +12 */
	__u64	handle;		/* +16 IN  entry handle */
};
static_assert(sizeof(struct write_arg) == 24);

/*
 * AR_PROC_REQUEST_MSG_FROM_PROC_DEV, _IOR('M',7,280), blocking. Delivers a
 * write that landed on one of this daemon's /proc files (so userspace can react
 * to `echo ... > /proc/umap/<name>`). Pinned from kernel @0x1bd4-0x1c98:
 * payload copied to struct+0 (min(write_len, caller_max)); length at +256 (IN:
 * caller's max, OUT: bytes copied); a constant 1 at +260; the producing entry
 * handle at +272.
 * TODO: the vendor also emits a second event class (a read/`cat` request: scans
 * the +0xc38 read-bitmap @0x1e08 and returns only handle@+272) so the daemon can
 * regenerate show text on demand. Not modelled here (we keep show text resident).
 */
struct msg_arg {		/* 280 bytes (0x118) */
	char	data[PAYLOAD_LEN];	/* +0   OUT payload written to the file */
	__u32	len;			/* +256 IN  caller max / OUT bytes copied */
	__u32	type;			/* +260 OUT 1 = write-from-proc event */
	__u32	flag;			/* +264 OUT read-request marker (TODO) */
	__u32	_pad;			/* +268 */
	__u64	handle;			/* +272 OUT producing entry handle */
};
static_assert(sizeof(struct msg_arg) == 280);

static struct {
	dev_t			devt;
	struct cdev		cdev;
	struct class		*class;
	struct device		*dev;
	struct proc_dir_entry	*dir;
	struct list_head	devs;
	struct mutex		lock;
	wait_queue_head_t	msg_wq;		/* woken when any proc_dev gets a msg */
	u64			next_handle;
} g;

static struct proc_dev *dev_by_handle(u64 h)
{
	struct proc_dev *d;

	list_for_each_entry(d, &g.devs, node)
		if (d->handle == h)
			return d;
	return NULL;
}

static int umap_show(struct seq_file *s, void *v)
{
	struct proc_dev *d = s->private;

	mutex_lock(&d->lock);
	seq_write(s, d->show_buf, d->show_len);
	mutex_unlock(&d->lock);
	return 0;
}

static int umap_open(struct inode *ino, struct file *f)
{
	return single_open(f, umap_show, pde_data(ino));
}

static ssize_t umap_write(struct file *f, const char __user *u, size_t n,
			  loff_t *off)
{
	struct proc_dev *d = pde_data(file_inode(f));
	size_t len = min(n, (size_t)PAYLOAD_LEN - 1);

	mutex_lock(&d->lock);
	memset(d->msg, 0, PAYLOAD_LEN);
	if (copy_from_user(d->msg, u, len)) {
		mutex_unlock(&d->lock);
		return -EFAULT;
	}
	d->msg_len = len;
	d->msg_pending = true;
	mutex_unlock(&d->lock);
	wake_up_interruptible(&g.msg_wq);
	return n;
}

static const struct proc_ops umap_proc_ops = {
	.proc_open	= umap_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
	.proc_write	= umap_write,
};

static long pc_create(void __user *arg, unsigned int sz)
{
	struct create_arg ca;
	struct proc_dev *d;

	if (sz != sizeof(ca) || copy_from_user(&ca, arg, sizeof(ca)))
		return -EFAULT;
	ca.name[sizeof(ca.name) - 1] = '\0';
	d = kzalloc(sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	init_waitqueue_head(&d->wq);
	mutex_init(&d->lock);
	/*
	 * The vendor strsep()s ca.name on '/' and proc_mkdir()s the leading
	 * components (nested /proc/umap/<a>/<b>); the skeleton registers a flat
	 * entry. TODO: replicate the nested-dir walk (_register_create_procfs_file
	 * @0x12a0) if a consumer relies on the hierarchy.
	 */
	strscpy(d->name, ca.name[0] ? ca.name : "umap", sizeof(d->name));
	d->key = ca.key;
	mutex_lock(&g.lock);
	d->handle = ++g.next_handle;
	d->pde = proc_create_data(d->name, 0644, g.dir, &umap_proc_ops, d);
	list_add_tail(&d->node, &g.devs);
	mutex_unlock(&g.lock);

	ca.handle = d->handle;	/* OUT: create_arg+136 */
	if (copy_to_user(arg, &ca, sizeof(ca)))
		return -EFAULT;
	return 0;
}

static long pc_msg(void __user *arg, unsigned int sz)
{
	struct msg_arg out;
	struct proc_dev *d;
	u32 cap = PAYLOAD_LEN;
	int ret;

	if (sz != sizeof(out))
		return -EINVAL;
	/* IN: caller's max payload size (msg_arg+256); vendor clamps to it. */
	if (get_user(cap, &((struct msg_arg __user *)arg)->len))
		return -EFAULT;
	cap = min_t(u32, cap, PAYLOAD_LEN);

	/* The first device with a pending write wins (simple model). */
	for (;;) {
		struct proc_dev *found = NULL;

		mutex_lock(&g.lock);
		list_for_each_entry(d, &g.devs, node) {
			if (d->msg_pending) {
				found = d;
				break;
			}
		}
		mutex_unlock(&g.lock);
		if (found) {
			memset(&out, 0, sizeof(out));
			mutex_lock(&found->lock);
			out.len = min_t(u32, found->msg_len, cap);
			memcpy(out.data, found->msg, out.len);
			out.type = 1;		/* write-from-proc event (re: +260) */
			out.handle = found->handle;
			found->msg_pending = false;
			mutex_unlock(&found->lock);
			break;
		}
		/* block until any device gets a message (poll guards lost wakeups) */
		ret = wait_event_interruptible_timeout(g.msg_wq, false, HZ);
		if (ret < 0)
			return ret;
	}
	if (copy_to_user(arg, &out, sizeof(out)))
		return -EFAULT;
	return 0;
}

/* nr8 AR_PROC_WRITE_MSG_TO_PROC_FS (24B): set the text `cat` returns. */
static long pc_write(void __user *arg, unsigned int sz)
{
	struct write_arg wa;
	struct proc_dev *d;
	size_t len;

	if (sz != sizeof(wa) || copy_from_user(&wa, arg, sizeof(wa)))
		return -EFAULT;
	mutex_lock(&g.lock);
	d = dev_by_handle(wa.handle);
	mutex_unlock(&g.lock);
	if (!d)
		return -EINVAL;

	/*
	 * Vendor copies wa.len bytes from the user pointer wa.pbuf into
	 * entry->show_buf (growing it via kmalloc) and NUL-terminates. We keep a
	 * fixed SHOW_LEN buffer and clamp; the seq_file show() returns show_len.
	 */
	len = min_t(size_t, wa.len, SHOW_LEN - 1);
	mutex_lock(&d->lock);
	if (copy_from_user(d->show_buf, (const void __user *)(uintptr_t)wa.pbuf,
			   len)) {
		mutex_unlock(&d->lock);
		return -EFAULT;
	}
	d->show_buf[len] = '\0';
	d->show_len = len;
	mutex_unlock(&d->lock);
	return 0;
}

/* nr9 AR_PROC_REQUEST_CLOSE (8B): remove a registered entry by handle. */
static long pc_close(void __user *arg, unsigned int sz)
{
	u64 h;
	struct proc_dev *d;

	if (sz != sizeof(h) || copy_from_user(&h, arg, sizeof(h)))
		return -EFAULT;
	mutex_lock(&g.lock);
	d = dev_by_handle(h);
	if (d) {
		if (d->pde)
			proc_remove(d->pde);
		list_del(&d->node);
	}
	mutex_unlock(&g.lock);
	if (!d)
		return -EINVAL;
	kfree(d);
	return 0;
}

static long pc_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	unsigned int nr = _IOC_NR(cmd), sz = _IOC_SIZE(cmd);
	void __user *uarg = (void __user *)arg;

	if (_IOC_TYPE(cmd) != MPP_MAGIC)
		return -EINVAL;
	switch (nr) {
	case 6: {
		return pc_create(uarg, sz);	/* CREATE              144 */
	}
	case 7: {
		return pc_msg(uarg, sz);	/* MSG_FROM_PROC_DEV   280 */
	}
	case 8: {
		return pc_write(uarg, sz);	/* WRITE_MSG_TO_PROC_FS 24 */
	}
	case 9: {
		return pc_close(uarg, sz);	/* CLOSE                 8 */
	}
	default: {
		return -EINVAL;
	}
	}
}

static const struct file_operations pc_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= pc_ioctl,
	.compat_ioctl	= compat_ptr_ioctl,
};

static int ar_mpp_proc_probe(struct platform_device *pdev)
{
	int ret;

	INIT_LIST_HEAD(&g.devs);
	mutex_init(&g.lock);
	init_waitqueue_head(&g.msg_wq);
	g.dir = proc_mkdir(PROC_DIR, NULL);

	ret = alloc_chrdev_region(&g.devt, 0, 1, "ar_mpp_proc_ctl");
	if (ret)
		goto err_proc;
	cdev_init(&g.cdev, &pc_fops);
	ret = cdev_add(&g.cdev, g.devt, 1);
	if (ret)
		goto err_region;
	/* NOT "adf_ctl": ar_mpp_drv already registers a class of that name, and a modern
	 * kernel rejects a duplicate class name in the same directory with -EEXIST, which
	 * failed this probe and tore down /proc/umap. Use a distinct class name.
	 */
	g.class = class_create("adf_proc_ctl");
	if (IS_ERR(g.class)) {
		ret = PTR_ERR(g.class);
		goto err_cdev;
	}
	g.dev = device_create(g.class, NULL, g.devt, NULL, "ar_mpp_proc_ctl");
	if (IS_ERR(g.dev)) {
		ret = PTR_ERR(g.dev);
		goto err_class;
	}
	dev_info(&pdev->dev, "ar_mpp_procdev_init done\n");
	return 0;

err_class:
	class_destroy(g.class);
err_cdev:
	cdev_del(&g.cdev);
err_region:
	unregister_chrdev_region(g.devt, 1);
err_proc:
	if (g.dir)
		proc_remove(g.dir);
	return ret;
}

static void ar_mpp_proc_remove(struct platform_device *pdev)
{
	struct proc_dev *d, *t;

	device_destroy(g.class, g.devt);
	class_destroy(g.class);
	cdev_del(&g.cdev);
	unregister_chrdev_region(g.devt, 1);
	list_for_each_entry_safe(d, t, &g.devs, node) {
		if (d->pde)
			proc_remove(d->pde);
		list_del(&d->node);
		kfree(d);
	}
	if (g.dir)
		proc_remove(g.dir);
}

static const struct of_device_id ar_mpp_proc_of[] = {
	{ .compatible = "artosyn,ar_mpp_proc" },
	{ }
};
MODULE_DEVICE_TABLE(of, ar_mpp_proc_of);

static struct platform_driver ar_mpp_proc_driver = {
	.probe	= ar_mpp_proc_probe,
	.remove	= ar_mpp_proc_remove,
	.driver	= {
		.name		= "ar_mpp_proc_ctrl",
		.of_match_table	= ar_mpp_proc_of,
	},
};
module_platform_driver(ar_mpp_proc_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("missinglynk (open reimpl)");
MODULE_DESCRIPTION("Artosyn /dev/ar_mpp_proc_ctl umap shuttle (open)");
