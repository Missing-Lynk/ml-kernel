// SPDX-License-Identifier: GPL-2.0
/*
 * ar_mpp_drv.ko - open reimplementation of the Artosyn /dev/ar_mpp_ctl device.
 *
 * It is NOT a codec driver: it is an engine-agnostic GIC-IRQ forwarder.
 * Userspace programs the codec/jpeg/ge2d engines directly via
 * /dev/mem; this driver only forwards their completion interrupts. Userspace
 * REGISTERs an engine by GIC hwirq, blocks in WAIT_EVENT; the handler
 * disable_irq_nosync()s and pushes a 16-byte {u32 hwirq, u32 pad, u64 ktime_ns}
 * event, then userspace re-ENABLEs to re-arm.
 *
 * ABI (byte-verified against the vendor .ko):
 * magic 'M', nr 0..9:
 *   0: REGISTER (24: {u32 id; char name[20]})  1: FREE/UNREGISTER   2: DISABLE
 *   3: ENABLE (re-arm)   4: WAIT_EVENT (16)     5: unused by the daemon
 *   6: driver-info-by-name query (704)          7: WAIT_FIRST_VSYNC (24)
 *   8: QUERY_OOM (100)   9: QUERY totals (40)
 * REGISTER is nr0, byte-proven from the live daemon: ar_irq_reg_with_name issues
 * _IOW('M',0,24) (mov x20,#0x4d00 + movk #0x4018 at libmpp_service 0x778e8). The
 * id it passes is (gic_hwirq - 0x20) = the raw DTS SPI number (h26x sends 68, not
 * 100), reconciled in virq_for_hwirq().
 *
 * Binds to DT "artosyn,ar_mpp". The engine hwirqs come from the node's own
 * interrupts AND from its child nodes' interrupts (ahb_dma/axi_dma/h26x/...), so
 * the nr6 query can report a child's IRQ count by DT compatible name (what the
 * userspace AXI/AHB DMA init asserts on). IRQs are requested on demand (REGISTER),
 * so engines with their own driver (ar_scaler / GIC107) never conflict.
 * TODO(not-yet-hardware-validated): the nr6 struct field offsets and the child-walk
 * are RE-derived from libmpp_service; validate the decode completion path on device.
 */
#define pr_fmt(fmt) "ar_mpp_drv: " fmt

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/ktime.h>

#define MPP_MAGIC	'M'
#define MPP_RING_SZ	64
/* Table sizes: parent (h26x/jpeg/ge2d ~4) + ahb_dma (8) + axi_dma (3) + headroom. */
#define MAX_ENGINES	32
#define MPP_INFO_SZ	704		/* nr6 driver-info struct */

static unsigned int max_irq_event_cnt = MPP_RING_SZ;
module_param(max_irq_event_cnt, uint, 0444);

struct mpp_event {		/* 16 bytes, copied to userspace by WAIT */
	__u32	hwirq;
	__u32	pad;
	__u64	ktime_ns;
};

struct mpp_reg_arg {		/* 24-byte REGISTER arg */
	__u32	hwirq;
	__u32	flags;
	__u64	priv;
	__u64	_resv;
};

/* one per opened fd */
struct mpp_fd {
	wait_queue_head_t	wq;
	spinlock_t		lock;
	struct mpp_event	ring[MPP_RING_SZ];
	unsigned int		head, tail;
};

/* one per registered engine IRQ (global; IRQ handler dev_id points here) */
struct mpp_engine {
	bool		active;
	u32		hwirq;
	int		virq;
	struct mpp_fd	*owner;
};

static struct {
	dev_t			devt;
	struct cdev		cdev;
	struct class		*class;
	struct device		*dev;
	struct device_node	*np;		/* ar_mpp node, for the nr6 child lookup */
	void __iomem		*vsync_base;
	struct mpp_engine	engines[MAX_ENGINES];
	/* hwirq->virq table built from DT at probe */
	u32			hwirq_tbl[MAX_ENGINES];
	int			virq_tbl[MAX_ENGINES];
	int			n_irq;
	spinlock_t		reg_lock;
} g;

static int virq_for_hwirq(u32 hwirq)
{
	int i;

	/* The vendor daemon's REGISTER passes the id as (gic_hwirq - 0x20), i.e. the raw
	 * DTS SPI number (h26x sends 68, not the GIC hwirq 100), while our table is built
	 * from irqd_to_hwirq() (the GIC hwirq, e.g. 100). Match either the id as-is or the
	 * id + 0x20, so both the daemon's SPI-relative ids and direct GIC hwirqs resolve.
	 */
	for (i = 0; i < g.n_irq; i++) {
		if (g.hwirq_tbl[i] == hwirq || g.hwirq_tbl[i] == hwirq + 0x20)
			return g.virq_tbl[i];
	}
	return -ENOENT;
}

static irqreturn_t mpp_irq_handler(int irq, void *dev_id)
{
	struct mpp_engine *e = dev_id;
	struct mpp_fd *fd = e->owner;
	unsigned long flags;
	unsigned int next;

	/* one-shot: mask until userspace re-ENABLEs (vendor protocol) */
	disable_irq_nosync(irq);
	if (!fd)
		return IRQ_HANDLED;

	spin_lock_irqsave(&fd->lock, flags);
	next = (fd->head + 1) % MPP_RING_SZ;
	if (next != fd->tail) {		/* drop if ring full */
		fd->ring[fd->head].hwirq = e->hwirq;
		fd->ring[fd->head].pad = 0;
		fd->ring[fd->head].ktime_ns = ktime_get_raw_ns();
		fd->head = next;
	}
	spin_unlock_irqrestore(&fd->lock, flags);
	wake_up_interruptible(&fd->wq);
	return IRQ_HANDLED;
}

static int mpp_open(struct inode *ino, struct file *f)
{
	struct mpp_fd *fd = kzalloc(sizeof(*fd), GFP_KERNEL);

	if (!fd)
		return -ENOMEM;
	init_waitqueue_head(&fd->wq);
	spin_lock_init(&fd->lock);
	f->private_data = fd;
	return 0;
}

static int mpp_release(struct inode *ino, struct file *f)
{
	struct mpp_fd *fd = f->private_data;
	unsigned long flags;
	int i;

	/* free any engines this fd registered */
	spin_lock_irqsave(&g.reg_lock, flags);
	for (i = 0; i < MAX_ENGINES; i++) {
		if (g.engines[i].active && g.engines[i].owner == fd) {
			int virq = g.engines[i].virq;

			g.engines[i].active = false;
			g.engines[i].owner = NULL;
			spin_unlock_irqrestore(&g.reg_lock, flags);
			free_irq(virq, &g.engines[i]);
			spin_lock_irqsave(&g.reg_lock, flags);
		}
	}
	spin_unlock_irqrestore(&g.reg_lock, flags);
	kfree(fd);
	return 0;
}

static long mpp_register(struct mpp_fd *fd, void __user *arg, unsigned int sz)
{
	struct mpp_reg_arg ra;
	struct mpp_engine *e = NULL;
	unsigned long flags;
	int virq, i, ret;

	if (sz != sizeof(ra) || copy_from_user(&ra, arg, sizeof(ra)))
		return -EFAULT;
	virq = virq_for_hwirq(ra.hwirq);
	if (virq < 0) {
		pr_warn("REGISTER: id %u (or +0x20=%u) not in DT table\n", ra.hwirq, ra.hwirq + 0x20);
		return -EINVAL;
	}
	pr_info("REGISTER: id %u -> virq %d (request_irq)\n", ra.hwirq, virq);	/* TODO(debug) */
	spin_lock_irqsave(&g.reg_lock, flags);
	for (i = 0; i < MAX_ENGINES; i++) {
		if (!g.engines[i].active) {
			e = &g.engines[i];
			break;
		}
	}
	if (e) {
		e->active = true;
		e->hwirq = ra.hwirq;
		e->virq = virq;
		e->owner = fd;
	}
	spin_unlock_irqrestore(&g.reg_lock, flags);
	if (!e)
		return -ENOSPC;

	ret = request_irq(virq, mpp_irq_handler, 0, "ar_mpp", e);
	if (ret) {
		spin_lock_irqsave(&g.reg_lock, flags);
		e->active = false;
		e->owner = NULL;
		spin_unlock_irqrestore(&g.reg_lock, flags);
		return ret;
	}
	return 0;
}

static struct mpp_engine *engine_by_hwirq(u32 hwirq, struct mpp_fd *fd)
{
	int i;

	for (i = 0; i < MAX_ENGINES; i++) {
		if (g.engines[i].active && g.engines[i].owner == fd &&
		    g.engines[i].hwirq == hwirq) {
			return &g.engines[i];
		}
	}
	return NULL;
}

static long mpp_simple_hwirq(struct mpp_fd *fd, unsigned int nr,
			     void __user *arg, unsigned int sz)
{
	struct mpp_engine *e;
	unsigned long flags;
	u32 hwirq;
	int virq;

	if (sz != sizeof(hwirq) || copy_from_user(&hwirq, arg, sizeof(hwirq)))
		return -EFAULT;

	/* Look up the engine and snapshot its virq under reg_lock; the engines[]
	 * array is also mutated by REGISTER and by release on other CPUs. For FREE,
	 * detach the slot while still holding the lock so it can't be observed
	 * half-freed, then run the (sleeping) IRQ ops outside the spinlock.
	 */
	spin_lock_irqsave(&g.reg_lock, flags);
	e = engine_by_hwirq(hwirq, fd);
	if (!e) {
		spin_unlock_irqrestore(&g.reg_lock, flags);
		return -EINVAL;
	}
	virq = e->virq;
	if (nr == 1) {		/* FREE/UNREGISTER: detach before unlocking */
		e->active = false;
		e->owner = NULL;
	}
	spin_unlock_irqrestore(&g.reg_lock, flags);

	switch (nr) {
	case 1: {	/* FREE/UNREGISTER */
		free_irq(virq, e);
		return 0;
	}
	case 2: {	/* DISABLE */
		disable_irq(virq);
		return 0;
	}
	case 3: {	/* ENABLE (re-arm after each one-shot event) */
		enable_irq(virq);
		return 0;
	}
	}
	return -EINVAL;
}

static long mpp_wait(struct mpp_fd *fd, void __user *arg, unsigned int sz)
{
	struct mpp_event ev;
	unsigned long flags;
	int ret;

	if (sz < sizeof(ev))
		return -EINVAL;
	ret = wait_event_interruptible(fd->wq, fd->head != fd->tail);
	if (ret)
		return ret;
	spin_lock_irqsave(&fd->lock, flags);
	ev = fd->ring[fd->tail];
	fd->tail = (fd->tail + 1) % MPP_RING_SZ;
	spin_unlock_irqrestore(&fd->lock, flags);
	if (copy_to_user(arg, &ev, sizeof(ev)))
		return -EFAULT;
	return 0;
}

/* nr7: poll the display vsync register with a 100 ms timeout (the vendor's only
 * MMIO path). Returns the timestamp. Best-effort without the exact bit semantics.
 */
static long mpp_vsync_poll(void __user *arg, unsigned int sz)
{
	u8 buf[24] = {0};
	ktime_t deadline = ktime_add_ms(ktime_get(), 100);
	u32 prev = 0;

	if (g.vsync_base) {
		prev = readl(g.vsync_base);
		while (ktime_before(ktime_get(), deadline)) {
			if (readl(g.vsync_base) != prev)
				break;
			cpu_relax();
		}
	}
	*(u64 *)buf = ktime_get_raw_ns();
	if (sz > sizeof(buf))
		sz = sizeof(buf);
	if (copy_to_user(arg, buf, sz))
		return -EFAULT;
	return 0;
}

/* nr6: driver-info-by-name query (_IOR('M',6,704)). Userspace writes a DT compatible
 * string into name[0..31]; the kernel finds the matching ar_mpp child node and returns
 * its IRQ count + hwirq list + register bases. libmpp_service's AHB/AXI DMA init asserts
 * total_irq_num equals the channel count (ahb=8, axi=3), so a wrong count aborts SYS_Init.
 * Field offsets are RE-derived from libmpp_service: name@+0[32],
 * total_irq_num@+32, hwirq[]@+40, reg_count@+184, reg[0].base@+192, reg[1].base@+200.
 * The returned hwirqs match g.hwirq_tbl (same irqd_to_hwirq source), so the userspace can
 * echo them straight back to REGISTER (nr5). TODO(not-yet-hardware-validated): the array
 * and reg-base offsets are inferred, not byte-proven.
 */
static long mpp_driver_info(void __user *arg, unsigned int sz)
{
	struct device_node *child, *match = NULL;
	char name[33];
	int single_idx = -1;
	u8 *buf;
	long ret = 0;

	if (sz < MPP_INFO_SZ)
		return -EINVAL;
	buf = kzalloc(MPP_INFO_SZ, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	if (copy_from_user(buf, arg, MPP_INFO_SZ)) {
		ret = -EFAULT;
		goto out;
	}
	memcpy(name, buf, 32);
	name[32] = '\0';

	/* Resolve the queried compatible to an IRQ source. Two forms: (1) a real ar_mpp
	 * CHILD node with that compatible (ahb_dma/axi_dma) - return all its interrupts;
	 * (2) an engine that lives as a parent interrupt-name (h26x/jpeg/ge2d, which the
	 * open DTS flattened onto ar_mpp itself) - match the compatible's suffix after the
	 * comma against ar_mpp's "interrupt-names" and return that single interrupt. The
	 * vendor's h26x codec queries "artosyn,h26x" this way and needs SPI 68 / GIC 100.
	 */
	if (g.np) {
		for_each_child_of_node(g.np, child) {
			if (of_device_is_compatible(child, name)) {
				match = child;		/* keep the ref; put below */
				break;
			}
		}
	}
	if (!match && g.np) {
		const char *suffix = strrchr(name, ',');

		suffix = suffix ? suffix + 1 : name;
		if (suffix[0]) {
			int idx = of_property_match_string(g.np, "interrupt-names", suffix);

			if (idx >= 0) {
				single_idx = idx;	/* one named interrupt of ar_mpp itself */
				match = g.np;		/* not ref-counted; do not of_node_put */
			}
		}
	}

	/* Rebuild the reply body, preserving the caller's name field at +0. The daemon reads
	 * total_irq_num at +32 and the hwirq array at +36 (proven from ar_get_driver_info).
	 */
	memset(buf + 32, 0, MPP_INFO_SZ - 32);
	if (match) {
		struct resource res;
		u32 n, i, reg_count = 0;

		n = (single_idx >= 0) ? 1 : of_irq_count(match);
		if (n > (184 - 36) / 4) {	/* keep the array below the reg fields at +184 */
			n = (184 - 36) / 4;
		}
		*(u32 *)(buf + 32) = n;
		for (i = 0; i < n; i++) {
			int j = (single_idx >= 0) ? single_idx : (int)i;
			int virq = of_irq_get(match, j);
			struct irq_data *d = virq > 0 ? irq_get_irq_data(virq) : NULL;

			*(u32 *)(buf + 36 + 4 * i) = d ? (u32)irqd_to_hwirq(d) : 0;
		}
		if (single_idx < 0) {	/* only child nodes carry reg windows */
			if (of_address_to_resource(match, 0, &res) == 0) {
				*(u64 *)(buf + 192) = res.start;
				reg_count = 1;
				if (of_address_to_resource(match, 1, &res) == 0) {
					*(u64 *)(buf + 200) = res.start;
					reg_count = 2;
				}
			}
			*(u32 *)(buf + 184) = reg_count;
		}
		pr_info("nr6 driver-info: '%s' -> %u irqs (first hwirq %u), %u regs\n",	/* TODO(debug) */
			name, n, *(u32 *)(buf + 36), reg_count);
		if (single_idx < 0)
			of_node_put(match);
	} else {
		pr_info("nr6 driver-info: '%s' -> not found (0 irqs)\n", name);	/* TODO(debug) */
	}

	if (copy_to_user(arg, buf, MPP_INFO_SZ))
		ret = -EFAULT;
out:
	kfree(buf);
	return ret;
}

static long mpp_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	struct mpp_fd *fd = f->private_data;
	unsigned int nr = _IOC_NR(cmd), sz = _IOC_SIZE(cmd);
	void __user *uarg = (void __user *)arg;
	u8 zero[704];

	if (_IOC_TYPE(cmd) != MPP_MAGIC)
		return -EINVAL;

	switch (nr) {
	case 0: {	/* REGISTER by hwirq (the vendor daemon's ar_irq_reg_with_name uses
			 * _IOW('M',0,24); proven from libmpp_service 0x778e8). This is the
			 * call that request_irq()s the codec completion IRQ (h26x hwirq 100).
			 */
		return mpp_register(fd, uarg, sz);
	}
	case 1:
	case 2:
	case 3: {	/* FREE / DISABLE / ENABLE */
		return mpp_simple_hwirq(fd, nr, uarg, sz);
	}
	case 4: {	/* WAIT_EVENT */
		return mpp_wait(fd, uarg, sz);
	}
	case 5: {	/* unused by the vendor daemon (no _IOW('M',5,..) call site) */
		return 0;
	}
	case 6: {	/* driver-info-by-name query */
		return mpp_driver_info(uarg, sz);
	}
	case 7: {	/* WAIT_FIRST_VSYNC */
		return mpp_vsync_poll(uarg, sz);
	}
	case 8:	/* QUERY_OOM (100) */
	case 9: {	/* QUERY totals (40) */
		/* not used by the codec/DMA path - zero-fill until a caller needs them. */
		memset(zero, 0, sizeof(zero));
		if (sz > sizeof(zero))
			sz = sizeof(zero);
		return copy_to_user(uarg, zero, sz) ? -EFAULT : 0;
	}
	default: {
		return -EINVAL;
	}
	}
}

static const struct file_operations mpp_fops = {
	.owner		= THIS_MODULE,
	.open		= mpp_open,
	.release	= mpp_release,
	.unlocked_ioctl	= mpp_ioctl,
	.compat_ioctl	= compat_ptr_ioctl,
};

static int ar_mpp_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *child;
	int i, ret, n;

	spin_lock_init(&g.reg_lock);
	g.np = np;

	/* Build the hwirq->virq table from the node's own interrupts (h26x/jpeg/ge2d)
	 * AND from every child node's interrupts (ahb_dma/axi_dma/...). The vendor
	 * layout puts the DMA engines in child nodes, and the userspace queries them by
	 * child compatible (nr6), so both levels must be in the table for REGISTER.
	 */
	n = of_irq_count(np);
	for (i = 0; i < n && g.n_irq < MAX_ENGINES; i++) {
		int virq = of_irq_get(np, i);
		struct irq_data *d;

		if (virq <= 0)
			continue;
		d = irq_get_irq_data(virq);
		g.virq_tbl[g.n_irq] = virq;
		g.hwirq_tbl[g.n_irq] = d ? (u32)irqd_to_hwirq(d) : 0;
		g.n_irq++;
		dev_info(&pdev->dev, "engine irq[%d]: hwirq %u -> virq %d\n",
			 i, g.hwirq_tbl[g.n_irq - 1], virq);
	}
	for_each_child_of_node(np, child) {
		int cn = of_irq_count(child), j;

		for (j = 0; j < cn && g.n_irq < MAX_ENGINES; j++) {
			int virq = of_irq_get(child, j);
			struct irq_data *d;

			if (virq <= 0)
				continue;
			d = irq_get_irq_data(virq);
			g.virq_tbl[g.n_irq] = virq;
			g.hwirq_tbl[g.n_irq] = d ? (u32)irqd_to_hwirq(d) : 0;
			g.n_irq++;
			dev_info(&pdev->dev, "child %pOFn irq[%d]: hwirq %u -> virq %d\n",
				 child, j, g.hwirq_tbl[g.n_irq - 1], virq);
		}
	}

	/* optional vsync reg window (reg index 0) */
	g.vsync_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(g.vsync_base))
		g.vsync_base = NULL;

	ret = alloc_chrdev_region(&g.devt, 0, 1, "ar_mpp_ctl");
	if (ret)
		return ret;
	cdev_init(&g.cdev, &mpp_fops);
	ret = cdev_add(&g.cdev, g.devt, 1);
	if (ret)
		goto err_region;
	g.class = class_create("adf_ctl");
	if (IS_ERR(g.class)) {
		ret = PTR_ERR(g.class);
		goto err_cdev;
	}
	g.dev = device_create(g.class, NULL, g.devt, NULL, "ar_mpp_ctl");
	if (IS_ERR(g.dev)) {
		ret = PTR_ERR(g.dev);
		goto err_class;
	}
	dev_info(&pdev->dev, "ready (%d engine irqs)\n", g.n_irq);
	return 0;

err_class:
	class_destroy(g.class);
err_cdev:
	cdev_del(&g.cdev);
err_region:
	unregister_chrdev_region(g.devt, 1);
	return ret;
}

static void ar_mpp_remove(struct platform_device *pdev)
{
	device_destroy(g.class, g.devt);
	class_destroy(g.class);
	cdev_del(&g.cdev);
	unregister_chrdev_region(g.devt, 1);
}

static const struct of_device_id ar_mpp_of[] = {
	{ .compatible = "artosyn,ar_mpp" },
	{ }
};
MODULE_DEVICE_TABLE(of, ar_mpp_of);

static struct platform_driver ar_mpp_driver = {
	.probe	= ar_mpp_probe,
	.remove	= ar_mpp_remove,
	.driver	= {
		.name		= "ar_mpp_drv",
		.of_match_table	= ar_mpp_of,
	},
};
module_platform_driver(ar_mpp_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("missinglynk (open reimpl)");
MODULE_DESCRIPTION("Artosyn /dev/ar_mpp_ctl GIC-IRQ forwarder (open)");
