/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar_chrdev.h - the char-device bring-up ladder shared by the MPP control nodes.
 *
 * /dev/ar_mpp_ctl (ar_mpp_drv) and /dev/ar_mpp_proc_ctl (ar_mpp_proc_ctrl) create their
 * node the same way: alloc_chrdev_region, cdev_add, class_create, device_create. They
 * differ only in the node name, the class name, and the fops. Header-inline rather than a
 * shared object because the two are separate modules and neither should depend on the other.
 *
 * Teardown is registered per step with devm_add_action_or_reset as soon as that step
 * succeeds, so devm unwinds in reverse on both the probe error path and on remove: no unwind
 * ladder, and no remove() whose ordering has to be kept in step with probe by hand.
 */
#ifndef _AR_CHRDEV_H
#define _AR_CHRDEV_H

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>

struct ar_chrdev {
	dev_t			devt;
	struct cdev		cdev;
	struct class		*class;
	struct device		*dev;
};

static inline void ar_chrdev_put_region(void *data)
{
	unregister_chrdev_region(((struct ar_chrdev *)data)->devt, 1);
}

static inline void ar_chrdev_put_cdev(void *data)
{
	cdev_del(&((struct ar_chrdev *)data)->cdev);
}

static inline void ar_chrdev_put_class(void *data)
{
	class_destroy(((struct ar_chrdev *)data)->class);
}

static inline void ar_chrdev_put_device(void *data)
{
	struct ar_chrdev *c = data;

	device_destroy(c->class, c->devt);
}

/*
 * ar_chrdev_register - create a single-minor char device owned by @parent's devm scope.
 * @class_name must be unique kernel-wide: a duplicate class name in the same directory is
 * rejected with -EEXIST on a modern kernel.
 */
static inline int ar_chrdev_register(struct device *parent, struct ar_chrdev *c,
				     const struct file_operations *fops,
				     const char *name, const char *class_name)
{
	int ret;

	ret = alloc_chrdev_region(&c->devt, 0, 1, name);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(parent, ar_chrdev_put_region, c);
	if (ret)
		return ret;

	cdev_init(&c->cdev, fops);
	ret = cdev_add(&c->cdev, c->devt, 1);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(parent, ar_chrdev_put_cdev, c);
	if (ret)
		return ret;

	c->class = class_create(class_name);
	if (IS_ERR(c->class))
		return PTR_ERR(c->class);

	ret = devm_add_action_or_reset(parent, ar_chrdev_put_class, c);
	if (ret)
		return ret;

	c->dev = device_create(c->class, NULL, c->devt, NULL, name);
	if (IS_ERR(c->dev))
		return PTR_ERR(c->dev);

	return devm_add_action_or_reset(parent, ar_chrdev_put_device, c);
}

#endif /* _AR_CHRDEV_H */
