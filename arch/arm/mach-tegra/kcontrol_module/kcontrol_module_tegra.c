/*
 * KControl GPU module for tegra devices
 *
 * Copyright (c) 2013 Dennis Rassmann
 * Author: Dennis Rassmann <showp1984@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/kallsyms.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/dcache.h>
#include "../dvfs.h"
#include "..//clock.h"

#define THIS_EXPERIMENTAL 0

#define DRIVER_AUTHOR "Dennis Rassmann <showp1984@gmail.com>"
#define DRIVER_DESCRIPTION "KControl GPU module for tegra devices"
#define DRIVER_VERSION "1.0"
#define LOGTAG "kcontrol_gpu_tegra: "

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");

enum {
	CLK_VDE = 0,
	CLK_MPE,
	CLK_2D,
	CLK_EPP,
	CLK_3D,
	CLK_3D2,
	CLK_SE,
	CLK_CBUS
};

struct global_attr_kcontrol {
	struct attribute attr;
	ssize_t (*show)(struct kobject *kobj,
			struct attribute *attr, char *buf);
	ssize_t (*store)(struct kobject *a, struct attribute *b,
			 const char *c, size_t count);
};

#define define_one_global_ro_kcontrol(_name)		\
static struct global_attr_kcontrol _name =		\
__ATTR(_name, 0444, show_##_name, NULL)

#define define_one_global_rw_kcontrol(_name)		\
static struct global_attr_kcontrol _name =		\
__ATTR(_name, 0644, show_##_name, store_##_name)

/* module parameters */
static uint dvfs_core_table = 0x00000000;
module_param(dvfs_core_table, uint, 0444);

static uint soc_speedo_id = 0x00000000;
module_param(soc_speedo_id, uint, 0444);

static struct dvfs *core_table;
static int *soc_speedo;

struct kobject *kcontrol_gpu_tegra_kobject;

static ssize_t show_tegra_freqs(struct kobject *a, struct attribute *b,
				   char *buf)
{
	ssize_t len = 0;
	int i = 0, j=0;
	if ((core_table != NULL) && (soc_speedo != NULL)) {
		struct dvfs *d = core_table;
		for (i=0; (strcmp((d->clk_name), "spdif_out") != 0); i++) {
				d = (core_table+i);
			if (d->speedo_id != *soc_speedo)
				continue;
			if ((strcmp(d->clk_name, "vde") == 0) ||
				(strcmp(d->clk_name, "mpe") == 0) ||
				(strcmp(d->clk_name, "2d") == 0) ||
				(strcmp(d->clk_name, "epp") == 0) ||
				(strcmp(d->clk_name, "3d") == 0) ||
				(strcmp(d->clk_name, "3d2") == 0) ||
				(strcmp(d->clk_name, "se") == 0) ||
				(strcmp(d->clk_name, "cbus") == 0)) {
				for (j=0; j<(d->num_freqs); j++) {
					if (d->freqs[j] < 1000000)
						continue;
					len += sprintf(buf + len, "%s %lu\n", d->clk_name, d->freqs[j]);
				}
				if (strcmp(d->clk_name, "cbus") == 0) {
					break;
				}
			}
		}
	} else {
		len += sprintf(buf + len, "Error! Pointer == null!\n");
	}
	return len;
}
static ssize_t store_tegra_freqs(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	int i = 0;
	unsigned int clock = 0, freq = 0;
	long unsigned int hz = 0;
	const char *clk = NULL;
	struct dvfs *d = core_table;
	struct clk *set_clk = NULL;
	struct clk *shared_bus_user;

	if ((core_table != NULL) && (soc_speedo != NULL)) {
		if ((buf[0] >= 0) &&
			(buf[1] == ' ') &&
			(buf[2] >= 0) &&
			(buf[3] == ' ')) {
			sscanf(buf, "%u %u %lu", &clock, &freq, &hz);
			freq++;
			switch (clock) {
			case CLK_VDE:
				clk = "vde";
				break;
			case CLK_MPE:
				clk = "mpe";
				break;
			case CLK_2D:
				clk = "2d";
				break;
			case CLK_EPP:
				clk = "epp";
				break;
			case CLK_3D:
				clk = "3d";
				break;
			case CLK_3D2:
				clk = "3d2";
				break;
			case CLK_SE:
				clk = "se";
				break;
			case CLK_CBUS:
				clk = "cbus";
				break;
			}
			set_clk = tegra_get_clock_by_name(clk);
			if (set_clk != NULL) {
				if (set_clk->max_rate < hz) {
					if (set_clk->max_rate != hz) {
						pr_warn(LOGTAG"Increasing %s maximum rate from %lu to %lu\n", set_clk->name, set_clk->max_rate, hz);

						set_clk->max_rate = hz;
						list_for_each_entry(shared_bus_user,
									&set_clk->shared_bus_list, u.shared_bus_user.node) {
							shared_bus_user->u.shared_bus_user.rate = hz;
							shared_bus_user->max_rate = hz;
						}
					}
				}
			}
			for (i=0; (strcmp((d->clk_name), "spdif_out") != 0); i++) {
				d = (core_table+i);
				if (d->speedo_id != *soc_speedo)
					continue;
				if (strcmp((d->clk_name), clk) == 0) {
					d->freqs[freq] = hz;
					break;
				}
			}
		}
	} else {
		pr_err(LOGTAG"Error! Pointer == null!\n");
	}
	return count;
}
define_one_global_rw_kcontrol(tegra_freqs);

static ssize_t show_tegra_maxfreqs(struct kobject *a, struct attribute *b,
				   char *buf)
{
	ssize_t len = 0;
	int i = 0;
	struct clk *set_clk = NULL;

	if ((core_table != NULL) && (soc_speedo != NULL)) {
		struct dvfs *d = core_table;
		for (i=0; (strcmp((d->clk_name), "spdif_out") != 0); i++) {
				d = (core_table+i);
			if (d->speedo_id != *soc_speedo)
				continue;
			if ((strcmp(d->clk_name, "vde") == 0) ||
				(strcmp(d->clk_name, "mpe") == 0) ||
				(strcmp(d->clk_name, "2d") == 0) ||
				(strcmp(d->clk_name, "epp") == 0) ||
				(strcmp(d->clk_name, "3d") == 0) ||
				(strcmp(d->clk_name, "3d2") == 0) ||
				(strcmp(d->clk_name, "se") == 0) ||
				(strcmp(d->clk_name, "cbus") == 0)) {
				set_clk = tegra_get_clock_by_name(d->clk_name);
				if (set_clk != NULL) {
					len += sprintf(buf + len, "%s %lu\n", d->clk_name, set_clk->max_rate);
				}
				if (strcmp(d->clk_name, "cbus") == 0) {
					break;
				}
			}
		}
	} else {
		len += sprintf(buf + len, "Error! Pointer == null!\n");
	}
	return len;
}

extern unsigned long clk_get_rate(struct clk *);
static ssize_t store_tegra_maxfreqs(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int clock = 0;
	long unsigned int hz = 0;
	const char *clk = NULL;
	struct clk *set_clk = NULL;
	struct clk *shared_bus_user;

	if (core_table != NULL) {
		if ((buf[0] >= 0) &&
			(buf[1] == ' ')) {
			sscanf(buf, "%u %lu", &clock, &hz);
			switch (clock) {
			case CLK_VDE:
				clk = "vde";
				break;
			case CLK_MPE:
				clk = "mpe";
				break;
			case CLK_2D:
				clk = "2d";
				break;
			case CLK_EPP:
				clk = "epp";
				break;
			case CLK_3D:
				clk = "3d";
				break;
			case CLK_3D2:
				clk = "3d2";
				break;
			case CLK_SE:
				clk = "se";
				break;
			case CLK_CBUS:
				clk = "cbus";
				break;
			}
			set_clk = tegra_get_clock_by_name(clk);
			if (set_clk != NULL) {
				if (set_clk->max_rate == hz)
					return count;

				pr_warn(LOGTAG"Changing %s maximum rate from %lu to %lu\n", set_clk->name, set_clk->max_rate, hz);

				set_clk->max_rate = hz;
				list_for_each_entry(shared_bus_user,
							&set_clk->shared_bus_list, u.shared_bus_user.node) {
					shared_bus_user->u.shared_bus_user.rate = hz;
					shared_bus_user->max_rate = hz;
				}
			}
		}
	} else {
		pr_err(LOGTAG"Error! Pointer == null!\n");
	}
	return count;
}
define_one_global_rw_kcontrol(tegra_maxfreqs);

static ssize_t show_tegra_curfreqs(struct kobject *a, struct attribute *b,
				   char *buf)
{
	ssize_t len = 0;
	int i = 0;
	const char *clk = NULL;
	struct clk *set_clk = NULL;

	if ((core_table != NULL) && (soc_speedo != NULL)) {
		struct dvfs *d = core_table;
		for (i=0; (strcmp((d->clk_name), "spdif_out") != 0); i++) {
				d = (core_table+i);
			if (d->speedo_id != *soc_speedo)
				continue;
			if ((strcmp(d->clk_name, "vde") == 0) ||
				(strcmp(d->clk_name, "mpe") == 0) ||
				(strcmp(d->clk_name, "2d") == 0) ||
				(strcmp(d->clk_name, "epp") == 0) ||
				(strcmp(d->clk_name, "3d") == 0) ||
				(strcmp(d->clk_name, "3d2") == 0) ||
				(strcmp(d->clk_name, "se") == 0) ||
				(strcmp(d->clk_name, "cbus") == 0)) {
				if (strcmp(d->clk_name, "vde") == 0)
					clk = "vde";
				if (strcmp(d->clk_name, "mpe") == 0)
					clk = "mpe";
				if (strcmp(d->clk_name, "2d") == 0)
					clk = "2d";
				if (strcmp(d->clk_name, "epp") == 0)
					clk = "epp";
				if (strcmp(d->clk_name, "3d") == 0)
					clk = "3d";
				if (strcmp(d->clk_name, "3d2") == 0)
					clk = "3d2";
				if (strcmp(d->clk_name, "se") == 0)
					clk = "se";
				if (strcmp(d->clk_name, "cbus") == 0)
					clk = "cbus";
				set_clk = tegra_get_clock_by_name(clk);
				if (set_clk != NULL) {
					if ((d->cur_rate == 0) &&
						((strcmp(d->clk_name, "vde") == 0) ||
						(strcmp(d->clk_name, "mpe") == 0) ||
						(strcmp(d->clk_name, "se") == 0))) {
						len += sprintf(buf + len, "%s %lu\n", d->clk_name, clk_get_rate(set_clk));
					} else {
						len += sprintf(buf + len, "%s %lu\n", d->clk_name, d->cur_rate);
					}
				}
			}
			if (strcmp(d->clk_name, "cbus") == 0) {
				break;
			}
		}
	} else {
		len += sprintf(buf + len, "Error! Pointer == null!\n");
	}
	return len;
}
define_one_global_ro_kcontrol(tegra_curfreqs);

static ssize_t show_version(struct kobject *a, struct attribute *b,
				   char *buf)
{
	ssize_t len = 0;
	len += sprintf(buf + len, DRIVER_VERSION);
	len += sprintf(buf + len, "\n");
	return len;
}
define_one_global_ro_kcontrol(version);

static struct attribute *kcontrol_gpu_tegra_attributes[] = {
	&version.attr,
	&tegra_freqs.attr,
	&tegra_curfreqs.attr,
	&tegra_maxfreqs.attr,
	NULL
};

static struct attribute_group kcontrol_gpu_tegra_attr_group = {
	.attrs = kcontrol_gpu_tegra_attributes,
	.name = "kcontrol_gpu_tegra",
};

static int __init kcontrol_gpu_tegra_init(void)
{
	int rc = 0;

#if THIS_EXPERIMENTAL
    printk(KERN_WARNING LOGTAG "#######################################");
    printk(KERN_WARNING LOGTAG "WARNING: THIS MODULE IS EXPERIMENTAL!\n");
    printk(KERN_WARNING LOGTAG "You have been warned.\n");
	printk(KERN_INFO LOGTAG "%s, version %s\n", DRIVER_DESCRIPTION,	DRIVER_VERSION);
	printk(KERN_INFO LOGTAG "author: %s\n", DRIVER_AUTHOR);
    printk(KERN_WARNING LOGTAG "#######################################");
#else
	printk(KERN_INFO LOGTAG "%s, version %s\n", DRIVER_DESCRIPTION,	DRIVER_VERSION);
	printk(KERN_INFO LOGTAG "author: %s\n", DRIVER_AUTHOR);
#endif

	WARN(dvfs_core_table == 0x00000000, LOGTAG "dvfs_core_table == 0x00000000!");
	WARN(soc_speedo_id == 0x00000000, LOGTAG "soc_speedo_id == 0x00000000!");

	if ((dvfs_core_table != 0x00000000) && (soc_speedo_id != 0x00000000)) {
		core_table = (struct dvfs *)dvfs_core_table;
		soc_speedo = (int *)soc_speedo_id;

		if (kernel_kobj) {
			rc = sysfs_create_group(kernel_kobj, &kcontrol_gpu_tegra_attr_group);
			if (rc) {
				pr_warn(LOGTAG"sysfs: ERROR, could not create sysfs group");
			}
		} else
			pr_warn(LOGTAG"sysfs: ERROR, could not find sysfs kobj");

		pr_info(LOGTAG "everything done, have fun!\n");
	} else {
		pr_err(LOGTAG "Error, you need to insert this module WITH parameters!\n");
		pr_err(LOGTAG "Nothing modified, removing myself!\n");
		return -EAGAIN;
	}
	return 0;
}

static void __exit kcontrol_gpu_tegra_exit(void)
{
	sysfs_remove_group(kernel_kobj, &kcontrol_gpu_tegra_attr_group);
	printk(KERN_INFO LOGTAG "unloaded\n");
}

module_init(kcontrol_gpu_tegra_init);
module_exit(kcontrol_gpu_tegra_exit);
