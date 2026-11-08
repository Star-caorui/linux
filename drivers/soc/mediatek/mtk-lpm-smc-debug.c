// SPDX-License-Identifier: GPL-2.0-only
/*
 * DEBUG ONLY: issue the MediaTek LPM SiP call (MTK_SIP_MTK_LPM_CONTROL) from
 * debugfs, to explore what the vendor TF-A's low-power code reports and
 * needs. Write "x1 x2 x3 x4" (numbers, any base) to
 * /sys/kernel/debug/mtk-lpm-smc; reading returns "x1 x2 x3 x4 -> a0 a1 a2 a3"
 * of the last call.
 */

#include <linux/arm-smccc.h>
#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/seq_file.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>
#include <linux/uaccess.h>

#define MTK_SIP_MTK_LPM_CONTROL		MTK_SIP_SMC_CMD(0x507)

static DEFINE_MUTEX(lock);
static unsigned long args[4];
static struct arm_smccc_res last;

static int smc_show(struct seq_file *s, void *unused)
{
	guard(mutex)(&lock);
	seq_printf(s, "%#lx %#lx %#lx %#lx -> %#lx %#lx %#lx %#lx\n",
		   args[0], args[1], args[2], args[3],
		   last.a0, last.a1, last.a2, last.a3);
	return 0;
}

static int smc_open(struct inode *inode, struct file *file)
{
	return single_open(file, smc_show, NULL);
}

static ssize_t smc_write(struct file *file, const char __user *ubuf,
			 size_t len, loff_t *ppos)
{
	char buf[128];
	unsigned long a[4] = { };
	int n;

	if (len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = 0;
	n = sscanf(buf, "%li %li %li %li", &a[0], &a[1], &a[2], &a[3]);
	if (n < 1)
		return -EINVAL;
	/* Only the LPM user IDs (0xda....) */
	if ((a[0] & 0xff000000) != 0xda000000)
		return -EPERM;

	guard(mutex)(&lock);
	memcpy(args, a, sizeof(args));
	arm_smccc_smc(MTK_SIP_MTK_LPM_CONTROL, a[0], a[1], a[2], a[3],
		      0, 0, 0, &last);
	pr_info("mtk-lpm-smc: %#lx %#lx %#lx %#lx -> %#lx %#lx\n",
		a[0], a[1], a[2], a[3], last.a0, last.a1);
	return len;
}

static const struct file_operations smc_fops = {
	.open = smc_open,
	.read = seq_read,
	.write = smc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init mtk_lpm_smc_debug_init(void)
{
	if (!of_machine_is_compatible("mediatek,mt6833"))
		return 0;
	debugfs_create_file("mtk-lpm-smc", 0600, NULL, NULL, &smc_fops);
	return 0;
}
late_initcall(mtk_lpm_smc_debug_init);
