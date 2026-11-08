// SPDX-License-Identifier: GPL-2.0-only
/*
 * Power a hot-unplugged CPU fully off on MediaTek SoCs whose TF-A runs the
 * CPU power controller (CPC) in automatic mode, such as MT6833.
 *
 * There, PSCI CPU_OFF only arms the CPC's automatic power-off: the core
 * loses power when it reaches WFI, but its SPM power register keeps PWR_ON
 * set. PSCI CPU_ON then sets PWR_ON once more and waits for the core's
 * power acknowledge, which never comes without a 0->1 edge, so CPU_ON spins
 * in EL3 for good and the CPU that asked for it is lost. The firmware
 * expects the OS to clear PWR_ON once the core is dead, with the SiP call
 * MTK_SIP_POWER_DOWN_CORE; it refuses unless PSCI already counts the core
 * as off.
 */

#include <linux/arm-smccc.h>
#include <linux/cpu.h>
#include <linux/cpuhotplug.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>
#include <asm/cputype.h>
#include <asm/smp_plat.h>

#define MTK_SIP_POWER_DOWN_CORE		MTK_SIP_SMC_CMD(0x212)

static int mtk_cpu_hotplug_dead(unsigned int cpu)
{
	unsigned int core = MPIDR_AFFINITY_LEVEL(cpu_logical_map(cpu), 1);
	struct arm_smccc_res res;

	arm_smccc_smc(MTK_SIP_POWER_DOWN_CORE, core, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0)
		pr_warn("mtk-cpu-hotplug: CPU%u (core %u) not powered down: %ld\n",
			cpu, core, (long)res.a0);
	return 0;
}

static const char * const mtk_cpu_hotplug_machines[] __initconst = {
	"mediatek,mt6833",
	NULL
};

static int __init mtk_cpu_hotplug_init(void)
{
	int ret;

	if (!of_machine_compatible_match(mtk_cpu_hotplug_machines))
		return 0;

	/* Teardown runs on a control CPU after cpu_kill() saw the CPU off. */
	ret = cpuhp_setup_state_nocalls(CPUHP_BP_PREPARE_DYN,
					"soc/mediatek/cpu-hotplug:dead",
					NULL, mtk_cpu_hotplug_dead);
	return ret < 0 ? ret : 0;
}
device_initcall(mtk_cpu_hotplug_init);
