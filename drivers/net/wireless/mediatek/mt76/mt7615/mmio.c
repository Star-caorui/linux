// SPDX-License-Identifier: BSD-3-Clause-Clear
/* Copyright (C) 2020 MediaTek Inc. */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pci.h>

#include "mt7615.h"
#include "regs.h"
#include "mac.h"
#include "../trace.h"

const u32 mt7615e_reg_map[] = {
	[MT_TOP_CFG_BASE]	= 0x01000,
	[MT_HW_BASE]		= 0x01000,
	[MT_PCIE_REMAP_2]	= 0x02504,
	[MT_ARB_BASE]		= 0x20c00,
	[MT_HIF_BASE]		= 0x04000,
	[MT_CSR_BASE]		= 0x07000,
	[MT_PLE_BASE]		= 0x08000,
	[MT_PSE_BASE]		= 0x0c000,
	[MT_CFG_BASE]		= 0x20200,
	[MT_AGG_BASE]		= 0x20a00,
	[MT_TMAC_BASE]		= 0x21000,
	[MT_RMAC_BASE]		= 0x21200,
	[MT_DMA_BASE]		= 0x21800,
	[MT_PF_BASE]		= 0x22000,
	[MT_WTBL_BASE_ON]	= 0x23000,
	[MT_WTBL_BASE_OFF]	= 0x23400,
	[MT_LPON_BASE]		= 0x24200,
	[MT_MIB_BASE]		= 0x24800,
	[MT_WTBL_BASE_ADDR]	= 0x30000,
	[MT_PCIE_REMAP_BASE2]	= 0x80000,
	[MT_TOP_MISC_BASE]	= 0xc0000,
	[MT_EFUSE_ADDR_BASE]	= 0x81070000,
};

const u32 mt7663e_reg_map[] = {
	[MT_TOP_CFG_BASE]	= 0x01000,
	[MT_HW_BASE]		= 0x02000,
	[MT_DMA_SHDL_BASE]	= 0x06000,
	[MT_PCIE_REMAP_2]	= 0x0700c,
	[MT_ARB_BASE]		= 0x20c00,
	[MT_HIF_BASE]		= 0x04000,
	[MT_CSR_BASE]		= 0x07000,
	[MT_PLE_BASE]		= 0x08000,
	[MT_PSE_BASE]		= 0x0c000,
	[MT_PP_BASE]            = 0x0e000,
	[MT_CFG_BASE]		= 0x20000,
	[MT_AGG_BASE]		= 0x22000,
	[MT_TMAC_BASE]		= 0x24000,
	[MT_RMAC_BASE]		= 0x25000,
	[MT_DMA_BASE]		= 0x27000,
	[MT_PF_BASE]		= 0x28000,
	[MT_WTBL_BASE_ON]	= 0x29000,
	[MT_WTBL_BASE_OFF]	= 0x29800,
	[MT_LPON_BASE]		= 0x2b000,
	[MT_MIB_BASE]		= 0x2d000,
	[MT_WTBL_BASE_ADDR]	= 0x30000,
	[MT_PCIE_REMAP_BASE2]	= 0x90000,
	[MT_TOP_MISC_BASE]	= 0xc0000,
	[MT_EFUSE_ADDR_BASE]	= 0x78011000,
};

static void
mt7615_rx_poll_complete(struct mt76_dev *mdev, enum mt76_rxq_id q)
{
	mt76_connac_irq_enable(mdev, MT_INT_RX_DONE(q));
}

static void mt6833_irq_disable(struct mt7615_dev *dev)
{
	if (dev->mt6833_irq > 0 &&
	    !test_and_set_bit(0, &dev->mt6833_irq_state))
		disable_irq_nosync(dev->mt6833_irq);
}

static void mt6833_irq_enable(struct mt7615_dev *dev)
{
	if (dev->mt6833_irq > 0 &&
	    test_and_clear_bit(0, &dev->mt6833_irq_state))
		enable_irq(dev->mt6833_irq);
}

static irqreturn_t mt7615_irq_handler(int irq, void *dev_instance)
{
	struct mt7615_dev *dev = dev_instance;

	if (is_mt6833(&dev->mt76)) {
		if (!test_bit(MT76_STATE_INITIALIZED, &dev->mphy.state))
			return IRQ_NONE;
		mt6833_irq_disable(dev);
	} else {
		mt76_wr(dev, MT_INT_MASK_CSR, 0);
		if (!test_bit(MT76_STATE_INITIALIZED, &dev->mphy.state))
			return IRQ_NONE;
	}

	tasklet_schedule(&dev->mt76.irq_tasklet);

	return IRQ_HANDLED;
}

static void mt7615_irq_tasklet(struct tasklet_struct *t)
{
	struct mt7615_dev *dev = from_tasklet(dev, t, mt76.irq_tasklet);
	u32 raw_intr, intr, mask = 0, tx_mcu_mask = mt7615_tx_mcu_int_mask(dev);
	u32 mcu_int;

	if (is_mt6833(&dev->mt76) &&
	    !mt76_connac_pm_ref(&dev->mphy, &dev->pm)) {
		if (test_bit(0, &dev->mt6833_irq_state))
			queue_work(dev->mt76.wq, &dev->pm.wake_work);
		return;
	}

	mt76_wr(dev, MT_INT_MASK_CSR, 0);

	raw_intr = mt76_rr(dev, MT_INT_SOURCE_CSR);
	intr = raw_intr & dev->mt76.mmio.irqmask;
	mt76_wr(dev, MT_INT_SOURCE_CSR, is_mt6833(&dev->mt76) ? raw_intr : intr);

	trace_dev_irq(&dev->mt76, intr, dev->mt76.mmio.irqmask);

	mask |= intr & MT_INT_RX_DONE_ALL;
	if (intr & tx_mcu_mask)
		mask |= tx_mcu_mask;
	mt76_set_irq_mask(&dev->mt76, MT_INT_MASK_CSR, mask, 0);

	if (intr & tx_mcu_mask)
		napi_schedule(&dev->mt76.tx_napi);

	if (intr & MT_INT_RX_DONE(0))
		napi_schedule(&dev->mt76.napi[0]);

	if (intr & MT_INT_RX_DONE(1))
		napi_schedule(&dev->mt76.napi[1]);

	if (!(intr & (MT_INT_MCU_CMD | MT7663_INT_MCU_CMD)))
		goto out;

	if (is_mt7663(&dev->mt76)) {
		mcu_int = mt76_rr(dev, MT_MCU2HOST_INT_STATUS);
		mcu_int &= MT7663_MCU_CMD_ERROR_MASK;
		mt76_wr(dev, MT_MCU2HOST_INT_STATUS, mcu_int);
	} else {
		mcu_int = mt76_rr(dev, MT_MCU_CMD);
		mcu_int &= MT_MCU_CMD_ERROR_MASK;
	}

	if (!mcu_int)
		goto out;

	dev->reset_state = mcu_int;
	queue_work(dev->mt76.wq, &dev->reset_work);
	wake_up(&dev->reset_wait);

out:
	if (is_mt6833(&dev->mt76)) {
		mt6833_irq_enable(dev);
		mt76_connac_pm_unref(&dev->mphy, &dev->pm);
	}
}

/*
 * MT6833: the WMAC's 1 MiB window shows these chip address ranges at fixed offsets (MediaTek's
 * soc2_1x1_bus2chip_cr_mapping; MT6833 is SOC2_1X1), and nothing else: there is no PCIe remap. Its 0x700c is
 * CONN_HIF_ON_ADDR_REMAP1, which points the AP's connsys windows at 0x180a0000 and 0x180d0000
 * elsewhere, and the PCIe remap's 0x90000 + 512 KiB runs past the end of the window.
 */
static const struct mt76_connac_reg_map mt6833_reg_map[] = {
	{ 0x80000000, 0x02000, 0x01000 },	/* MCU_CFG */
	{ 0x50000000, 0x04000, 0x04000 },	/* CONN_HIF (PDMA) */
	{ 0x50002000, 0x05000, 0x01000 },	/* CONN_HIF (reserved) */
	{ 0x5000a000, 0x06000, 0x01000 },	/* CONN_HIF (DMASHDL) */
	{ 0x82060000, 0x08000, 0x04000 },	/* WF_UMAC_TOP (PLE) */
	{ 0x82068000, 0x0c000, 0x02000 },	/* WF_UMAC_TOP (PSE) */
	{ 0x8206c000, 0x0e000, 0x02000 },	/* WF_UMAC_TOP (PP) */
	{ 0x82070000, 0x10000, 0x10000 },	/* WF_PHY */
	{ 0x820f0000, 0x20000, 0x10000 },	/* WF_LMAC_TOP */
	{ 0x820e0000, 0x30000, 0x10000 },	/* WF_LMAC_TOP (WTBL) */
	{ 0x81000000, 0x50000, 0x10000 },	/* BTSYS_OFF */
	{ 0x80070000, 0x60000, 0x10000 },	/* GPSSYS */
	{ 0x40000000, 0x70000, 0x10000 },	/* WF_SYSRAM */
	{ 0x00300000, 0x80000, 0x10000 },	/* MCU_SYSRAM */
	{ 0x80010000, 0xa1000, 0x01000 },	/* CONN_MCU_DMA */
	{ 0x80030000, 0xa2000, 0x01000 },	/* CONN_MCU_BTIF0 */
	{ 0x81030000, 0xa3000, 0x01000 },	/* CONN_MCU_CFG_ON */
	{ 0x80050000, 0xa4000, 0x01000 },	/* CONN_UART_PTA */
	{ 0x81040000, 0xa5000, 0x01000 },	/* CONN_MCU_CIRQ */
	{ 0x81050000, 0xa6000, 0x01000 },	/* CONN_MCU_GPT */
	{ 0x81060000, 0xa7000, 0x01000 },	/* CONN_PTA */
	{ 0x81080000, 0xa8000, 0x01000 },	/* CONN_MCU_WDT */
	{ 0x81090000, 0xa9000, 0x01000 },	/* CONN_MCU_PDA */
	{ 0x810a0000, 0xaa000, 0x01000 },	/* CONN_RDD_AHB_WRAP0 */
	{ 0x810b0000, 0xab000, 0x01000 },	/* BTSYS_ON */
	{ 0x810c0000, 0xac000, 0x01000 },	/* CONN_RBIST_TOP */
	{ 0x810d0000, 0xad000, 0x01000 },	/* CONN_RDD_AHB_WRAP0 */
	{ 0x820d0000, 0xae000, 0x01000 },	/* WFSYS_ON */
	{ 0x60000000, 0xaf000, 0x01000 },	/* CONN_MCU_PDA */
	{ 0x80020000, 0xb0000, 0x10000 },	/* CONN_TOP_MISC_OFF */
	{ 0x81020000, 0xc0000, 0x10000 },	/* CONN_TOP_MISC_ON */
};

/* the offset of a register in the window: offsets below 1 MiB are taken as they are */
static bool mt7615_reg_addr(struct mt7615_dev *dev, u32 addr, u32 *offset)
{
	int i;

	if (addr < 0x100000) {
		*offset = addr;
		return true;
	}

	if (!is_mt6833(&dev->mt76)) {
		*offset = mt7615_reg_map(dev, addr);
		return true;
	}

	for (i = 0; i < ARRAY_SIZE(mt6833_reg_map); i++) {
		if (addr - mt6833_reg_map[i].phys < mt6833_reg_map[i].size) {
			*offset = mt6833_reg_map[i].maps + addr - mt6833_reg_map[i].phys;
			return true;
		}
	}

	dev_warn_ratelimited(dev->mt76.dev, "register 0x%08x is out of reach\n", addr);
	return false;
}

/*
 * DEBUG: MT6833 registers that remain reachable while the firmware owns the WMAC
 * (MT76_STATE_PM), as verified under LPCTL=0x00030001 in c2d-nm-2 and soc2_1x1ShowHifInfo():
 * - 0x07000..0x07fff: CONN_HIF_ON (LPCTL 0x7000, IRQ_ENA 0x7008, host debug/misc 0x7100..0x713c)
 * - 0xbc010..0xbc020: connsys AXI sleep-protect/GALS/mux/HIF-busy status
 * - 0xc1140..0xc1144: CONN_TOP_MISC_ON (CONN_ON_MISC and ON_HOST_DBG)
 * Everything else (PDMA 0x4000..0x6fff, UMAC/PHY/LMAC/WTBL, WFSYS_ON) is off limits without
 * driver own and may hang the AXI bus when WFSYS is in deep sleep.
 */
static bool mt6833_pm_reg_safe(u32 addr)
{
	return (addr >= 0x7000 && addr < 0x8000) ||
	       (addr >= 0xbc010 && addr <= 0xbc020) ||
	       (addr >= 0xc1140 && addr <= 0xc1144);
}

static bool mt6833_pm_reg_skip(struct mt7615_dev *dev, u32 offset, u32 addr,
			       bool write, u32 val, void *caller)
{
	int n;

	if (!is_mt6833(&dev->mt76) ||
	    !test_bit(MT76_STATE_PM, &dev->mphy.state) ||
	    (in_task() && READ_ONCE(dev->mt6833_pmctrl_task) == current) ||
	    mt6833_pm_reg_safe(addr))
		return false;

	n = atomic_inc_return(&dev->mt6833_pm_reg_skips);
	if (n <= 16)
		dev_warn(dev->mt76.dev,
			 "PM skip #%d: %s 0x%08x (win 0x%05x) in %s by %pS\n",
			 n, write ? "wr" : "rr", offset, addr,
			 in_hardirq() ? "hardirq" :
			 in_serving_softirq() ? "softirq" : "task", caller);
	if (in_hardirq())
		WARN_ONCE(1, "mt6833-wmac: hardirq %s 0x%08x (0x%05x) in firmware own\n",
			  write ? "wr" : "rr", offset, addr);
	else if (in_serving_softirq())
		WARN_ONCE(1, "mt6833-wmac: softirq %s 0x%08x (0x%05x) in firmware own\n",
			  write ? "wr" : "rr", offset, addr);
	else
		WARN_ONCE(1, "mt6833-wmac: task %s 0x%08x (0x%05x) in firmware own\n",
			  write ? "wr" : "rr", offset, addr);

	/*
	 * GIC SPI 377 is level-high: if mt7615_irq_handler() tried to mask MT_INT_MASK_CSR
	 * (0x4204) and we dropped the write, gate the line in CONN_HIF_ON_IRQ_ENA (0x7008) so
	 * the unmasked level interrupt cannot livelock the CPU before the trace gets out.
	 */
	if (in_hardirq())
		mt6833_irq_disable(dev);

	return true;
}

/*
 * MT6833: WFSYS (UMAC PLE/PSE/PP 0x08000..0x0ffff, PHY 0x10000..0x1ffff, LMAC/WTBL
 * 0x20000..0x3ffff and WFSYS_ON 0xae000..0xaefff) is powered down and AXI sleep-protected by
 * the firmware whenever it is idle; taking driver own through LPCTL wakes CONN_HIF (PDMA), not
 * WFSYS, so direct MMIO to WFSYS faults with a synchronous external abort. Reach WFSYS by chip
 * address through CMD_ID_ACCESS_REG (0xc0), whose MCU dispatch wakes WFSYS first (as MediaTek's
 * gen4m does for addresses >= 0x10000, and as mt7663s does).
 */
static bool mt6833_wfsys_chip_addr(u32 addr, u32 *chip)
{
	int i;

	if (!((addr >= 0x08000 && addr < 0x40000) ||
	      (addr >= 0xae000 && addr < 0xaf000)))
		return false;

	for (i = 0; i < ARRAY_SIZE(mt6833_reg_map); i++) {
		if (addr - mt6833_reg_map[i].maps < mt6833_reg_map[i].size) {
			*chip = mt6833_reg_map[i].phys +
				(addr - mt6833_reg_map[i].maps);
			return true;
		}
	}
	return false;
}

static bool mt6833_wfsys_can_mcu(struct mt7615_dev *dev)
{
	return !WARN_ON_ONCE(!test_bit(MT76_STATE_MCU_RUNNING, &dev->mphy.state) ||
			     in_atomic() || irqs_disabled());
}

static u32 mt7615_rr(struct mt76_dev *mdev, u32 offset)
{
	struct mt7615_dev *dev = container_of(mdev, struct mt7615_dev, mt76);
	u32 addr, chip;

	if (!mt7615_reg_addr(dev, offset, &addr))
		return ~0;
	if (mt6833_pm_reg_skip(dev, offset, addr, false, 0, (void *)_RET_IP_))
		return 0;
	if (is_mt6833(mdev) && mt6833_wfsys_chip_addr(addr, &chip))
		return mt6833_wfsys_can_mcu(dev) ?
		       mt76_connac_mcu_reg_rr(mdev, chip) : 0;

	return dev->bus_ops->rr(mdev, addr);
}

static void mt7615_wr(struct mt76_dev *mdev, u32 offset, u32 val)
{
	struct mt7615_dev *dev = container_of(mdev, struct mt7615_dev, mt76);
	u32 addr, chip;

	if (!mt7615_reg_addr(dev, offset, &addr))
		return;
	if (mt6833_pm_reg_skip(dev, offset, addr, true, val, (void *)_RET_IP_))
		return;
	if (is_mt6833(mdev) && mt6833_wfsys_chip_addr(addr, &chip)) {
		if (mt6833_wfsys_can_mcu(dev))
			mt76_connac_mcu_reg_wr(mdev, chip, val);
		return;
	}

	dev->bus_ops->wr(mdev, addr, val);
}

static u32 mt7615_rmw(struct mt76_dev *mdev, u32 offset, u32 mask, u32 val)
{
	struct mt7615_dev *dev = container_of(mdev, struct mt7615_dev, mt76);
	u32 addr, chip;

	if (!mt7615_reg_addr(dev, offset, &addr))
		return ~0;
	if (mt6833_pm_reg_skip(dev, offset, addr, true, val, (void *)_RET_IP_))
		return 0;
	if (is_mt6833(mdev) && mt6833_wfsys_chip_addr(addr, &chip)) {
		if (!mt6833_wfsys_can_mcu(dev))
			return 0;
		val |= mt76_connac_mcu_reg_rr(mdev, chip) & ~mask;
		mt76_connac_mcu_reg_wr(mdev, chip, val);
		return val;
	}

	return dev->bus_ops->rmw(mdev, addr, mask, val);
}

static void mt7615_write_copy(struct mt76_dev *mdev, u32 offset,
			      const void *data, int len)
{
	struct mt7615_dev *dev = container_of(mdev, struct mt7615_dev, mt76);
	u32 addr, chip;

	if (!mt7615_reg_addr(dev, offset, &addr))
		return;
	if (mt6833_pm_reg_skip(dev, offset, addr, true, 1, (void *)_RET_IP_))
		return;
	if (is_mt6833(mdev) && mt6833_wfsys_chip_addr(addr, &chip)) {
		int i;

		if (!mt6833_wfsys_can_mcu(dev))
			return;
		for (i = 0; i + 4 <= len; i += 4)
			mt76_connac_mcu_reg_wr(mdev, chip + i,
					       get_unaligned_le32(data + i));
		return;
	}

	dev->bus_ops->write_copy(mdev, addr, data, len);
}

static void mt7615_read_copy(struct mt76_dev *mdev, u32 offset,
			     void *data, int len)
{
	struct mt7615_dev *dev = container_of(mdev, struct mt7615_dev, mt76);
	u32 addr, chip;

	if (!mt7615_reg_addr(dev, offset, &addr))
		return;
	if (mt6833_pm_reg_skip(dev, offset, addr, false, 0, (void *)_RET_IP_)) {
		memset(data, 0, len);
		return;
	}
	if (is_mt6833(mdev) && mt6833_wfsys_chip_addr(addr, &chip)) {
		int i;

		if (!mt6833_wfsys_can_mcu(dev)) {
			memset(data, 0, len);
			return;
		}
		for (i = 0; i + 4 <= len; i += 4)
			put_unaligned_le32(mt76_connac_mcu_reg_rr(mdev, chip + i),
					   data + i);
		return;
	}

	dev->bus_ops->read_copy(mdev, addr, data, len);
}

/* rev: the chip id and revision, or 0 to read them from the chip */
int mt7615_mmio_probe(struct device *pdev, void __iomem *mem_base,
		      int irq, const u32 *map, u32 rev)
{
	static const struct mt76_driver_ops drv_ops = {
		/* txwi_size = txd size + txp size */
		.txwi_size = MT_TXD_SIZE + sizeof(struct mt76_connac_txp_common),
		.drv_flags = MT_DRV_TXWI_NO_FREE | MT_DRV_HW_MGMT_TXQ,
		.survey_flags = SURVEY_INFO_TIME_TX |
				SURVEY_INFO_TIME_RX |
				SURVEY_INFO_TIME_BSS_RX,
		.token_size = MT7615_TOKEN_SIZE,
		.tx_prepare_skb = mt7615_tx_prepare_skb,
		.tx_complete_skb = mt76_connac_tx_complete_skb,
		.rx_check = mt7615_rx_check,
		.rx_skb = mt7615_queue_rx_skb,
		.rx_poll_complete = mt7615_rx_poll_complete,
		.sta_add = mt7615_mac_sta_add,
		.sta_remove = mt7615_mac_sta_remove,
		.update_survey = mt7615_update_channel,
		.set_channel = mt7615_set_channel,
	};
	struct mt76_bus_ops *bus_ops;
	struct ieee80211_ops *ops;
	struct mt7615_dev *dev;
	struct mt76_dev *mdev;
	int ret;

	ops = devm_kmemdup(pdev, &mt7615_ops, sizeof(mt7615_ops), GFP_KERNEL);
	if (!ops)
		return -ENOMEM;

	mdev = mt76_alloc_device(pdev, sizeof(*dev), ops, &drv_ops);
	if (!mdev)
		return -ENOMEM;

	dev = container_of(mdev, struct mt7615_dev, mt76);
	mt76_mmio_init(&dev->mt76, mem_base);
	tasklet_setup(&mdev->irq_tasklet, mt7615_irq_tasklet);

	dev->reg_map = map;
	dev->ops = ops;
	dev->rxd_dw4 = -1;
	if (rev)
		mdev->rev = rev;
	else
		mdev->rev = (mt76_rr(dev, MT_HW_CHIPID) << 16) |
			    (mt76_rr(dev, MT_HW_REV) & 0xff);
	dev_dbg(mdev->dev, "ASIC revision: %04x\n", mdev->rev);

	dev->bus_ops = dev->mt76.bus;
	bus_ops = devm_kmemdup(dev->mt76.dev, dev->bus_ops, sizeof(*bus_ops),
			       GFP_KERNEL);
	if (!bus_ops) {
		ret = -ENOMEM;
		goto err_free_dev;
	}

	bus_ops->rr = mt7615_rr;
	bus_ops->wr = mt7615_wr;
	bus_ops->rmw = mt7615_rmw;
	bus_ops->write_copy = mt7615_write_copy;
	bus_ops->read_copy = mt7615_read_copy;
	dev->mt76.bus = bus_ops;

	mt76_wr(dev, MT_INT_MASK_CSR, 0);

	if (is_mt6833(mdev))
		dev->mt6833_irq = irq;
	ret = devm_request_irq(mdev->dev, irq, mt7615_irq_handler,
			       is_mt6833(mdev) ? 0 : IRQF_SHARED,
			       KBUILD_MODNAME, dev);
	if (ret)
		goto err_free_dev;

	if (is_mt6833(mdev))
		mt76_wr(dev, MT_CONN_HIF_ON_IRQ_ENA, MT_CONN_HIF_ON_IRQ_ENA_HOST);
	else if (is_mt7663(mdev))
		mt76_wr(dev, MT_PCIE_IRQ_ENABLE, 1);

	ret = mt7615_register_device(dev);
	if (ret)
		goto err_free_irq;

	return 0;

err_free_irq:
	devm_free_irq(pdev, irq, dev);
err_free_dev:
	mt76_free_device(&dev->mt76);

	return ret;
}

static int __init mt7615_init(void)
{
	int ret;

	ret = pci_register_driver(&mt7615_pci_driver);
	if (ret)
		return ret;

	if (IS_ENABLED(CONFIG_MT7622_WMAC)) {
		ret = platform_driver_register(&mt7622_wmac_driver);
		if (ret)
			goto err_pci;
	}

	if (IS_ENABLED(CONFIG_MT6833_WMAC)) {
		ret = platform_driver_register(&mt6833_wmac_driver);
		if (ret)
			goto err_mt7622;
	}

	return 0;

err_mt7622:
	if (IS_ENABLED(CONFIG_MT7622_WMAC))
		platform_driver_unregister(&mt7622_wmac_driver);
err_pci:
	pci_unregister_driver(&mt7615_pci_driver);
	return ret;
}

static void __exit mt7615_exit(void)
{
	if (IS_ENABLED(CONFIG_MT6833_WMAC))
		platform_driver_unregister(&mt6833_wmac_driver);
	if (IS_ENABLED(CONFIG_MT7622_WMAC))
		platform_driver_unregister(&mt7622_wmac_driver);
	pci_unregister_driver(&mt7615_pci_driver);
}

module_init(mt7615_init);
module_exit(mt7615_exit);
MODULE_DESCRIPTION("MediaTek MT7615E MMIO helpers");
MODULE_LICENSE("Dual BSD/GPL");
