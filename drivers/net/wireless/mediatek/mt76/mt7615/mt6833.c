// SPDX-License-Identifier: BSD-3-Clause-Clear
/*
 * The Wi-Fi of MediaTek MT6833's connsys (CONNAC 1.x, A-die MT6631).
 *
 * The WMAC is an MT7663 core (1x1) behind a 1 MiB window on the SoC's bus, with the same
 * register map, PDMA rings and MCU protocol. What differs:
 * - It lives in connsys, which Bluetooth, GPS and FM share: the connsys driver powers it on,
 *   copies the ROM patches (Wi-Fi's included) into connsys' reserved memory and runs the WMT
 *   bring-up with the RF calibration. Wi-Fi itself sends nothing over WMT.
 * - No PCIe: no doorbell, no PCIe interrupt enable, no HIF2.
 * - Two of the RAM firmware's regions go straight into connsys' reserved memory (EMI).
 * - No efuse: the calibration is the board's NVRAM file (nvdata's WIFI), which the firmware
 *   takes whole in one command; the MAC address is in it.
 * - Receive descriptors carry a fifth word.
 */

#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/soc/mediatek/mtk-connsys.h>
#include "mt7615.h"

static struct mtk_connsys *mt6833_connsys(struct device *dev)
{
	struct device_node *np;
	struct mtk_connsys *cs;

	np = of_parse_phandle(dev->of_node, "mediatek,connsys", 0);
	if (!np)
		return ERR_PTR(-EINVAL);
	cs = mtk_connsys_get(np);
	of_node_put(np);
	return cs;
}

/* from mt7615_init_hardware(), after the EEPROM defaults: the board's NVRAM and MAC address */
int mt6833_wmac_init(struct mt7615_dev *dev)
{
	struct device *pdev = dev->mt76.dev;
	const struct firmware *fw;
	int ret;

	if (!is_mt6833(&dev->mt76))
		return 0;

	ret = request_firmware(&fw, MT6833_NVRAM, pdev);
	if (ret) {
		dev_warn(pdev, "no %s (%d): uncalibrated, random MAC address\n",
			 MT6833_NVRAM, ret);
		return 0;
	}
	if (fw->size < MT6833_NVRAM_SIZE) {
		dev_warn(pdev, "%s: %zu bytes, expected %d\n", MT6833_NVRAM, fw->size,
			 MT6833_NVRAM_SIZE);
		release_firmware(fw);
		return 0;
	}
	dev->nvram = devm_kmemdup(pdev, fw->data, MT6833_NVRAM_SIZE, GFP_KERNEL);
	release_firmware(fw);
	if (!dev->nvram)
		return -ENOMEM;

	/* WIFI_CFG_PARAM_STRUCT: two version words, then the MAC address */
	if (is_valid_ether_addr(dev->nvram + 4))
		ether_addr_copy(dev->mphy.macaddr, dev->nvram + 4);
	return 0;
}

static int mt6833_wmac_probe(struct platform_device *pdev)
{
	struct mtk_connsys *cs;
	void __iomem *mem_base;
	int irq, ret;

	cs = mt6833_connsys(&pdev->dev);
	if (IS_ERR(cs))
		return dev_err_probe(&pdev->dev, PTR_ERR(cs), "connsys\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	mem_base = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR(mem_base))
		return PTR_ERR(mem_base);

	/* buffer addresses in the descriptors are 32 bits wide */
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	/* nothing in the window answers before connsys is up */
	ret = mtk_connsys_power_on(cs);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "connsys power-on\n");

	ret = mt7615_mmio_probe(&pdev->dev, mem_base, irq, mt7663e_reg_map, 0x6833 << 16);
	if (ret)
		mtk_connsys_power_off(cs);
	return ret;
}

static void mt6833_wmac_remove(struct platform_device *pdev)
{
	struct mt7615_dev *dev = platform_get_drvdata(pdev);
	struct mtk_connsys *cs = mt6833_connsys(&pdev->dev);

	mt7615_unregister_device(dev);
	if (!IS_ERR(cs))
		mtk_connsys_power_off(cs);
}

static const struct of_device_id mt6833_wmac_of_match[] = {
	{ .compatible = "mediatek,mt6833-wmac" },
	{},
};
MODULE_DEVICE_TABLE(of, mt6833_wmac_of_match);

struct platform_driver mt6833_wmac_driver = {
	.driver = {
		.name = "mt6833-wmac",
		.of_match_table = mt6833_wmac_of_match,
	},
	.probe = mt6833_wmac_probe,
	.remove = mt6833_wmac_remove,
};

MODULE_FIRMWARE(MT6833_FIRMWARE_N9);
