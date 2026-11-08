/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek MT6833 connsys (CONNAC 1.x): what its function drivers need from the part that powers
 * it. Connsys is one subsystem shared by Bluetooth, Wi-Fi, GPS and FM: one CPU, one reserved
 * memory region (EMI), one control channel (WMT over BTIF). Powering it on, copying the ROM
 * patches into EMI and the WMT bring-up with its RF calibration are done once, for all of them.
 *
 * A prototype interface: the Bluetooth driver (btmtkconn) provides it for now.
 */
#ifndef __MTK_CONNSYS_H__
#define __MTK_CONNSYS_H__

#include <linux/err.h>
#include <linux/types.h>

struct device_node;
struct mtk_connsys;

#if IS_ENABLED(CONFIG_BT_MTKCONN)
/* the connsys of a "mediatek,connsys" phandle; ERR_PTR(-EPROBE_DEFER) until it is there */
struct mtk_connsys *mtk_connsys_get(struct device_node *np);
/* powered and calibrated on return (once per boot; may take two seconds), counted */
int mtk_connsys_power_on(struct mtk_connsys *cs);
void mtk_connsys_power_off(struct mtk_connsys *cs);
/* write into the reserved memory the connsys CPU sees at 0xf0000000 + offset */
int mtk_connsys_emi_write(struct mtk_connsys *cs, u32 offset, const void *data, size_t len);
#else
static inline struct mtk_connsys *mtk_connsys_get(struct device_node *np)
{
	return ERR_PTR(-ENODEV);
}

static inline int mtk_connsys_power_on(struct mtk_connsys *cs)
{
	return -ENODEV;
}

static inline void mtk_connsys_power_off(struct mtk_connsys *cs)
{
}

static inline int mtk_connsys_emi_write(struct mtk_connsys *cs, u32 offset, const void *data,
					size_t len)
{
	return -ENODEV;
}
#endif

#endif
