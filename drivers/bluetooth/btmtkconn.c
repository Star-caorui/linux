// SPDX-License-Identifier: GPL-2.0-only
/*
 * Bluetooth on MediaTek MT6833's connsys (CONNAC 1.x, A-die MT6631), over BTIF. A prototype.
 *
 * BTIF is an 8250-style port (8250_of, "mediatek,mtk-btif"); this is a serdev client on it, like
 * btmtkuart is for MT7622. Opening the HCI device the first time powers connsys on as MediaTek's
 * WMT driver does (common_main platform/mt6833.c), copies the three ROM patches into connsys'
 * reserved memory, lets its CPU run and replays the stock WMT bring-up (RF calibration included)
 * before switching Bluetooth on. Everything goes in STP "mand" frames (80, type << 4 | len_hi,
 * len_lo, 00, payload, 00 00): WMT is type 4, HCI (H:4) type 0, firmware log/assert text type 5.
 * Connsys stays powered once up (no power-off, no PSM): closing only switches Bluetooth off.
 *
 * Until connsys has a driver of its own, this one also provides <linux/soc/mediatek/mtk-connsys.h>
 * for the Wi-Fi driver: power-on is shared, and Wi-Fi copies part of its firmware into the same
 * reserved memory.
 */
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/serdev.h>
#include <linux/soc/mediatek/mtk-connsys.h>
#include <linux/skbuff.h>
#include <linux/unaligned.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "hci_uart.h"

/* the connsys node's reg entries */
enum {
	R_MCU,		/* 0x18002000 CONN_MCU_CONFIG */
	R_RGU,		/* 0x10007000 TOPRGU */
	R_INFRA,	/* 0x10001000 INFRACFG_AO */
	R_SPM,		/* 0x10006000 SPM */
	R_MISC_OFF,	/* 0x180b1000 CONN_TOP_MISC_OFF */
	R_MISC_ON,	/* 0x180c1000 CONN_TOP_MISC_ON */
	R_PERI,		/* 0x10003000 INFRA_AO_PERICFG */
	R_GPIO,		/* 0x10005000 GPIO */
	R_DRV,		/* 0x11ea0000 pad driving */
	R_BTIF,		/* 0x1100c000 BTIF */
	R_RC,		/* 0x1000f800 SRCLKEN_RC */
	R_HIF_ON,	/* 0x18007000 CONN_HIF_ON */
	R_NUM
};

#define RGU_SWSYSRST		0x18
#define RGU_KEY			(0x88u << 24)
#define RGU_CONN_MCU		BIT(12)

#define BTIF_FIFOCTRL		0x08
#define BTIF_DMA_EN		0x4c
#define BTIF_TRI_LVL		0x60
#define BTIF_HANDSHAKE		0x6c

#define STP_BT			0
#define STP_WMT			4
#define STP_LOG			5
#define STP_MAX			2048

#define PATCH_HDR		48

static const struct {
	const char *name;
	u32 offset;
} patches[] = {
	{ "soc2_2_ram_bt_1_1_hdr.bin", 0x170000 },
	{ "soc2_2_ram_wifi_1_1_hdr.bin", 0x2a0000 },
	{ "soc2_2_ram_mcu_1_1_hdr.bin", 0x000000 },
};

/* the stock bring-up after STP is up (WMT_SOC.cfg of gold: co-clock, MT6631); then BT on */
struct wmt_step {
	u8 len;
	u8 cmd[12];
	u16 timeout_ms;
};

static const struct wmt_step wmt_init[] = {
	{ 5, { 0x01, 0x04, 0x01, 0x00, 0x04 }, 500 },				/* query STP */
	{ 9, { 0x01, 0x02, 0x05, 0x00, 0x11, 0x33, 0x68, 0x00, 0x00 }, 500 },	/* chip id */
	{ 5, { 0x01, 0x02, 0x01, 0x00, 0x12 }, 500 },				/* A-die */
	{ 6, { 0x01, 0x10, 0x02, 0x00, 0x15, 0x01 }, 500 },			/* coex */
	{ 8, { 0x01, 0x02, 0x04, 0x00, 0x10, 0x03, 0xc0, 0x0e }, 500 },
	{ 8, { 0x01, 0x14, 0x04, 0x00, 0x07, 0x02, 0x00, 0x03 }, 500 },
	{ 5, { 0x01, 0x14, 0x01, 0x00, 0x01 }, 4000 },				/* RF calibration */
	{ 5, { 0x01, 0x14, 0x01, 0x00, 0x03 }, 1000 },
	{ 8, { 0x01, 0x10, 0x04, 0x00, 0x1e, 0x00, 0x00, 0x00 }, 500 },
	{ 8, { 0x01, 0x10, 0x04, 0x00, 0x1f, 0x00, 0x00, 0x00 }, 500 },
	{ 8, { 0x01, 0x10, 0x04, 0x00, 0x20, 0x00, 0x00, 0x00 }, 500 },
	{ 6, { 0x01, 0x05, 0x02, 0x00, 0x02, 0x02 }, 500 },			/* FM strap */
};

static const u8 wmt_bt_on[] = { 0x01, 0x06, 0x02, 0x00, 0x00, 0x01 };
static const u8 wmt_bt_off[] = { 0x01, 0x06, 0x02, 0x00, 0x00, 0x00 };

struct mtk_connsys {
	struct list_head node;
	struct device_node *np;		/* the "mediatek,connsys" node */
};

struct btmtkconn {
	struct mtk_connsys cs;
	struct serdev_device *serdev;
	struct hci_dev *hdev;
	struct hci_uart hu;		/* only for h4_recv_buf() */
	void __iomem *r[R_NUM];
	struct clk *ccif;
	struct regulator *vcn18, *vcn33_1, *vcn33_2;
	phys_addr_t emi_phys;
	size_t emi_size;
	struct mutex power_lock;	/* powered, failed, users */
	bool powered;
	bool failed;
	unsigned int users;		/* other connsys functions (Wi-Fi) */

	/* STP receive */
	u8 hdr[4];
	unsigned int hdr_len, type, len, got, tail;
	unsigned int log_cnt;		/* STP_LOG packets */
	u8 *buf;
	struct sk_buff *rx_skb;

	/* WMT */
	struct mutex wmt_lock;
	struct completion wmt_done;
	u8 wmt_evt[16];
	unsigned int wmt_evt_len;

	/* transmit */
	struct mutex tx_lock;
	struct sk_buff_head txq;
	struct work_struct tx_work;
};

static inline u32 rd(struct btmtkconn *bc, int r, u32 off)
{
	return readl(bc->r[r] + off);
}

static inline void wr(struct btmtkconn *bc, int r, u32 off, u32 v)
{
	writel(v, bc->r[r] + off);
}

static inline void rmw(struct btmtkconn *bc, int r, u32 off, u32 clr, u32 set)
{
	wr(bc, r, off, (rd(bc, r, off) & ~clr) | set);
}

static int stp_write(struct btmtkconn *bc, unsigned int type, const u8 *data, size_t len)
{
	u8 *frame;
	int ret;

	if (len > STP_MAX)
		return -EINVAL;
	frame = kmalloc(len + 6, GFP_KERNEL);
	if (!frame)
		return -ENOMEM;
	frame[0] = 0x80;
	frame[1] = (type << 4) | (len >> 8);
	frame[2] = len & 0xff;
	frame[3] = 0;
	memcpy(frame + 4, data, len);
	frame[len + 4] = 0;
	frame[len + 5] = 0;

	mutex_lock(&bc->tx_lock);
	ret = serdev_device_write(bc->serdev, frame, len + 6, msecs_to_jiffies(1000));
	mutex_unlock(&bc->tx_lock);
	kfree(frame);
	if (ret < 0)
		return ret;
	if (ret != len + 6)
		return -ETIMEDOUT;
	bc->hdev->stat.byte_tx += len + 6;
	return 0;
}

static int wmt_cmd(struct btmtkconn *bc, const u8 *cmd, size_t len, unsigned int timeout_ms)
{
	int ret;

	mutex_lock(&bc->wmt_lock);
	reinit_completion(&bc->wmt_done);
	bc->wmt_evt_len = 0;
	ret = stp_write(bc, STP_WMT, cmd, len);
	if (ret)
		goto out;
	if (!wait_for_completion_timeout(&bc->wmt_done, msecs_to_jiffies(timeout_ms))) {
		bt_dev_err(bc->hdev, "WMT %*ph: no answer", (int)min_t(size_t, len, 8), cmd);
		ret = -ETIMEDOUT;
		goto out;
	}
	if (bc->wmt_evt_len < 5 || bc->wmt_evt[0] != 0x02 || bc->wmt_evt[1] != cmd[1] ||
	    bc->wmt_evt[4]) {
		bt_dev_err(bc->hdev, "WMT %*ph: answer %*ph", (int)min_t(size_t, len, 8), cmd,
			   bc->wmt_evt_len, bc->wmt_evt);
		ret = -EIO;
	}
out:
	mutex_unlock(&bc->wmt_lock);
	return ret;
}

static int recv_event(struct hci_dev *hdev, struct sk_buff *skb)
{
	return hci_recv_frame(hdev, skb);
}

static const struct h4_recv_pkt recv_pkts[] = {
	{ H4_RECV_ACL,   .recv = hci_recv_frame },
	{ H4_RECV_SCO,   .recv = hci_recv_frame },
	{ H4_RECV_EVENT, .recv = recv_event },
	{ H4_RECV_ISO,   .recv = hci_recv_frame },
};

static void stp_frame(struct btmtkconn *bc)
{
	switch (bc->type) {
	case STP_BT:
		bc->rx_skb = h4_recv_buf(&bc->hu, bc->rx_skb, bc->buf, bc->len, recv_pkts,
					 ARRAY_SIZE(recv_pkts));
		if (IS_ERR(bc->rx_skb)) {
			bt_dev_err(bc->hdev, "H:4 reassembly failed (%ld)", PTR_ERR(bc->rx_skb));
			bc->rx_skb = NULL;
		}
		break;
	case STP_WMT:
		if (bc->len >= 2 && bc->buf[0] == 0x02 && bc->buf[1] == 0x03) {
			/* sleep/wake-up events: only with PSM, which is not on */
			bt_dev_dbg(bc->hdev, "WMT %*ph", bc->len, bc->buf);
			break;
		}
		bc->wmt_evt_len = min_t(unsigned int, bc->len, sizeof(bc->wmt_evt));
		memcpy(bc->wmt_evt, bc->buf, bc->wmt_evt_len);
		complete(&bc->wmt_done);
		break;
	case STP_LOG:
		/* an assert message, then the core dump: the first ones to the log */
		if (bc->log_cnt++ < 32)
			bt_dev_err(bc->hdev, "firmware (%u): %*pE", bc->log_cnt,
				   min_t(int, bc->len, 512), bc->buf);
		else if (!(bc->log_cnt % 256))
			bt_dev_err(bc->hdev, "firmware: %u messages", bc->log_cnt);
		break;
	default:
		bt_dev_dbg(bc->hdev, "STP type %u, %u bytes", bc->type, bc->len);
	}
}

static size_t btmtkconn_receive_buf(struct serdev_device *serdev, const u8 *data, size_t count)
{
	struct btmtkconn *bc = serdev_device_get_drvdata(serdev);
	size_t left = count;

	bc->hdev->stat.byte_rx += count;
	while (left) {
		unsigned int n;

		if (bc->tail) {
			n = min_t(size_t, left, bc->tail);
			bc->tail -= n;
			data += n;
			left -= n;
			continue;
		}
		if (bc->hdr_len < 4) {
			u8 c = *data++;

			left--;
			if (!bc->hdr_len && c != 0x80)
				continue;	/* resync on a header */
			bc->hdr[bc->hdr_len++] = c;
			if (bc->hdr_len < 4)
				continue;
			bc->type = bc->hdr[1] >> 4;
			bc->len = ((bc->hdr[1] & 0xf) << 8) | bc->hdr[2];
			bc->got = 0;
			/*
			 * the fourth byte is 0, or a checksum: the firmware's assert messages
			 * (STP_LOG) come with one
			 */
			if ((bc->hdr[3] && bc->hdr[3] != (u8)(bc->hdr[0] + bc->hdr[1] + bc->hdr[2])) ||
			    bc->len > STP_MAX) {
				bt_dev_err(bc->hdev, "bad STP header %*ph", 4, bc->hdr);
				bc->hdr_len = 0;
			} else if (!bc->len) {
				bc->hdr_len = 0;
				bc->tail = 2;
			}
			continue;
		}
		n = min_t(size_t, left, bc->len - bc->got);
		memcpy(bc->buf + bc->got, data, n);
		bc->got += n;
		data += n;
		left -= n;
		if (bc->got == bc->len) {
			stp_frame(bc);
			bc->hdr_len = 0;
			bc->tail = 2;
		}
	}
	return count;
}

static const struct serdev_device_ops btmtkconn_client_ops = {
	.receive_buf = btmtkconn_receive_buf,
	.write_wakeup = serdev_device_write_wakeup,
};

static void release_patches(const struct firmware **fw)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(patches); i++) {
		release_firmware(fw[i]);
		fw[i] = NULL;
	}
}

/* all of them before connsys is touched: early opens come before the root file system */
static int load_patches(struct btmtkconn *bc, const struct firmware **fw)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(patches); i++) {
		ret = firmware_request_nowarn(&fw[i], patches[i].name, &bc->serdev->dev);
		if (!ret && (fw[i]->size <= PATCH_HDR ||
			     patches[i].offset + fw[i]->size - PATCH_HDR > bc->emi_size))
			ret = -EINVAL;
		if (ret) {
			release_patches(fw);
			return ret;
		}
	}
	return 0;
}

static int copy_patches(struct btmtkconn *bc, const struct firmware **fw)
{
	void *emi;
	int i;

	emi = memremap(bc->emi_phys, bc->emi_size, MEMREMAP_WC);
	if (!emi)
		return -ENOMEM;
	memset(emi, 0, bc->emi_size);
	for (i = 0; i < ARRAY_SIZE(patches); i++)
		memcpy(emi + patches[i].offset, fw[i]->data + PATCH_HDR, fw[i]->size - PATCH_HDR);
	wmb();
	memunmap(emi);
	return 0;
}

static void xo_osc_ctrl(struct btmtkconn *bc)
{
	const u32 b = 0x200;	/* 0x180c1200 */
	int i;

	if (!(rd(bc, R_RC, 0x4) & BIT(0))) {
		/* legacy (non-RC) mode, co-clock */
		rmw(bc, R_MISC_ON, b + 0x40, BIT(7), 0);
		rmw(bc, R_MISC_ON, b + 0x0, 0x0000ffff, 0x708);
		rmw(bc, R_MISC_ON, b + 0x4, 0x0000ff00 | BIT(0), 0x600);
		return;
	}
	/* RC mode, co-clock: GPS, BT, Wi-Fi, top */
	for (i = 0; i < 4; i++) {
		wr(bc, R_MISC_ON, b + 0x44 + 8 * i, 0x02080706);
		rmw(bc, R_MISC_ON, b + 0x48 + 8 * i, BIT(16), 0);
	}
	for (i = 0; i < 4; i++)
		rmw(bc, R_MISC_ON, b + 0x40, 0, BIT(4 + i) | BIT(12 + i));
	rmw(bc, R_MISC_ON, b + 0x40, BIT(8), 0);
}

static int connsys_power_on(struct btmtkconn *bc, const struct firmware **fw)
{
	struct device *dev = &bc->serdev->dev;
	u32 v;
	int i, ret;

	/* EMI remapping: connsys sees its memory at 0xf0000000, PERICFG at 0x10003000 */
	rmw(bc, R_INFRA, 0x380, GENMASK(19, 0), bc->emi_phys >> 16);
	rmw(bc, R_INFRA, 0x388, GENMASK(19, 0), 0x10003000 >> 16);

	ret = regulator_set_voltage(bc->vcn18, 1800000, 1800000) ?: regulator_enable(bc->vcn18);
	if (ret)
		return ret;
	ret = regulator_set_voltage(bc->vcn33_1, 3300000, 3300000) ?:
	      regulator_enable(bc->vcn33_1);
	if (ret)
		return ret;

	/* A-die interface pins, mode 1, and their driving */
	rmw(bc, R_GPIO, 0x480, ~0x88888888u, 0x11111111);
	rmw(bc, R_GPIO, 0x490, 0x7, 0x1);
	rmw(bc, R_DRV, 0x0, ~0xc0007fffu, 0x09048000);
	rmw(bc, R_DRV, 0x10, ~0xfffff000u, 0);
	udelay(150);

	ret = regulator_set_voltage(bc->vcn33_2, 2800000, 2800000) ?:
	      regulator_enable(bc->vcn33_2);
	if (ret)
		return ret;

	wr(bc, R_RGU, RGU_SWSYSRST, rd(bc, R_RGU, RGU_SWSYSRST) | RGU_CONN_MCU | RGU_KEY);
	wr(bc, R_SPM, 0x0, rd(bc, R_SPM, 0x0) | BIT(0) | (0x0b16u << 16));

	ret = clk_prepare_enable(bc->ccif);
	if (ret)
		return ret;
	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return ret;
	wr(bc, R_PERI, 0x314, 1);	/* CON_STA: CON_PWR_ON */
	mdelay(5);

	for (i = 0; i < 10; i++) {
		v = rd(bc, R_MISC_OFF, 0x10);
		if (v == 0x100a0000)
			break;
		msleep(20);
	}
	if (v != 0x100a0000) {
		dev_err(dev, "connsys IP version 0x%08x\n", v);
		return -ETIMEDOUT;
	}

	ret = copy_patches(bc, fw);
	if (ret) {
		dev_err(dev, "ROM patches: %d\n", ret);
		return ret;
	}

	rmw(bc, R_MISC_ON, 0x168, 0, BIT(0));	/* conn2ap EMI access in HW mode */
	wr(bc, R_MCU, 0x504, 0xf0170000);	/* BT entry */
	wr(bc, R_MCU, 0x508, 0xf02a0000);	/* Wi-Fi entry */
	xo_osc_ctrl(bc);
	rmw(bc, R_MISC_ON, 0x130, 0, BIT(8));	/* A-die is MT6631 */
	wr(bc, R_MCU, 0x440, 0x90000002);	/* AHB bus timeout */

	wr(bc, R_RGU, RGU_SWSYSRST, (rd(bc, R_RGU, RGU_SWSYSRST) & ~RGU_CONN_MCU) | RGU_KEY);
	for (i = 0; i < 12; i++) {
		v = rd(bc, R_MCU, 0x600);
		if (v == 0x1d1e)
			break;
		msleep(20);
	}
	if (v != 0x1d1e) {
		dev_err(dev, "connsys CPU not idle: 0x%x, pc 0x%x\n", v, rd(bc, R_HIF_ON, 0x104));
		return -ETIMEDOUT;
	}
	msleep(20);

	/* BTIF as the stock driver leaves it: new handshake, thresholds, FIFOs cleared */
	rmw(bc, R_BTIF, BTIF_HANDSHAKE, 0, BIT(0));
	wr(bc, R_BTIF, BTIF_TRI_LVL, 8 | (1 << 4));
	wr(bc, R_BTIF, BTIF_DMA_EN, BIT(2));
	rmw(bc, R_BTIF, BTIF_FIFOCTRL, 0, BIT(1) | BIT(2));
	bc->hdr_len = 0;
	bc->tail = 0;
	return 0;
}

/* with power_lock held */
static int connsys_up(struct btmtkconn *bc)
{
	const struct firmware *fw[ARRAY_SIZE(patches)] = { };
	struct hci_dev *hdev = bc->hdev;
	int i, ret;

	if (bc->failed)
		return -EIO;	/* power-on is tried once per boot */
	if (bc->powered)
		return 0;

	ret = load_patches(bc, fw);
	if (ret) {
		bt_dev_warn(hdev, "ROM patches not available (%d): not powering on yet", ret);
		return ret;
	}
	bc->powered = true;
	ret = connsys_power_on(bc, fw);
	release_patches(fw);
	if (!ret)
		for (i = 0; i < ARRAY_SIZE(wmt_init) && !ret; i++)
			ret = wmt_cmd(bc, wmt_init[i].cmd, wmt_init[i].len, wmt_init[i].timeout_ms);
	if (ret) {
		bc->failed = true;
		bt_dev_err(hdev, "connsys bring-up failed: %d", ret);
		return ret;
	}
	bt_dev_info(hdev, "connsys up");
	return 0;
}

static LIST_HEAD(connsys_list);
static DEFINE_MUTEX(connsys_list_lock);

struct mtk_connsys *mtk_connsys_get(struct device_node *np)
{
	struct mtk_connsys *cs, *found = ERR_PTR(-EPROBE_DEFER);

	mutex_lock(&connsys_list_lock);
	list_for_each_entry(cs, &connsys_list, node)
		if (cs->np == np) {
			found = cs;
			break;
		}
	mutex_unlock(&connsys_list_lock);
	return found;
}
EXPORT_SYMBOL_GPL(mtk_connsys_get);

int mtk_connsys_power_on(struct mtk_connsys *cs)
{
	struct btmtkconn *bc = container_of(cs, struct btmtkconn, cs);
	int ret;

	mutex_lock(&bc->power_lock);
	ret = connsys_up(bc);
	if (!ret)
		bc->users++;
	mutex_unlock(&bc->power_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_connsys_power_on);

/* connsys has no power-off yet: it stays up */
void mtk_connsys_power_off(struct mtk_connsys *cs)
{
	struct btmtkconn *bc = container_of(cs, struct btmtkconn, cs);

	mutex_lock(&bc->power_lock);
	if (!WARN_ON(!bc->users))
		bc->users--;
	mutex_unlock(&bc->power_lock);
}
EXPORT_SYMBOL_GPL(mtk_connsys_power_off);

int mtk_connsys_emi_write(struct mtk_connsys *cs, u32 offset, const void *data, size_t len)
{
	struct btmtkconn *bc = container_of(cs, struct btmtkconn, cs);
	void *emi;

	if (offset > bc->emi_size || len > bc->emi_size - offset)
		return -EINVAL;
	emi = memremap(bc->emi_phys + offset, len, MEMREMAP_WC);
	if (!emi)
		return -ENOMEM;
	memcpy(emi, data, len);
	wmb();
	memunmap(emi);
	return 0;
}
EXPORT_SYMBOL_GPL(mtk_connsys_emi_write);

static int btmtkconn_open(struct hci_dev *hdev)
{
	struct btmtkconn *bc = hci_get_drvdata(hdev);
	int ret;

	mutex_lock(&bc->power_lock);
	ret = connsys_up(bc);
	mutex_unlock(&bc->power_lock);
	if (ret)
		return ret;
	return wmt_cmd(bc, wmt_bt_on, sizeof(wmt_bt_on), 2000);
}

static int btmtkconn_close(struct hci_dev *hdev)
{
	struct btmtkconn *bc = hci_get_drvdata(hdev);

	cancel_work_sync(&bc->tx_work);
	skb_queue_purge(&bc->txq);
	kfree_skb(bc->rx_skb);
	bc->rx_skb = NULL;
	mutex_lock(&bc->power_lock);
	if (bc->powered && !bc->failed)
		wmt_cmd(bc, wmt_bt_off, sizeof(wmt_bt_off), 2000);
	mutex_unlock(&bc->power_lock);
	return 0;
}

static int btmtkconn_flush(struct hci_dev *hdev)
{
	struct btmtkconn *bc = hci_get_drvdata(hdev);

	skb_queue_purge(&bc->txq);
	return 0;
}

static void btmtkconn_tx_work(struct work_struct *work)
{
	struct btmtkconn *bc = container_of(work, struct btmtkconn, tx_work);
	struct sk_buff *skb;

	while ((skb = skb_dequeue(&bc->txq))) {
		int ret = stp_write(bc, STP_BT, skb->data, skb->len);

		if (ret)
			bt_dev_err(bc->hdev, "send: %d", ret);
		kfree_skb(skb);
	}
}

/*
 * The firmware does not know Read Synchronization Train Parameters although its LMP features say
 * it supports the train: answer it with the Core spec's defaults.
 */
static bool answer_read_sync_train(struct hci_dev *hdev, struct sk_buff *skb)
{
	static const u8 evt[] = { 0x0e, 0x0b, 0x01, 0x77, 0x0c, 0x00, 0x80, 0x00,
				  0x00, 0xee, 0x02, 0x00, 0x00 };
	struct sk_buff *e;

	if (hci_skb_pkt_type(skb) != HCI_COMMAND_PKT || skb->len < 2 ||
	    get_unaligned_le16(skb->data) != HCI_OP_READ_SYNC_TRAIN_PARAMS)
		return false;
	e = bt_skb_alloc(sizeof(evt), GFP_KERNEL);
	if (!e)
		return false;
	skb_put_data(e, evt, sizeof(evt));
	hci_skb_pkt_type(e) = HCI_EVENT_PKT;
	hci_recv_frame(hdev, e);
	kfree_skb(skb);
	return true;
}

static int btmtkconn_send(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct btmtkconn *bc = hci_get_drvdata(hdev);

	if (answer_read_sync_train(hdev, skb))
		return 0;
	memcpy(skb_push(skb, 1), &hci_skb_pkt_type(skb), 1);
	skb_queue_tail(&bc->txq, skb);
	schedule_work(&bc->tx_work);
	return 0;
}

static int btmtkconn_set_bdaddr(struct hci_dev *hdev, const bdaddr_t *bdaddr)
{
	struct sk_buff *skb;

	skb = __hci_cmd_sync(hdev, 0xfc1a, sizeof(*bdaddr), bdaddr, HCI_INIT_TIMEOUT);
	if (IS_ERR(skb))
		return PTR_ERR(skb);
	kfree_skb(skb);
	return 0;
}

static void put_connsys_node(void *np)
{
	of_node_put(np);
}

/*
 * DEBUG: connsys' reserved memory, read-only, for post-mortems: the RAM code and data the connsys
 * CPUs run from, their state after an exception, the core dump area.
 */
static ssize_t btmtkconn_emi_read(struct file *file, char __user *ubuf, size_t count,
				  loff_t *ppos)
{
	struct btmtkconn *bc = file->private_data;
	size_t len;
	void *emi;

	if (*ppos < 0 || *ppos >= bc->emi_size)
		return 0;
	len = min_t(size_t, count, bc->emi_size - *ppos);
	len = min_t(size_t, len, SZ_1M);
	emi = memremap(bc->emi_phys + *ppos, len, MEMREMAP_WC);
	if (!emi)
		return -ENOMEM;
	if (copy_to_user(ubuf, emi, len)) {
		memunmap(emi);
		return -EFAULT;
	}
	memunmap(emi);
	*ppos += len;
	return len;
}

static const struct file_operations btmtkconn_emi_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = btmtkconn_emi_read,
	.llseek = default_llseek,
};

static void btmtkconn_debugfs(struct btmtkconn *bc)
{
	struct dentry *d = debugfs_create_dir("btmtkconn", NULL);

	debugfs_create_file_size("emi", 0400, d, bc, &btmtkconn_emi_fops, bc->emi_size);
}

static int btmtkconn_probe(struct serdev_device *serdev)
{
	struct device *dev = &serdev->dev;
	struct device_node *np;
	struct reserved_mem *rmem;
	struct btmtkconn *bc;
	struct hci_dev *hdev;
	int i, ret;

	bc = devm_kzalloc(dev, sizeof(*bc), GFP_KERNEL);
	if (!bc)
		return -ENOMEM;
	bc->buf = devm_kmalloc(dev, STP_MAX, GFP_KERNEL);
	if (!bc->buf)
		return -ENOMEM;
	bc->serdev = serdev;

	np = of_parse_phandle(dev->of_node, "mediatek,connsys", 0);
	if (!np)
		return dev_err_probe(dev, -EINVAL, "no mediatek,connsys\n");
	for (i = 0; i < R_NUM; i++) {
		struct resource res;

		/* not requested: SPM, GPIO, INFRACFG and BTIF have their own drivers */
		ret = of_address_to_resource(np, i, &res);
		if (!ret) {
			bc->r[i] = devm_ioremap(dev, res.start, resource_size(&res));
			if (!bc->r[i])
				ret = -ENOMEM;
		}
		if (ret) {
			of_node_put(np);
			return dev_err_probe(dev, ret, "connsys reg %d\n", i);
		}
	}
	bc->cs.np = np;
	ret = devm_add_action_or_reset(dev, put_connsys_node, np);
	if (ret)
		return ret;

	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	rmem = np ? of_reserved_mem_lookup(np) : NULL;
	of_node_put(np);
	if (!rmem)
		return dev_err_probe(dev, -EINVAL, "no memory-region\n");
	bc->emi_phys = rmem->base;
	bc->emi_size = rmem->size;

	bc->ccif = devm_clk_get(dev, "ccif");
	if (IS_ERR(bc->ccif))
		return dev_err_probe(dev, PTR_ERR(bc->ccif), "ccif clock\n");
	bc->vcn18 = devm_regulator_get(dev, "vcn18");
	if (IS_ERR(bc->vcn18))
		return dev_err_probe(dev, PTR_ERR(bc->vcn18), "vcn18\n");
	bc->vcn33_1 = devm_regulator_get(dev, "vcn33-1");
	if (IS_ERR(bc->vcn33_1))
		return dev_err_probe(dev, PTR_ERR(bc->vcn33_1), "vcn33-1\n");
	bc->vcn33_2 = devm_regulator_get(dev, "vcn33-2");
	if (IS_ERR(bc->vcn33_2))
		return dev_err_probe(dev, PTR_ERR(bc->vcn33_2), "vcn33-2\n");

	mutex_init(&bc->power_lock);
	mutex_init(&bc->wmt_lock);
	mutex_init(&bc->tx_lock);
	init_completion(&bc->wmt_done);
	skb_queue_head_init(&bc->txq);
	INIT_WORK(&bc->tx_work, btmtkconn_tx_work);

	serdev_device_set_drvdata(serdev, bc);
	serdev_device_set_client_ops(serdev, &btmtkconn_client_ops);
	pm_runtime_enable(dev);

	hdev = hci_alloc_dev();
	if (!hdev)
		return -ENOMEM;
	bc->hdev = hdev;
	bc->hu.hdev = hdev;
	hdev->bus = HCI_UART;
	hdev->manufacturer = 70;	/* MediaTek */
	hci_set_drvdata(hdev, bc);
	hdev->open = btmtkconn_open;
	hdev->close = btmtkconn_close;
	hdev->flush = btmtkconn_flush;
	hdev->send = btmtkconn_send;
	hdev->set_bdaddr = btmtkconn_set_bdaddr;
	SET_HCIDEV_DEV(hdev, dev);
	/* the factory address is in nvdata: user space sets it (btmgmt public-addr) */
	hci_set_quirk(hdev, HCI_QUIRK_INVALID_BDADDR);
	/* switching Bluetooth off and on again brings back the firmware's placeholder address */
	hci_set_quirk(hdev, HCI_QUIRK_NON_PERSISTENT_SETUP);

	/*
	 * Opened and set up for good here, while connsys is still off: the 8250 core writes its baud
	 * divisor through THR whenever the termios change, which would reach a running connsys.
	 */
	ret = serdev_device_open(serdev);
	if (ret) {
		hci_free_dev(hdev);
		return dev_err_probe(dev, ret, "open BTIF\n");
	}
	/* serdev opens with RTS/CTS: BTIF has no modem lines, CTS would never let it send */
	serdev_device_set_flow_control(serdev, false);

	ret = hci_register_dev(hdev);
	if (ret < 0) {
		serdev_device_close(serdev);
		hci_free_dev(hdev);
		return dev_err_probe(dev, ret, "register\n");
	}

	mutex_lock(&connsys_list_lock);
	list_add_tail(&bc->cs.node, &connsys_list);
	mutex_unlock(&connsys_list_lock);
	btmtkconn_debugfs(bc);
	return 0;
}

static void btmtkconn_remove(struct serdev_device *serdev)
{
	struct btmtkconn *bc = serdev_device_get_drvdata(serdev);

	mutex_lock(&connsys_list_lock);
	list_del(&bc->cs.node);
	mutex_unlock(&connsys_list_lock);
	hci_unregister_dev(bc->hdev);
	hci_free_dev(bc->hdev);
	serdev_device_close(serdev);
}

static const struct of_device_id btmtkconn_of_match[] = {
	{ .compatible = "mediatek,mt6833-bluetooth" },
	{ }
};
MODULE_DEVICE_TABLE(of, btmtkconn_of_match);

static struct serdev_device_driver btmtkconn_driver = {
	.probe = btmtkconn_probe,
	.remove = btmtkconn_remove,
	.driver = {
		.name = "btmtkconn",
		.of_match_table = btmtkconn_of_match,
		.suppress_bind_attrs = true,
	},
};
module_serdev_device_driver(btmtkconn_driver);

MODULE_DESCRIPTION("MediaTek MT6833 connsys Bluetooth (prototype)");
MODULE_LICENSE("GPL");
