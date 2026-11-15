// SPDX-License-Identifier: BSD-3-Clause-Clear
/* Copyright (C) 2019 MediaTek Inc.
 *
 * Author: Roy Luo <royluo@google.com>
 *         Ryder Lee <ryder.lee@mediatek.com>
 */

#include <linux/firmware.h>
#include <linux/of.h>
#include <linux/soc/mediatek/mtk-connsys.h>
#include "mt7615.h"
#include "mcu.h"
#include "mac.h"
#include "eeprom.h"

static bool prefer_offload_fw = true;
module_param(prefer_offload_fw, bool, 0644);
MODULE_PARM_DESC(prefer_offload_fw,
		 "Prefer client mode offload firmware (MT7663)");

struct mt7615_patch_hdr {
	char build_date[16];
	char platform[4];
	__be32 hw_sw_ver;
	__be32 patch_ver;
	__be16 checksum;
} __packed;

struct mt7615_fw_trailer {
	__le32 addr;
	u8 chip_id;
	u8 feature_set;
	u8 eco_code;
	char fw_ver[10];
	char build_date[15];
	__le32 len;
} __packed;

#define FW_V3_COMMON_TAILER_SIZE	36
#define FW_V3_REGION_TAILER_SIZE	40
#define FW_START_OVERRIDE		BIT(0)
#define FW_START_DLYCAL                 BIT(1)
#define FW_START_WORKING_PDA_CR4	BIT(2)

struct mt7663_fw_buf {
	__le32 crc;
	__le32 d_img_size;
	__le32 block_size;
	u8 rsv[4];
	__le32 img_dest_addr;
	__le32 img_size;
	u8 feature_set;
};

#define MT7615_PATCH_ADDRESS		0x80000
#define MT7622_PATCH_ADDRESS		0x9c000
#define MT7663_PATCH_ADDRESS		0xdc000

#define N9_REGION_NUM			2
#define CR4_REGION_NUM			1

#define IMG_CRC_LEN			4

/* MT6833 prototype: the host side of the PDMA and the handshake registers, to the log */
static void mt6833_dump_regs(struct mt7615_dev *dev, const char *when)
{
	static const struct {
		const char *name;
		u32 addr;
	} r[] = {
		{ "INT_STA", 0x4200 }, { "INT_MSK", 0x4204 }, { "GLO_CFG", 0x4208 },
		{ "APSRC", 0x4160 }, { "MCU2HOST_STA", 0x41f0 }, { "MCU2HOST_EN", 0x41f4 },
		{ "TX3_BASE", 0x4330 }, { "TX3_CNT", 0x4334 }, { "TX3_CIDX", 0x4338 },
		{ "TX3_DIDX", 0x433c }, { "TX15_BASE", 0x43f0 }, { "TX15_CIDX", 0x43f8 },
		{ "TX15_DIDX", 0x43fc }, { "RX0_BASE", 0x4400 }, { "RX0_CIDX", 0x4408 },
		{ "RX0_DIDX", 0x440c }, { "RX1_BASE", 0x4410 }, { "RX1_CNT", 0x4414 },
		{ "RX1_CIDX", 0x4418 }, { "RX1_DIDX", 0x441c }, { "LPCTL", 0x7000 },
		{ "IRQ_ENA", 0x7008 }, { "HIF_ON_MISC", 0x713c }, { "CONN_ON_MISC", 0xc1140 },
		{ "ON_HOST_DBG", 0xc1144 }, { "C2A_SLP", 0xbc010 }, { "MCU2A_SLP", 0xbc014 },
		{ "C2A_GALS", 0xbc018 }, { "C2A_MUX", 0xbc01c }, { "HIF_BUSY", 0xbc020 },
		{ "CSR", 0x7100 }, { "MCU_PC", 0x7104 }, { "RGU", 0x7108 },
		{ "WFSYS_BUSY", 0x710c }, { "MCU_PWR", 0x7114 }, { "FW_OWN_SET", 0x7118 },
		{ "MBOX0", 0x711c }, { "MBOX1", 0x7120 }, { "MBOX2", 0x7124 },
		{ "MBOX3", 0x7128 }, { "MCU_PC", 0x7104 }, { "MCU_PC", 0x7104 },
	};
	char buf[512];
	int i, n = 0;

	for (i = 0; i < ARRAY_SIZE(r); i++) {
		n += scnprintf(buf + n, sizeof(buf) - n, " %s=%08x", r[i].name,
			       mt76_rr(dev, r[i].addr));
		if (i % 6 == 5 || i == ARRAY_SIZE(r) - 1) {
			dev_info(dev->mt76.dev, "regs %s:%s\n", when, buf);
			n = 0;
		}
	}
}

/* MT6833 prototype: every MCU command and event to the log (not the firmware scatter) */
static bool mt6833_trace = true;
module_param(mt6833_trace, bool, 0644);
MODULE_PARM_DESC(mt6833_trace, "MT6833 prototype: log MCU commands and events");

static void mt6833_trace_tx(struct mt7615_dev *dev, struct sk_buff *skb, int cmd, int seq)
{
	int hdr = cmd & __MCU_CMD_FIELD_UNI ? sizeof(struct mt7615_uni_txd) :
					      sizeof(struct mt7615_mcu_txd);
	/* not the MAC addresses at offset 4 */
	int n = cmd == MCU_CE_CMD(SET_NVRAM) || cmd == MCU_CE_CMD(BSS_ACTIVATE_CTRL) ||
		cmd == MCU_CE_CMD(SET_BSS_INFO) ? 4 : cmd == MCU_CE_CMD(UPDATE_STA_RECORD) ? 2 : 16;

	if (!is_mt6833(&dev->mt76) || !mt6833_trace || cmd == MCU_CMD(FW_SCATTER))
		return;
	dev_info(dev->mt76.dev, "mcu tx %08x seq %d len %d: %*ph\n", cmd, seq,
		 skb->len - hdr, min_t(int, skb->len - hdr, n), skb->data + hdr);
}

void mt7615_mcu_fill_msg(struct mt7615_dev *dev, struct sk_buff *skb,
			 int cmd, int *wait_seq)
{
	int txd_len, mcu_cmd = FIELD_GET(__MCU_CMD_FIELD_ID, cmd);
	struct mt7615_uni_txd *uni_txd;
	struct mt7615_mcu_txd *mcu_txd;
	u8 seq, q_idx, pkt_fmt;
	__le32 *txd;
	u32 val;

	/* TODO: make dynamic based on msg type */
	dev->mt76.mcu.timeout = 20 * HZ;

	seq = ++dev->mt76.mcu.msg_seq & 0xf;
	if (!seq)
		seq = ++dev->mt76.mcu.msg_seq & 0xf;
	if (wait_seq)
		*wait_seq = seq;

	txd_len = cmd & __MCU_CMD_FIELD_UNI ? sizeof(*uni_txd) : sizeof(*mcu_txd);
	txd = (__le32 *)skb_push(skb, txd_len);

	if (cmd != MCU_CMD(FW_SCATTER)) {
		q_idx = MT_TX_MCU_PORT_RX_Q0;
		pkt_fmt = MT_TX_TYPE_CMD;
	} else {
		q_idx = MT_TX_MCU_PORT_RX_FWDL;
		pkt_fmt = MT_TX_TYPE_FW;
	}

	val = FIELD_PREP(MT_TXD0_TX_BYTES, skb->len) |
	      FIELD_PREP(MT_TXD0_P_IDX, MT_TX_PORT_IDX_MCU) |
	      FIELD_PREP(MT_TXD0_Q_IDX, q_idx);
	txd[0] = cpu_to_le32(val);

	val = MT_TXD1_LONG_FORMAT |
	      FIELD_PREP(MT_TXD1_HDR_FORMAT, MT_HDR_FORMAT_CMD) |
	      FIELD_PREP(MT_TXD1_PKT_FMT, pkt_fmt);
	txd[1] = cpu_to_le32(val);

	if (cmd & __MCU_CMD_FIELD_UNI) {
		uni_txd = (struct mt7615_uni_txd *)txd;
		uni_txd->len = cpu_to_le16(skb->len - sizeof(uni_txd->txd));
		uni_txd->option = MCU_CMD_UNI_EXT_ACK;
		uni_txd->cid = cpu_to_le16(mcu_cmd);
		uni_txd->s2d_index = MCU_S2D_H2N;
		uni_txd->pkt_type = MCU_PKT_ID;
		uni_txd->seq = seq;

		return;
	}

	mcu_txd = (struct mt7615_mcu_txd *)txd;
	mcu_txd->len = cpu_to_le16(skb->len - sizeof(mcu_txd->txd));
	mcu_txd->pq_id = cpu_to_le16(MCU_PQ_ID(MT_TX_PORT_IDX_MCU, q_idx));
	mcu_txd->s2d_index = MCU_S2D_H2N;
	mcu_txd->pkt_type = MCU_PKT_ID;
	mcu_txd->seq = seq;
	mcu_txd->cid = mcu_cmd;
	mcu_txd->ext_cid = FIELD_GET(__MCU_CMD_FIELD_EXT_ID, cmd);

	if (mcu_txd->ext_cid || (cmd & __MCU_CMD_FIELD_CE)) {
		if (cmd & __MCU_CMD_FIELD_QUERY)
			mcu_txd->set_query = MCU_Q_QUERY;
		else
			mcu_txd->set_query = MCU_Q_SET;
		mcu_txd->ext_cid_ack = !!mcu_txd->ext_cid;
	} else {
		mcu_txd->set_query = MCU_Q_NA;
	}
}
EXPORT_SYMBOL_GPL(mt7615_mcu_fill_msg);

int mt7615_mcu_parse_response(struct mt76_dev *mdev, int cmd,
			      struct sk_buff *skb, int seq)
{
	struct mt7615_mcu_rxd *rxd;
	int ret = 0;

	if (!skb) {
		dev_err(mdev->dev, "Message %08x (seq %d) timeout\n",
			cmd, seq);
		if (is_mt6833(mdev))
			mt6833_dump_regs(container_of(mdev, struct mt7615_dev, mt76),
					 "timeout");
		return -ETIMEDOUT;
	}

	rxd = (struct mt7615_mcu_rxd *)skb->data;
	if (skb->len < sizeof(*rxd))
		return -EMSGSIZE;

	if (seq != rxd->seq)
		return -EAGAIN;

	if (cmd == MCU_CMD(PATCH_SEM_CONTROL)) {
		skb_pull(skb, sizeof(*rxd) - 4);
		ret = *skb->data;
	} else if (cmd == MCU_EXT_CMD(THERMAL_CTRL)) {
		if (skb->len < sizeof(*rxd) + (is_mt7663(mdev) ? 8 : 4))
			return -EMSGSIZE;
		skb_pull(skb, sizeof(*rxd));
		if (is_mt7663(mdev))
			skb_pull(skb, 4);
		ret = le32_to_cpu(*(__le32 *)skb->data);
	} else if (cmd == MCU_EXT_QUERY(RF_REG_ACCESS)) {
		if (skb->len < sizeof(*rxd) + 8 + sizeof(__le32))
			return -EMSGSIZE;
		skb_pull(skb, sizeof(*rxd));
		ret = le32_to_cpu(*(__le32 *)&skb->data[8]);
	} else if (cmd == MCU_UNI_CMD(DEV_INFO_UPDATE) ||
		   cmd == MCU_UNI_CMD(BSS_INFO_UPDATE) ||
		   cmd == MCU_UNI_CMD(STA_REC_UPDATE) ||
		   cmd == MCU_UNI_CMD(HIF_CTRL) ||
		   cmd == MCU_UNI_CMD(OFFLOAD) ||
		   cmd == MCU_UNI_CMD(SUSPEND)) {
		struct mt76_connac_mcu_uni_event *event;

		if (skb->len < sizeof(*rxd) + sizeof(*event))
			return -EMSGSIZE;
		skb_pull(skb, sizeof(*rxd));
		event = (struct mt76_connac_mcu_uni_event *)skb->data;
		ret = le32_to_cpu(event->status);
	} else if (cmd == MCU_CE_QUERY(REG_READ)) {
		struct mt76_connac_mcu_reg_event *event;

		if (skb->len < sizeof(*rxd) + sizeof(*event))
			return -EMSGSIZE;
		skb_pull(skb, sizeof(*rxd));
		event = (struct mt76_connac_mcu_reg_event *)skb->data;
		ret = (int)le32_to_cpu(event->val);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(mt7615_mcu_parse_response);

static int
mt7615_mcu_send_message(struct mt76_dev *mdev, struct sk_buff *skb,
			int cmd, int *seq)
{
	struct mt7615_dev *dev = container_of(mdev, struct mt7615_dev, mt76);
	enum mt76_mcuq_id qid;

	mt7615_mcu_fill_msg(dev, skb, cmd, seq);
	mt6833_trace_tx(dev, skb, cmd, seq ? *seq : -1);
	if (test_bit(MT76_STATE_MCU_RUNNING, &dev->mphy.state))
		qid = MT_MCUQ_WM;
	else
		qid = MT_MCUQ_FWDL;

	return mt76_tx_queue_skb_raw(dev, dev->mt76.q_mcu[qid], skb, 0);
}

u32 mt7615_rf_rr(struct mt7615_dev *dev, u32 wf, u32 reg)
{
	struct {
		__le32 wifi_stream;
		__le32 address;
		__le32 data;
	} req = {
		.wifi_stream = cpu_to_le32(wf),
		.address = cpu_to_le32(reg),
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_EXT_QUERY(RF_REG_ACCESS),
				 &req, sizeof(req), true);
}

int mt7615_rf_wr(struct mt7615_dev *dev, u32 wf, u32 reg, u32 val)
{
	struct {
		__le32 wifi_stream;
		__le32 address;
		__le32 data;
	} req = {
		.wifi_stream = cpu_to_le32(wf),
		.address = cpu_to_le32(reg),
		.data = cpu_to_le32(val),
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_EXT_CMD(RF_REG_ACCESS),
				 &req, sizeof(req), false);
}

void mt7622_trigger_hif_int(struct mt7615_dev *dev, bool en)
{
	if (!is_mt7622(&dev->mt76))
		return;

	regmap_update_bits(dev->infracfg, MT_INFRACFG_MISC,
			   MT_INFRACFG_MISC_AP2CONN_WAKE,
			   !en * MT_INFRACFG_MISC_AP2CONN_WAKE);
}
EXPORT_SYMBOL_GPL(mt7622_trigger_hif_int);

static int mt7615_mcu_drv_pmctrl(struct mt7615_dev *dev)
{
	struct mt76_phy *mphy = &dev->mt76.phy;
	struct mt76_connac_pm *pm = &dev->pm;
	struct mt76_dev *mdev = &dev->mt76;
	u32 addr;
	int err;

	if (is_mt7663(mdev)) {
		/* Clear firmware own via N9 eint */
		if (!is_mt6833(mdev)) {
			mt76_wr(dev, MT_PCIE_DOORBELL_PUSH, MT_CFG_LPCR_HOST_DRV_OWN);
			mt76_poll(dev, MT_CONN_ON_MISC, MT_CFG_LPCR_HOST_FW_OWN, 0, 3000);
		}

		addr = MT_CONN_HIF_ON_LPCTL;
	} else {
		addr = MT_CFG_LPCR_HOST;
	}

	mt76_wr(dev, addr, MT_CFG_LPCR_HOST_DRV_OWN);

	mt7622_trigger_hif_int(dev, true);

	err = !mt76_poll_msec(dev, addr, MT_CFG_LPCR_HOST_FW_OWN, 0, 3000);

	mt7622_trigger_hif_int(dev, false);

	if (err) {
		dev_err(mdev->dev, "driver own failed\n");
		return -ETIMEDOUT;
	}

	clear_bit(MT76_STATE_PM, &mphy->state);

	pm->stats.last_wake_event = jiffies;
	pm->stats.doze_time += pm->stats.last_wake_event -
			       pm->stats.last_doze_event;

	return 0;
}

/*
 * MT6833: back from the firmware's deep sleep, the TX rings start over at index 0 (MediaTek's
 * asicCheckDummyReg()); the firmware keeps the ring bases and the RX rings. The host only hands
 * over the WMAC with nothing in flight, so what is left in the TX rings is done.
 */
static void mt6833_wfdma_check(struct mt7615_dev *dev)
{
	struct mt76_dev *mdev = &dev->mt76;
	u32 val = mt76_rr(dev, MT6833_DUMMY_CR);
	int i;

	if (val == MT6833_DUMMY_MAGIC)
		return;
	if (val != MT6833_DUMMY_RESET)
		dev_warn(mdev->dev, "PDMA dummy register %08x after sleep\n", val);

	for (i = 0; i < __MT_TXQ_MAX; i++) {
		struct mt76_queue *q = dev->mphy.q_tx[i];

		if (q && (i == 0 || q != dev->mphy.q_tx[0])) {
			mt76_queue_tx_cleanup(dev, q, true);
			mt76_queue_reset(dev, q, true);
		}
	}
	for (i = 0; i < __MT_MCUQ_MAX; i++) {
		if (!mdev->q_mcu[i])
			continue;
		mt76_queue_tx_cleanup(dev, mdev->q_mcu[i], true);
		mt76_queue_reset(dev, mdev->q_mcu[i], true);
	}
	dev->mt6833_deep_sleeps++;
	mt76_wr(dev, MT6833_DUMMY_CR, MT6833_DUMMY_MAGIC);
}

static int mt7615_mcu_lp_drv_pmctrl(struct mt7615_dev *dev)
{
	struct mt76_phy *mphy = &dev->mt76.phy;
	struct mt76_connac_pm *pm = &dev->pm;
	int i, err = 0;

	mutex_lock(&pm->mutex);

	if (!test_bit(MT76_STATE_PM, &mphy->state))
		goto out;

	for (i = 0; i < MT7615_DRV_OWN_RETRY_COUNT; i++) {
		if (is_mt6833(&dev->mt76))
			mt76_wr(dev, MT_CONN_HIF_ON_LPCTL, MT_CFG_LPCR_HOST_DRV_OWN);
		else
			mt76_wr(dev, MT_PCIE_DOORBELL_PUSH, MT_CFG_LPCR_HOST_DRV_OWN);
		if (mt76_poll_msec(dev, MT_CONN_HIF_ON_LPCTL,
				   MT_CFG_LPCR_HOST_FW_OWN, 0, 50))
			break;
	}

	if (i == MT7615_DRV_OWN_RETRY_COUNT) {
		dev_err(dev->mt76.dev, "driver own failed\n");
		err = -EIO;
		goto out;
	}
	if (is_mt6833(&dev->mt76)) {
		WRITE_ONCE(dev->mt6833_pmctrl_task, current);
		mt6833_wfdma_check(dev);
		if (READ_ONCE(dev->mt6833_pm_irq_masked)) {
			WRITE_ONCE(dev->mt6833_pm_irq_masked, false);
			mt76_wr(dev, MT_CONN_HIF_ON_IRQ_ENA,
				MT_CONN_HIF_ON_IRQ_ENA_HOST);
		}
		WRITE_ONCE(dev->mt6833_pmctrl_task, NULL);
	}
	clear_bit(MT76_STATE_PM, &mphy->state);

	pm->stats.last_wake_event = jiffies;
	pm->stats.doze_time += pm->stats.last_wake_event -
			       pm->stats.last_doze_event;
out:
	mutex_unlock(&pm->mutex);

	return err;
}

static int mt7615_mcu_fw_pmctrl(struct mt7615_dev *dev)
{
	struct mt76_phy *mphy = &dev->mt76.phy;
	struct mt76_connac_pm *pm = &dev->pm;
	int err = 0;
	u32 addr;

	mutex_lock(&pm->mutex);

	if (is_mt6833(&dev->mt76)) {
		if (dev->mt76.token_count)
			dev->mt6833_skip_token++;
		else if (pm->wake.count)
			dev->mt6833_skip_wake++;
	}
	if (mt76_connac_skip_fw_pmctrl(mphy, pm))
		goto out;
	dev->mt6833_fw_owns++;

	mt7622_trigger_hif_int(dev, true);

	addr = is_mt7663(&dev->mt76) ? MT_CONN_HIF_ON_LPCTL : MT_CFG_LPCR_HOST;
	if (is_mt6833(&dev->mt76)) {
		WRITE_ONCE(dev->mt6833_pmctrl_task, current);
		mt76_wr(dev, MT6833_DUMMY_CR, MT6833_DUMMY_MAGIC);
		WRITE_ONCE(dev->mt6833_pmctrl_task, NULL);
	}
	mt76_wr(dev, addr, MT_CFG_LPCR_HOST_FW_OWN);

	if (is_mt7622(&dev->mt76) &&
	    !mt76_poll_msec(dev, addr, MT_CFG_LPCR_HOST_FW_OWN,
			    MT_CFG_LPCR_HOST_FW_OWN, 3000)) {
		dev_err(dev->mt76.dev, "Timeout for firmware own\n");
		clear_bit(MT76_STATE_PM, &mphy->state);
		err = -EIO;
	}

	mt7622_trigger_hif_int(dev, false);
	if (!err) {
		pm->stats.last_doze_event = jiffies;
		pm->stats.awake_time += pm->stats.last_doze_event -
					pm->stats.last_wake_event;
	}
out:
	mutex_unlock(&pm->mutex);

	return err;
}

static void
mt7615_mcu_csa_finish(void *priv, u8 *mac, struct ieee80211_vif *vif)
{
	if (vif->bss_conf.csa_active)
		ieee80211_csa_finish(vif, 0);
}

static void
mt7615_mcu_rx_csa_notify(struct mt7615_dev *dev, struct sk_buff *skb)
{
	struct mt7615_phy *ext_phy = mt7615_ext_phy(dev);
	struct mt76_phy *mphy = &dev->mt76.phy;
	struct mt7615_mcu_csa_notify *c;

	c = (struct mt7615_mcu_csa_notify *)skb->data;

	if (c->omac_idx > EXT_BSSID_MAX)
		return;

	if (ext_phy && ext_phy->omac_mask & BIT_ULL(c->omac_idx))
		mphy = dev->mt76.phys[MT_BAND1];

	ieee80211_iterate_active_interfaces_atomic(mphy->hw,
			IEEE80211_IFACE_ITER_RESUME_ALL,
			mt7615_mcu_csa_finish, mphy->hw);
}

static void
mt7615_mcu_rx_radar_detected(struct mt7615_dev *dev, struct sk_buff *skb)
{
	struct mt76_phy *mphy = &dev->mt76.phy;
	struct mt7615_mcu_rdd_report *r;

	r = (struct mt7615_mcu_rdd_report *)skb->data;

	if (!dev->radar_pattern.n_pulses && !r->long_detected &&
	    !r->constant_prf_detected && !r->staggered_prf_detected)
		return;

	if (r->band_idx && dev->mt76.phys[MT_BAND1])
		mphy = dev->mt76.phys[MT_BAND1];

	if (mt76_phy_dfs_state(mphy) < MT_DFS_STATE_CAC)
		return;

	ieee80211_radar_detected(mphy->hw, NULL);
	dev->hw_pattern++;
}

static void
mt7615_mcu_rx_log_message(struct mt7615_dev *dev, struct sk_buff *skb)
{
	struct mt7615_mcu_rxd *rxd = (struct mt7615_mcu_rxd *)skb->data;
	const char *data = (char *)&rxd[1];
	const char *type;

	switch (rxd->s2d_index) {
	case 0:
		type = "N9";
		break;
	case 2:
		type = "CR4";
		break;
	default:
		type = "unknown";
		break;
	}

	wiphy_info(mt76_hw(dev)->wiphy, "%s: %.*s", type,
		   (int)(skb->len - sizeof(*rxd)), data);
}

/* DEBUG: MT6833's firmware log (gen4m's EVENT_DEBUG_MSG) */
static void
mt6833_mcu_rx_debug_msg(struct mt7615_dev *dev, struct sk_buff *skb)
{
	struct {
		__le16 id;
		u8 type;	/* 3: text */
		u8 flags;
		__le32 value;
		__le16 size;
		u8 rsv[2];
		u8 msg[];
	} __packed *m;
	int len;

	skb_pull(skb, sizeof(struct mt7615_mcu_rxd));
	if (skb->len < sizeof(*m))
		return;
	m = (void *)skb->data;
	len = min_t(int, le16_to_cpu(m->size), skb->len - sizeof(*m));
	if (m->type == 3) {
		while (len && (m->msg[len - 1] == '\n' || !m->msg[len - 1]))
			len--;
		dev_info(dev->mt76.dev, "fw: %.*s\n", len, m->msg);
	} else {
		dev_info(dev->mt76.dev, "fw msg %u type %u value %08x: %*ph\n",
			 le16_to_cpu(m->id), m->type, le32_to_cpu(m->value),
			 min(len, 32), m->msg);
	}
}

static void
mt7615_mcu_rx_ext_event(struct mt7615_dev *dev, struct sk_buff *skb)
{
	struct mt7615_mcu_rxd *rxd = (struct mt7615_mcu_rxd *)skb->data;

	switch (rxd->ext_eid) {
	case MCU_EXT_EVENT_RDD_REPORT:
		mt7615_mcu_rx_radar_detected(dev, skb);
		break;
	case MCU_EXT_EVENT_CSA_NOTIFY:
		mt7615_mcu_rx_csa_notify(dev, skb);
		break;
	case MCU_EXT_EVENT_FW_LOG_2_HOST:
		mt7615_mcu_rx_log_message(dev, skb);
		break;
	default:
		break;
	}
}

static void
mt7615_mcu_scan_event(struct mt7615_dev *dev, struct sk_buff *skb)
{
	u8 *seq_num = skb->data + sizeof(struct mt7615_mcu_rxd);
	struct mt7615_phy *phy;
	struct mt76_phy *mphy;

	if (*seq_num & BIT(7) && dev->mt76.phys[MT_BAND1])
		mphy = dev->mt76.phys[MT_BAND1];
	else
		mphy = &dev->mt76.phy;

	phy = mphy->priv;

	spin_lock_bh(&dev->mt76.lock);
	__skb_queue_tail(&phy->scan_event_list, skb);
	spin_unlock_bh(&dev->mt76.lock);

	ieee80211_queue_delayed_work(mphy->hw, &phy->scan_work,
				     MT7615_HW_SCAN_TIMEOUT);
}

static void
mt7615_mcu_roc_event(struct mt7615_dev *dev, struct sk_buff *skb)
{
	struct mt7615_roc_tlv *event;
	struct mt7615_phy *phy;
	struct mt76_phy *mphy;
	int duration;

	skb_pull(skb, sizeof(struct mt7615_mcu_rxd));
	event = (struct mt7615_roc_tlv *)skb->data;

	if (event->dbdc_band && dev->mt76.phys[MT_BAND1])
		mphy = dev->mt76.phys[MT_BAND1];
	else
		mphy = &dev->mt76.phy;

	ieee80211_ready_on_channel(mphy->hw);

	phy = mphy->priv;
	phy->roc_grant = true;
	wake_up(&phy->roc_wait);

	duration = le32_to_cpu(event->max_interval);
	mod_timer(&phy->roc_timer,
		  round_jiffies_up(jiffies + msecs_to_jiffies(duration)));
}

static void
mt7615_mcu_beacon_loss_event(struct mt7615_dev *dev, struct sk_buff *skb)
{
	struct mt76_connac_beacon_loss_event *event;
	struct mt76_phy *mphy;
	u8 band_idx = 0; /* DBDC support */

	skb_pull(skb, sizeof(struct mt7615_mcu_rxd));
	event = (struct mt76_connac_beacon_loss_event *)skb->data;
	if (band_idx && dev->mt76.phys[MT_BAND1])
		mphy = dev->mt76.phys[MT_BAND1];
	else
		mphy = &dev->mt76.phy;

	ieee80211_iterate_active_interfaces_atomic(mphy->hw,
					IEEE80211_IFACE_ITER_RESUME_ALL,
					mt76_connac_mcu_beacon_loss_iter,
					event);
}

static void
mt7615_mcu_bss_event(struct mt7615_dev *dev, struct sk_buff *skb)
{
	struct mt76_connac_mcu_bss_event *event;
	struct mt76_phy *mphy;
	u8 band_idx = 0; /* DBDC support */

	skb_pull(skb, sizeof(struct mt7615_mcu_rxd));
	event = (struct mt76_connac_mcu_bss_event *)skb->data;

	if (band_idx && dev->mt76.phys[MT_BAND1])
		mphy = dev->mt76.phys[MT_BAND1];
	else
		mphy = &dev->mt76.phy;

	if (event->is_absent)
		ieee80211_stop_queues(mphy->hw);
	else
		ieee80211_wake_queues(mphy->hw);
}

static void
mt7615_mcu_rx_unsolicited_event(struct mt7615_dev *dev, struct sk_buff *skb)
{
	struct mt7615_mcu_rxd *rxd = (struct mt7615_mcu_rxd *)skb->data;

	switch (rxd->eid) {
	case MCU_EVENT_EXT:
		mt7615_mcu_rx_ext_event(dev, skb);
		break;
	case MCU_EVENT_BSS_BEACON_LOSS:
		mt7615_mcu_beacon_loss_event(dev, skb);
		break;
	case MCU_EVENT_ROC:
		mt7615_mcu_roc_event(dev, skb);
		break;
	case MCU_EVENT_SCHED_SCAN_DONE:
	case MCU_EVENT_SCAN_DONE:
		mt7615_mcu_scan_event(dev, skb);
		return;
	case MCU_EVENT_BSS_ABSENCE:
		mt7615_mcu_bss_event(dev, skb);
		break;
	case MCU_EVENT_COREDUMP:
		mt76_connac_mcu_coredump_event(&dev->mt76, skb,
					       &dev->coredump);
		return;
	case MT6833_EVENT_DEBUG_MSG:
		if (is_mt6833(&dev->mt76))
			mt6833_mcu_rx_debug_msg(dev, skb);
		break;
	case MT6833_EVENT_SLEEPY_INFO:
		/*
		 * the firmware wants to sleep: let it have the WMAC as soon as the host is idle
		 * (MediaTek's gen4m sets firmware own on this), rather than only after the next
		 * command, which may not come before the firmware sleeps on its own
		 */
		if (is_mt6833(&dev->mt76) &&
		    skb->len > sizeof(struct mt7615_mcu_rxd) &&
		    skb->data[sizeof(struct mt7615_mcu_rxd)] &&
		    test_bit(MT76_STATE_MCU_RUNNING, &dev->mphy.state))
			mt76_connac_power_save_sched(&dev->mphy, &dev->pm);
		break;
	default:
		break;
	}
	dev_kfree_skb(skb);
}

void mt7615_mcu_rx_event(struct mt7615_dev *dev, struct sk_buff *skb)
{
	struct mt7615_mcu_rxd *rxd = (struct mt7615_mcu_rxd *)skb->data;

	if (rxd->ext_eid == MCU_EXT_EVENT_THERMAL_PROTECT ||
	    rxd->ext_eid == MCU_EXT_EVENT_FW_LOG_2_HOST ||
	    rxd->ext_eid == MCU_EXT_EVENT_ASSERT_DUMP ||
	    rxd->ext_eid == MCU_EXT_EVENT_PS_SYNC ||
	    rxd->eid == MCU_EVENT_BSS_BEACON_LOSS ||
	    rxd->eid == MCU_EVENT_SCHED_SCAN_DONE ||
	    rxd->eid == MCU_EVENT_BSS_ABSENCE ||
	    rxd->eid == MCU_EVENT_SCAN_DONE ||
	    rxd->eid == MCU_EVENT_COREDUMP ||
	    rxd->eid == MCU_EVENT_ROC ||
	    (is_mt6833(&dev->mt76) && rxd->eid == MT6833_EVENT_SLEEPY_INFO) ||
	    (is_mt6833(&dev->mt76) && rxd->eid == MT6833_EVENT_DEBUG_MSG) ||
	    !rxd->seq)
		mt7615_mcu_rx_unsolicited_event(dev, skb);
	else
		mt76_mcu_rx_event(&dev->mt76, skb);
}

static int
mt7615_mcu_muar_config(struct mt7615_dev *dev, struct ieee80211_vif *vif,
		       bool bssid, bool enable)
{
	struct mt7615_vif *mvif = (struct mt7615_vif *)vif->drv_priv;
	u32 idx = mvif->mt76.omac_idx - REPEATER_BSSID_START;
	u32 mask = dev->omac_mask >> 32 & ~BIT(idx);
	const u8 *addr = vif->addr;
	struct {
		u8 mode;
		u8 force_clear;
		u8 clear_bitmap[8];
		u8 entry_count;
		u8 write;

		u8 index;
		u8 bssid;
		u8 addr[ETH_ALEN];
	} __packed req = {
		.mode = !!mask || enable,
		.entry_count = 1,
		.write = 1,

		.index = idx * 2 + bssid,
	};

	if (bssid)
		addr = vif->bss_conf.bssid;

	if (enable)
		ether_addr_copy(req.addr, addr);

	return mt76_mcu_send_msg(&dev->mt76, MCU_EXT_CMD(MUAR_UPDATE),
				 &req, sizeof(req), true);
}

static int
mt7615_mcu_add_dev(struct mt7615_phy *phy, struct ieee80211_vif *vif,
		   bool enable)
{
	struct mt7615_vif *mvif = (struct mt7615_vif *)vif->drv_priv;
	struct mt7615_dev *dev = phy->dev;
	struct {
		struct req_hdr {
			u8 omac_idx;
			u8 band_idx;
			__le16 tlv_num;
			u8 is_tlv_append;
			u8 rsv[3];
		} __packed hdr;
		struct req_tlv {
			__le16 tag;
			__le16 len;
			u8 active;
			u8 band_idx;
			u8 omac_addr[ETH_ALEN];
		} __packed tlv;
	} data = {
		.hdr = {
			.omac_idx = mvif->mt76.omac_idx,
			.band_idx = mvif->mt76.band_idx,
			.tlv_num = cpu_to_le16(1),
			.is_tlv_append = 1,
		},
		.tlv = {
			.tag = cpu_to_le16(DEV_INFO_ACTIVE),
			.len = cpu_to_le16(sizeof(struct req_tlv)),
			.active = enable,
			.band_idx = mvif->mt76.band_idx,
		},
	};

	if (mvif->mt76.omac_idx >= REPEATER_BSSID_START)
		return mt7615_mcu_muar_config(dev, vif, false, enable);

	memcpy(data.tlv.omac_addr, vif->addr, ETH_ALEN);
	return mt76_mcu_send_msg(&dev->mt76, MCU_EXT_CMD(DEV_INFO_UPDATE),
				 &data, sizeof(data), true);
}

static int
mt7615_mcu_add_beacon_offload(struct mt7615_dev *dev,
			      struct ieee80211_hw *hw,
			      struct ieee80211_vif *vif, bool enable)
{
	struct mt7615_vif *mvif = (struct mt7615_vif *)vif->drv_priv;
	struct mt76_wcid *wcid = &dev->mt76.global_wcid;
	struct ieee80211_mutable_offsets offs;
	struct ieee80211_tx_info *info;
	struct req {
		u8 omac_idx;
		u8 enable;
		u8 wlan_idx;
		u8 band_idx;
		u8 pkt_type;
		u8 need_pre_tbtt_int;
		__le16 csa_ie_pos;
		__le16 pkt_len;
		__le16 tim_ie_pos;
		u8 pkt[512];
		u8 csa_cnt;
		/* bss color change */
		u8 bcc_cnt;
		__le16 bcc_ie_pos;
	} __packed req = {
		.omac_idx = mvif->mt76.omac_idx,
		.enable = enable,
		.wlan_idx = wcid->idx,
		.band_idx = mvif->mt76.band_idx,
	};
	struct sk_buff *skb;

	if (!enable)
		goto out;

	skb = ieee80211_beacon_get_template(hw, vif, &offs, 0);
	if (!skb)
		return -EINVAL;

	if (skb->len > 512 - MT_TXD_SIZE) {
		dev_err(dev->mt76.dev, "Bcn size limit exceed\n");
		dev_kfree_skb(skb);
		return -EINVAL;
	}

	info = IEEE80211_SKB_CB(skb);
	info->hw_queue |= FIELD_PREP(MT_TX_HW_QUEUE_PHY, mvif->mt76.band_idx);

	mt7615_mac_write_txwi(dev, (__le32 *)(req.pkt), skb, wcid, NULL,
			      0, NULL, 0, true);
	memcpy(req.pkt + MT_TXD_SIZE, skb->data, skb->len);
	req.pkt_len = cpu_to_le16(MT_TXD_SIZE + skb->len);
	req.tim_ie_pos = cpu_to_le16(MT_TXD_SIZE + offs.tim_offset);
	if (offs.cntdwn_counter_offs[0]) {
		u16 csa_offs;

		csa_offs = MT_TXD_SIZE + offs.cntdwn_counter_offs[0] - 4;
		req.csa_ie_pos = cpu_to_le16(csa_offs);
		req.csa_cnt = skb->data[offs.cntdwn_counter_offs[0]];
	}
	dev_kfree_skb(skb);

out:
	return mt76_mcu_send_msg(&dev->mt76, MCU_EXT_CMD(BCN_OFFLOAD), &req,
				 sizeof(req), true);
}

static int
mt7615_mcu_ctrl_pm_state(struct mt7615_dev *dev, int band, int state)
{
	return mt76_connac_mcu_set_pm(&dev->mt76, band, state);
}

static int
mt7615_mcu_add_bss(struct mt7615_phy *phy, struct ieee80211_vif *vif,
		   struct ieee80211_sta *sta, bool enable)
{
	struct mt7615_vif *mvif = (struct mt7615_vif *)vif->drv_priv;
	struct mt7615_dev *dev = phy->dev;
	struct sk_buff *skb;

	if (mvif->mt76.omac_idx >= REPEATER_BSSID_START)
		mt7615_mcu_muar_config(dev, vif, true, enable);

	skb = mt76_connac_mcu_alloc_sta_req(&dev->mt76, &mvif->mt76, NULL);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	if (enable)
		mt76_connac_mcu_bss_omac_tlv(skb, vif);

	mt76_connac_mcu_bss_basic_tlv(skb, vif, sta, phy->mt76,
				      mvif->sta.wcid.idx, enable);

	if (enable && mvif->mt76.omac_idx >= EXT_BSSID_START &&
	    mvif->mt76.omac_idx < REPEATER_BSSID_START)
		mt76_connac_mcu_bss_ext_tlv(skb, &mvif->mt76);

	return mt76_mcu_skb_send_msg(&dev->mt76, skb,
				     MCU_EXT_CMD(BSS_INFO_UPDATE), true);
}

static int
mt7615_mcu_wtbl_tx_ba(struct mt7615_dev *dev,
		      struct ieee80211_ampdu_params *params,
		      bool enable)
{
	struct mt7615_sta *msta = (struct mt7615_sta *)params->sta->drv_priv;
	struct mt7615_vif *mvif = msta->vif;
	struct wtbl_req_hdr *wtbl_hdr;
	struct sk_buff *skb = NULL;
	int err;

	wtbl_hdr = mt76_connac_mcu_alloc_wtbl_req(&dev->mt76, &msta->wcid,
						  WTBL_SET, NULL, &skb);
	if (IS_ERR(wtbl_hdr))
		return PTR_ERR(wtbl_hdr);

	mt76_connac_mcu_wtbl_ba_tlv(&dev->mt76, skb, params, enable, true,
				    NULL, wtbl_hdr);

	err = mt76_mcu_skb_send_msg(&dev->mt76, skb,
				    MCU_EXT_CMD(WTBL_UPDATE), true);
	if (err < 0)
		return err;

	skb = mt76_connac_mcu_alloc_sta_req(&dev->mt76, &mvif->mt76,
					    &msta->wcid);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	mt76_connac_mcu_sta_ba_tlv(skb, params, enable, true);

	return mt76_mcu_skb_send_msg(&dev->mt76, skb,
				     MCU_EXT_CMD(STA_REC_UPDATE), true);
}

static int
mt7615_mcu_wtbl_rx_ba(struct mt7615_dev *dev,
		      struct ieee80211_ampdu_params *params,
		      bool enable)
{
	struct mt7615_sta *msta = (struct mt7615_sta *)params->sta->drv_priv;
	struct mt7615_vif *mvif = msta->vif;
	struct wtbl_req_hdr *wtbl_hdr;
	struct sk_buff *skb;
	int err;

	skb = mt76_connac_mcu_alloc_sta_req(&dev->mt76, &mvif->mt76,
					    &msta->wcid);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	mt76_connac_mcu_sta_ba_tlv(skb, params, enable, false);

	err = mt76_mcu_skb_send_msg(&dev->mt76, skb,
				    MCU_EXT_CMD(STA_REC_UPDATE), true);
	if (err < 0 || !enable)
		return err;

	skb = NULL;
	wtbl_hdr = mt76_connac_mcu_alloc_wtbl_req(&dev->mt76, &msta->wcid,
						  WTBL_SET, NULL, &skb);
	if (IS_ERR(wtbl_hdr))
		return PTR_ERR(wtbl_hdr);

	mt76_connac_mcu_wtbl_ba_tlv(&dev->mt76, skb, params, enable, false,
				    NULL, wtbl_hdr);

	return mt76_mcu_skb_send_msg(&dev->mt76, skb,
				     MCU_EXT_CMD(WTBL_UPDATE), true);
}

static int
mt7615_mcu_wtbl_sta_add(struct mt7615_phy *phy, struct ieee80211_vif *vif,
			struct ieee80211_sta *sta, bool enable)
{
	struct mt7615_vif *mvif = (struct mt7615_vif *)vif->drv_priv;
	struct sk_buff *skb, *sskb, *wskb = NULL;
	struct ieee80211_link_sta *link_sta;
	struct mt7615_dev *dev = phy->dev;
	struct wtbl_req_hdr *wtbl_hdr;
	struct mt7615_sta *msta;
	bool new_entry = true;
	int conn_state;
	int cmd, err;

	msta = sta ? (struct mt7615_sta *)sta->drv_priv : &mvif->sta;
	link_sta = sta ? &sta->deflink : NULL;

	sskb = mt76_connac_mcu_alloc_sta_req(&dev->mt76, &mvif->mt76,
					     &msta->wcid);
	if (IS_ERR(sskb))
		return PTR_ERR(sskb);

	if (!sta) {
		if (mvif->sta_added)
			new_entry = false;
		else
			mvif->sta_added = true;
	}
	conn_state = enable ? CONN_STATE_PORT_SECURE : CONN_STATE_DISCONNECT;
	mt76_connac_mcu_sta_basic_tlv(&dev->mt76, sskb, &vif->bss_conf,
				      link_sta, conn_state, new_entry);
	if (enable && sta)
		mt76_connac_mcu_sta_tlv(phy->mt76, sskb, sta, vif, 0,
					MT76_STA_INFO_STATE_ASSOC);

	wtbl_hdr = mt76_connac_mcu_alloc_wtbl_req(&dev->mt76, &msta->wcid,
						  WTBL_RESET_AND_SET, NULL,
						  &wskb);
	if (IS_ERR(wtbl_hdr)) {
		dev_kfree_skb(sskb);
		return PTR_ERR(wtbl_hdr);
	}

	if (enable) {
		mt76_connac_mcu_wtbl_generic_tlv(&dev->mt76, wskb, vif, sta,
						 NULL, wtbl_hdr);
		if (sta)
			mt76_connac_mcu_wtbl_ht_tlv(&dev->mt76, wskb, sta,
						    NULL, wtbl_hdr, true, true);
		mt76_connac_mcu_wtbl_hdr_trans_tlv(wskb, vif, &msta->wcid,
						   NULL, wtbl_hdr);
	}

	cmd = enable ? MCU_EXT_CMD(WTBL_UPDATE) : MCU_EXT_CMD(STA_REC_UPDATE);
	skb = enable ? wskb : sskb;

	err = mt76_mcu_skb_send_msg(&dev->mt76, skb, cmd, true);
	if (err < 0) {
		skb = enable ? sskb : wskb;
		dev_kfree_skb(skb);

		return err;
	}

	cmd = enable ? MCU_EXT_CMD(STA_REC_UPDATE) : MCU_EXT_CMD(WTBL_UPDATE);
	skb = enable ? sskb : wskb;

	return mt76_mcu_skb_send_msg(&dev->mt76, skb, cmd, true);
}

static int
mt7615_mcu_wtbl_update_hdr_trans(struct mt7615_dev *dev,
				 struct ieee80211_vif *vif,
				 struct ieee80211_sta *sta)
{
	return mt76_connac_mcu_wtbl_update_hdr_trans(&dev->mt76, vif, sta);
}

static const struct mt7615_mcu_ops wtbl_update_ops = {
	.add_beacon_offload = mt7615_mcu_add_beacon_offload,
	.set_pm_state = mt7615_mcu_ctrl_pm_state,
	.add_dev_info = mt7615_mcu_add_dev,
	.add_bss_info = mt7615_mcu_add_bss,
	.add_tx_ba = mt7615_mcu_wtbl_tx_ba,
	.add_rx_ba = mt7615_mcu_wtbl_rx_ba,
	.sta_add = mt7615_mcu_wtbl_sta_add,
	.set_drv_ctrl = mt7615_mcu_drv_pmctrl,
	.set_fw_ctrl = mt7615_mcu_fw_pmctrl,
	.set_sta_decap_offload = mt7615_mcu_wtbl_update_hdr_trans,
};

static int
mt7615_mcu_sta_ba(struct mt7615_dev *dev,
		  struct ieee80211_ampdu_params *params,
		  bool enable, bool tx)
{
	struct mt7615_sta *msta = (struct mt7615_sta *)params->sta->drv_priv;
	struct mt7615_vif *mvif = msta->vif;
	struct wtbl_req_hdr *wtbl_hdr;
	struct tlv *sta_wtbl;
	struct sk_buff *skb;

	skb = mt76_connac_mcu_alloc_sta_req(&dev->mt76, &mvif->mt76,
					    &msta->wcid);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	mt76_connac_mcu_sta_ba_tlv(skb, params, enable, tx);

	sta_wtbl = mt76_connac_mcu_add_tlv(skb, STA_REC_WTBL, sizeof(struct tlv));

	wtbl_hdr = mt76_connac_mcu_alloc_wtbl_req(&dev->mt76, &msta->wcid,
						  WTBL_SET, sta_wtbl, &skb);
	if (IS_ERR(wtbl_hdr))
		return PTR_ERR(wtbl_hdr);

	mt76_connac_mcu_wtbl_ba_tlv(&dev->mt76, skb, params, enable, tx,
				    sta_wtbl, wtbl_hdr);

	return mt76_mcu_skb_send_msg(&dev->mt76, skb,
				     MCU_EXT_CMD(STA_REC_UPDATE), true);
}

static int
mt7615_mcu_sta_tx_ba(struct mt7615_dev *dev,
		     struct ieee80211_ampdu_params *params,
		     bool enable)
{
	return mt7615_mcu_sta_ba(dev, params, enable, true);
}

static int
mt7615_mcu_sta_rx_ba(struct mt7615_dev *dev,
		     struct ieee80211_ampdu_params *params,
		     bool enable)
{
	return mt7615_mcu_sta_ba(dev, params, enable, false);
}

static int
__mt7615_mcu_add_sta(struct mt76_phy *phy, struct ieee80211_vif *vif,
		     struct ieee80211_sta *sta, bool enable, int cmd,
		     bool offload_fw)
{
	struct mt7615_vif *mvif = (struct mt7615_vif *)vif->drv_priv;
	struct mt76_sta_cmd_info info = {
		.sta = sta,
		.vif = vif,
		.offload_fw = offload_fw,
		.enable = enable,
		.newly = true,
		.cmd = cmd,
	};

	info.wcid = sta ? (struct mt76_wcid *)sta->drv_priv : &mvif->sta.wcid;
	return mt76_connac_mcu_sta_cmd(phy, &info);
}

static int
mt7615_mcu_add_sta(struct mt7615_phy *phy, struct ieee80211_vif *vif,
		   struct ieee80211_sta *sta, bool enable)
{
	return __mt7615_mcu_add_sta(phy->mt76, vif, sta, enable,
				    MCU_EXT_CMD(STA_REC_UPDATE), false);
}

static int
mt7615_mcu_sta_update_hdr_trans(struct mt7615_dev *dev,
				struct ieee80211_vif *vif,
				struct ieee80211_sta *sta)
{
	struct mt7615_sta *msta = (struct mt7615_sta *)sta->drv_priv;

	return mt76_connac_mcu_sta_update_hdr_trans(&dev->mt76,
						    vif, &msta->wcid,
						    MCU_EXT_CMD(STA_REC_UPDATE));
}

static const struct mt7615_mcu_ops sta_update_ops = {
	.add_beacon_offload = mt7615_mcu_add_beacon_offload,
	.set_pm_state = mt7615_mcu_ctrl_pm_state,
	.add_dev_info = mt7615_mcu_add_dev,
	.add_bss_info = mt7615_mcu_add_bss,
	.add_tx_ba = mt7615_mcu_sta_tx_ba,
	.add_rx_ba = mt7615_mcu_sta_rx_ba,
	.sta_add = mt7615_mcu_add_sta,
	.set_drv_ctrl = mt7615_mcu_drv_pmctrl,
	.set_fw_ctrl = mt7615_mcu_fw_pmctrl,
	.set_sta_decap_offload = mt7615_mcu_sta_update_hdr_trans,
};

static int
mt7615_mcu_uni_ctrl_pm_state(struct mt7615_dev *dev, int band, int state)
{
	return 0;
}

static int
mt7615_mcu_uni_add_beacon_offload(struct mt7615_dev *dev,
				  struct ieee80211_hw *hw,
				  struct ieee80211_vif *vif,
				  bool enable)
{
	struct mt7615_vif *mvif = (struct mt7615_vif *)vif->drv_priv;
	struct mt76_wcid *wcid = &dev->mt76.global_wcid;
	struct ieee80211_mutable_offsets offs;
	struct {
		struct req_hdr {
			u8 bss_idx;
			u8 pad[3];
		} __packed hdr;
		struct bcn_content_tlv {
			__le16 tag;
			__le16 len;
			__le16 tim_ie_pos;
			__le16 csa_ie_pos;
			__le16 bcc_ie_pos;
			/* 0: disable beacon offload
			 * 1: enable beacon offload
			 * 2: update probe respond offload
			 */
			u8 enable;
			/* 0: legacy format (TXD + payload)
			 * 1: only cap field IE
			 */
			u8 type;
			__le16 pkt_len;
			u8 pkt[512];
		} __packed beacon_tlv;
	} req = {
		.hdr = {
			.bss_idx = mvif->mt76.idx,
		},
		.beacon_tlv = {
			.tag = cpu_to_le16(UNI_BSS_INFO_BCN_CONTENT),
			.len = cpu_to_le16(sizeof(struct bcn_content_tlv)),
			.enable = enable,
		},
	};
	struct sk_buff *skb;

	if (!enable)
		goto out;

	skb = ieee80211_beacon_get_template(mt76_hw(dev), vif, &offs, 0);
	if (!skb)
		return -EINVAL;

	if (skb->len > 512 - MT_TXD_SIZE) {
		dev_err(dev->mt76.dev, "beacon size limit exceed\n");
		dev_kfree_skb(skb);
		return -EINVAL;
	}

	mt7615_mac_write_txwi(dev, (__le32 *)(req.beacon_tlv.pkt), skb,
			      wcid, NULL, 0, NULL, 0, true);
	memcpy(req.beacon_tlv.pkt + MT_TXD_SIZE, skb->data, skb->len);
	req.beacon_tlv.pkt_len = cpu_to_le16(MT_TXD_SIZE + skb->len);
	req.beacon_tlv.tim_ie_pos = cpu_to_le16(MT_TXD_SIZE + offs.tim_offset);

	if (offs.cntdwn_counter_offs[0]) {
		u16 csa_offs;

		csa_offs = MT_TXD_SIZE + offs.cntdwn_counter_offs[0] - 4;
		req.beacon_tlv.csa_ie_pos = cpu_to_le16(csa_offs);
	}
	dev_kfree_skb(skb);

out:
	return mt76_mcu_send_msg(&dev->mt76, MCU_UNI_CMD(BSS_INFO_UPDATE),
				 &req, sizeof(req), true);
}

static int
mt7615_mcu_uni_add_dev(struct mt7615_phy *phy, struct ieee80211_vif *vif,
		       bool enable)
{
	struct mt7615_vif *mvif = (struct mt7615_vif *)vif->drv_priv;

	return mt76_connac_mcu_uni_add_dev(phy->mt76, &vif->bss_conf, &mvif->mt76,
					   &mvif->sta.wcid, enable);
}

static int
mt7615_mcu_uni_add_bss(struct mt7615_phy *phy, struct ieee80211_vif *vif,
		       struct ieee80211_sta *sta, bool enable)
{
	struct mt7615_vif *mvif = (struct mt7615_vif *)vif->drv_priv;

	return mt76_connac_mcu_uni_add_bss(phy->mt76, vif, &mvif->sta.wcid,
					   enable, NULL);
}

static inline int
mt7615_mcu_uni_add_sta(struct mt7615_phy *phy, struct ieee80211_vif *vif,
		       struct ieee80211_sta *sta, bool enable)
{
	return __mt7615_mcu_add_sta(phy->mt76, vif, sta, enable,
				    MCU_UNI_CMD(STA_REC_UPDATE), true);
}

static int
mt7615_mcu_uni_tx_ba(struct mt7615_dev *dev,
		     struct ieee80211_ampdu_params *params,
		     bool enable)
{
	struct mt7615_sta *sta = (struct mt7615_sta *)params->sta->drv_priv;

	return mt76_connac_mcu_sta_ba(&dev->mt76, &sta->vif->mt76, params,
				      MCU_UNI_CMD(STA_REC_UPDATE), enable,
				      true);
}

static int
mt7615_mcu_uni_rx_ba(struct mt7615_dev *dev,
		     struct ieee80211_ampdu_params *params,
		     bool enable)
{
	struct mt7615_sta *msta = (struct mt7615_sta *)params->sta->drv_priv;
	struct mt7615_vif *mvif = msta->vif;
	struct wtbl_req_hdr *wtbl_hdr;
	struct tlv *sta_wtbl;
	struct sk_buff *skb;
	int err;

	skb = mt76_connac_mcu_alloc_sta_req(&dev->mt76, &mvif->mt76,
					    &msta->wcid);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	mt76_connac_mcu_sta_ba_tlv(skb, params, enable, false);

	err = mt76_mcu_skb_send_msg(&dev->mt76, skb,
				    MCU_UNI_CMD(STA_REC_UPDATE), true);
	if (err < 0 || !enable)
		return err;

	skb = mt76_connac_mcu_alloc_sta_req(&dev->mt76, &mvif->mt76,
					    &msta->wcid);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	sta_wtbl = mt76_connac_mcu_add_tlv(skb, STA_REC_WTBL,
					   sizeof(struct tlv));

	wtbl_hdr = mt76_connac_mcu_alloc_wtbl_req(&dev->mt76, &msta->wcid,
						  WTBL_SET, sta_wtbl, &skb);
	if (IS_ERR(wtbl_hdr))
		return PTR_ERR(wtbl_hdr);

	mt76_connac_mcu_wtbl_ba_tlv(&dev->mt76, skb, params, enable, false,
				    sta_wtbl, wtbl_hdr);

	return mt76_mcu_skb_send_msg(&dev->mt76, skb,
				     MCU_UNI_CMD(STA_REC_UPDATE), true);
}

static int
mt7615_mcu_sta_uni_update_hdr_trans(struct mt7615_dev *dev,
				    struct ieee80211_vif *vif,
				    struct ieee80211_sta *sta)
{
	struct mt7615_sta *msta = (struct mt7615_sta *)sta->drv_priv;

	return mt76_connac_mcu_sta_update_hdr_trans(&dev->mt76,
						    vif, &msta->wcid,
						    MCU_UNI_CMD(STA_REC_UPDATE));
}

/*
 * MT6833: its firmware (CONNAC 1, MediaTek's gen4m with legacy commands) knows none of the unified
 * commands: it does not answer UNI DEV_INFO_UPDATE and raises an exception later. gen4m activates
 * a BSS with CMD_ID_BSS_ACTIVATE_CTRL (0x11); BSS info, station records and block ack go through
 * CMD_ID_SET_BSS_INFO (0x12) and CMD_ID_UPDATE_STA_RECORD (0x13), which are not done yet: what
 * needs them (association, AP mode) fails cleanly instead of reaching the firmware.
 */
static int mt6833_mcu_bss_activate(struct mt7615_phy *phy, struct ieee80211_vif *vif,
				   bool enable)
{
	struct mt7615_vif *mvif = (struct mt7615_vif *)vif->drv_priv;
	struct {
		u8 bss_idx;
		u8 active;
		u8 network_type;	/* 0: station (AIS), 1: P2P and AP */
		u8 omac_idx;
		u8 addr[ETH_ALEN];
		u8 bmc_wlan_idx;	/* the WTBL entry for broadcast and management frames */
		u8 rsv;
	} __packed req = {
		.bss_idx = mvif->mt76.idx,
		.active = enable,
		.network_type = vif->type == NL80211_IFTYPE_STATION && !vif->p2p ? 0 : 1,
		.omac_idx = mvif->mt76.omac_idx,
		.bmc_wlan_idx = mvif->sta.wcid.idx,
	};

	ether_addr_copy(req.addr, vif->addr);

	return mt76_mcu_send_msg(&phy->dev->mt76, MCU_CE_CMD(BSS_ACTIVATE_CTRL), &req,
				 sizeof(req), false);
}

/*
 * MT6833: BSS info and station records the way its firmware takes them (MediaTek's gen4m
 * CMD_SET_BSS_INFO and CMD_UPDATE_STA_RECORD, legacy commands, as built for MT6833: SOC2_1X1,
 * no 802.11ax), station mode only. A station record's index is its WTBL entry (mt76's wcid).
 */
#define MT6833_MEDIA_CONNECTED		0
#define MT6833_MEDIA_DISCONNECTED	1
#define MT6833_PHY_HR_DSSS		BIT(0)
#define MT6833_PHY_ERP			BIT(1)
#define MT6833_PHY_OFDM			BIT(3)
#define MT6833_PHY_HT			BIT(4)
#define MT6833_PHY_VHT			BIT(5)
#define MT6833_STA_STATE_1		0
#define MT6833_STA_STATE_3		2
#define MT6833_STA_LEGACY_AP		(BIT(0) | BIT(6))
#define MT6833_WTBL_NONE		0xff

struct mt6833_bss_rlm {
	u8 bss_idx;
	u8 band;		/* 1: 2.4 GHz, 2: 5 GHz */
	u8 primary_ch;
	u8 sco;			/* 0: none, 1: secondary above, 3: below */
	u8 erp_protect;
	u8 ht_protect;
	u8 gf_mode;
	u8 rifs;
	__le16 ht_op_info3;
	__le16 ht_op_info2;
	u8 ht_op_info1;
	u8 short_preamble;
	u8 short_slot;
	u8 vht_width;		/* 0: 20/40 MHz, 1: 80, 2: 160 */
	u8 vht_s1;
	u8 vht_s2;
	__le16 vht_basic_mcs;
	u8 tx_nss;
	u8 rx_nss;
} __packed;

struct mt6833_bss_info {
	u8 bss_idx;
	u8 conn_state;
	u8 op_mode;		/* 0: infrastructure */
	u8 ssid_len;
	u8 ssid[32];
	u8 bssid[ETH_ALEN];
	u8 qbss;
	u8 version;
	__le16 op_rates;
	__le16 basic_rates;
	u8 ap_sta_idx;
	u8 pad0;
	__le16 fixed_rate;
	u8 non_ht_basic_phy;
	u8 auth_mode;
	u8 enc_status;
	u8 phy_types;
	u8 wapi;
	u8 ap_mode;
	u8 bmc_wlan_idx;
	u8 hidden_ssid;
	u8 disconnect_threshold;
	u8 iot_ap_act;
	u8 pad1[2];
	__le32 private_data;
	struct mt6833_bss_rlm rlm;
	u8 dbdc_band;
	u8 wmm_set;
	u8 dbdc_action;
	u8 nss;
	u8 pad2[2];
	u8 he_op[3];
	u8 bss_color;
	__le16 he_basic_mcs;
	u8 max_bssid_ind;
	u8 mbssid_idx;
	__le64 iot_ap_act64;
	u8 pad3[4];
} __packed;

struct mt6833_sta_rec {
	u8 sta_idx;
	u8 sta_type;
	u8 addr[ETH_ALEN];
	__le16 aid;
	__le16 listen_interval;
	u8 bss_idx;
	u8 phy_types;
	__le16 rates;
	__le16 basic_rates;
	u8 qos;
	u8 uapsd;
	u8 state;
	u8 mcs;
	u8 mcs32;
	u8 version;
	u8 rx_mcs_mask[10];
	__le16 rx_highest;
	__le32 tx_rate_info;
	__le16 ht_cap;
	__le16 ht_ext_cap;
	__le32 txbf_cap;
	u8 ampdu_param;
	u8 asel;
	u8 rcpi;
	u8 need_resp;
	u8 uapsd_ac;
	u8 uapsd_sp;
	u8 wlan_idx;
	u8 bmc_wlan_idx;
	__le32 vht_cap;
	__le16 vht_rx_mcs;
	__le16 vht_rx_highest;
	__le16 vht_tx_mcs;
	__le16 vht_tx_highest;
	u8 rts_policy;
	u8 vht_op_mode;
	u8 traffic_type;
	u8 tx_gf;
	u8 tx_sgi;
	u8 tx_stbc;
	__le16 fixed_rate;
	u8 tx_ampdu;
	u8 rx_ampdu;
	u8 pad0[2];
	__le32 fixed_phy_rate;
	__le16 max_link_speed;
	__le16 min_link_speed;
	__le32 flags;
	u8 tx_ba_size;
	u8 rx_ba_size;
	u8 ba_rsv[2];
	__le16 pfmu_id;		/* 0xffff: none */
	u8 txbf[30];		/* MU, ETxBF, sounding and iBF fields: unused */
	u8 tx_amsdu_in_ampdu;
	u8 rx_amsdu_in_ampdu;
	u8 pad1[2];
	__le32 tx_max_amsdu_len;
	u8 pad2[32];
} __packed;

static_assert(sizeof(struct mt6833_bss_rlm) == 22);
static_assert(sizeof(struct mt6833_bss_info) == 116);
static_assert(sizeof(struct mt6833_sta_rec) == 168);

/* mt76's rate tables (2.4 GHz: 1, 2, 5.5, 11, then OFDM 6..54; 5 GHz: OFDM) to gen4m's rate set */
static u16 mt6833_rate_set(u32 mask, enum nl80211_band band)
{
	unsigned long m = mask;
	u16 set = 0;
	int i;

	for_each_set_bit(i, &m, 12) {
		int r = band == NL80211_BAND_2GHZ ? i : i + 4;

		set |= BIT(r < 4 ? r : r + 2);
	}
	return set;
}

static u8 mt6833_phy_types(enum nl80211_band band, struct ieee80211_sta *sta)
{
	u8 set = band == NL80211_BAND_2GHZ ? MT6833_PHY_HR_DSSS | MT6833_PHY_ERP :
					     MT6833_PHY_OFDM;

	if (!sta || sta->deflink.ht_cap.ht_supported)
		set |= MT6833_PHY_HT;
	if (band != NL80211_BAND_2GHZ && (!sta || sta->deflink.vht_cap.vht_supported))
		set |= MT6833_PHY_VHT;
	return set;
}

static int mt6833_mcu_set_bss(struct mt7615_phy *phy, struct ieee80211_vif *vif,
			      struct ieee80211_sta *sta, bool connected)
{
	struct mt7615_vif *mvif = (struct mt7615_vif *)vif->drv_priv;
	struct cfg80211_chan_def *chandef = &phy->mt76->chandef;
	enum nl80211_band band = chandef->chan->band;
	struct ieee80211_bss_conf *conf = &vif->bss_conf;
	struct mt6833_bss_info req = {
		.bss_idx = mvif->mt76.idx,
		.conn_state = connected ? MT6833_MEDIA_CONNECTED : MT6833_MEDIA_DISCONNECTED,
		.qbss = conf->qos,
		.ap_sta_idx = sta ? ((struct mt7615_sta *)sta->drv_priv)->wcid.idx : MT6833_WTBL_NONE,
		.fixed_rate = cpu_to_le16(band == NL80211_BAND_2GHZ ? 0x0000 : 0x004b),
		.non_ht_basic_phy = band == NL80211_BAND_2GHZ ? 1 : 3,
		/* frames are protected by mac80211 for now: nothing for the firmware to hold back */
		.auth_mode = 0,
		.enc_status = 1,
		.phy_types = mt6833_phy_types(band, sta),
		.bmc_wlan_idx = mvif->sta.wcid.idx,
		.wmm_set = mvif->mt76.wmm_idx,
		.nss = 1,
	};
	struct mt6833_bss_rlm *rlm = &req.rlm;
	int width = cfg80211_chandef_get_width(chandef);

	if (vif->type != NL80211_IFTYPE_STATION)
		return -EOPNOTSUPP;

	req.ssid_len = min_t(int, vif->cfg.ssid_len, sizeof(req.ssid));
	memcpy(req.ssid, vif->cfg.ssid, req.ssid_len);
	if (conf->bssid)
		ether_addr_copy(req.bssid, conf->bssid);
	req.basic_rates = cpu_to_le16(mt6833_rate_set(conf->basic_rates, band));
	req.op_rates = sta ? cpu_to_le16(mt6833_rate_set(sta->deflink.supp_rates[band], band)) :
			     req.basic_rates;

	rlm->bss_idx = req.bss_idx;
	rlm->band = band == NL80211_BAND_2GHZ ? 1 : 2;
	rlm->primary_ch = chandef->chan->hw_value;
	if (width >= 40) {
		int pos = (chandef->chan->center_freq - (chandef->center_freq1 - width / 2 + 10)) / 20;

		rlm->sco = pos % 2 ? 3 : 1;
		rlm->ht_op_info1 = rlm->sco | BIT(2);
	}
	if (width >= 80) {
		rlm->vht_width = width == 80 ? 1 : 2;
		rlm->vht_s1 = ieee80211_frequency_to_channel(chandef->center_freq1);
	}
	rlm->erp_protect = conf->use_cts_prot;
	rlm->ht_protect = conf->ht_operation_mode & IEEE80211_HT_OP_MODE_PROTECTION;
	rlm->short_preamble = conf->use_short_preamble;
	rlm->short_slot = conf->use_short_slot;
	rlm->tx_nss = 1;
	rlm->rx_nss = 1;

	return mt76_mcu_send_msg(&phy->dev->mt76, MCU_CE_CMD(SET_BSS_INFO), &req, sizeof(req),
				 false);
}

static int mt6833_mcu_set_sta(struct mt7615_phy *phy, struct ieee80211_vif *vif,
			      struct ieee80211_sta *sta, bool assoc)
{
	struct mt7615_vif *mvif = (struct mt7615_vif *)vif->drv_priv;
	struct mt7615_sta *msta = (struct mt7615_sta *)sta->drv_priv;
	enum nl80211_band band = phy->mt76->chandef.chan->band;
	struct ieee80211_sta_ht_cap *ht = &sta->deflink.ht_cap;
	struct ieee80211_sta_vht_cap *vht = &sta->deflink.vht_cap;
	struct mt6833_sta_rec req = {
		.sta_idx = msta->wcid.idx,
		.sta_type = MT6833_STA_LEGACY_AP,
		.aid = cpu_to_le16(vif->cfg.aid),
		.bss_idx = mvif->mt76.idx,
		.phy_types = mt6833_phy_types(band, sta),
		.rates = cpu_to_le16(mt6833_rate_set(sta->deflink.supp_rates[band], band)),
		.basic_rates = cpu_to_le16(mt6833_rate_set(vif->bss_conf.basic_rates, band)),
		.qos = sta->wme,
		.state = assoc ? MT6833_STA_STATE_3 : MT6833_STA_STATE_1,
		.wlan_idx = msta->wcid.idx,
		.bmc_wlan_idx = MT6833_WTBL_NONE,
		.fixed_rate = cpu_to_le16(band == NL80211_BAND_2GHZ ? 0x0000 : 0x004b),
		.pfmu_id = cpu_to_le16(0xffff),
	};

	ether_addr_copy(req.addr, sta->addr);
	if (ht->ht_supported) {
		req.mcs = ht->mcs.rx_mask[0];
		req.mcs32 = ht->mcs.rx_mask[4] & 1;
		memcpy(req.rx_mcs_mask, ht->mcs.rx_mask, sizeof(req.rx_mcs_mask));
		req.rx_highest = ht->mcs.rx_highest;
		req.ht_cap = cpu_to_le16(ht->cap);
		req.ampdu_param = ht->ampdu_factor | ht->ampdu_density << 2;
	}
	if (vht->vht_supported) {
		req.vht_cap = cpu_to_le32(vht->cap);
		req.vht_rx_mcs = vht->vht_mcs.rx_mcs_map;
		req.vht_rx_highest = vht->vht_mcs.rx_highest;
		req.vht_tx_mcs = vht->vht_mcs.tx_mcs_map;
		req.vht_tx_highest = vht->vht_mcs.tx_highest;
	}

	return mt76_mcu_send_msg(&phy->dev->mt76, MCU_CE_CMD(UPDATE_STA_RECORD), &req,
				 sizeof(req), false);
}

static int mt6833_mcu_add_bss(struct mt7615_phy *phy, struct ieee80211_vif *vif,
			      struct ieee80211_sta *sta, bool enable)
{
	if (vif->type != NL80211_IFTYPE_STATION)
		return enable ? -EOPNOTSUPP : 0;

	return mt6833_mcu_set_bss(phy, vif, enable ? sta : NULL, enable && vif->cfg.assoc);
}

static int mt6833_mcu_add_sta(struct mt7615_phy *phy, struct ieee80211_vif *vif,
			      struct ieee80211_sta *sta, bool enable)
{
	struct mt7615_vif *mvif = (struct mt7615_vif *)vif->drv_priv;
	struct {
		u8 action;	/* 0: the station */
		u8 sta_idx;
		u8 bss_idx;
		u8 rsv;
	} __packed req = {
		.bss_idx = mvif->mt76.idx,
	};

	if (!sta || vif->type != NL80211_IFTYPE_STATION)
		return enable ? -EOPNOTSUPP : 0;
	if (enable)
		return mt6833_mcu_set_sta(phy, vif, sta, false);

	req.sta_idx = ((struct mt7615_sta *)sta->drv_priv)->wcid.idx;
	return mt76_mcu_send_msg(&phy->dev->mt76, MCU_CE_CMD(REMOVE_STA_RECORD), &req,
				 sizeof(req), false);
}

/*
 * MT6833, from bss_info_changed(BSS_CHANGED_ASSOC): the AP's station record with what the
 * association brought (state 3, capabilities, AID), the BSS connected, and the power management
 * told (MediaTek's CMD_INDICATE_PM_BSS_CONNECTED/ABORT).
 */
int mt6833_mcu_set_assoc(struct mt7615_phy *phy, struct ieee80211_vif *vif)
{
	struct mt7615_vif *mvif = (struct mt7615_vif *)vif->drv_priv;
	struct mt76_dev *mdev = &phy->dev->mt76;
	struct ieee80211_sta *sta;
	struct {
		u8 bss_idx;
		u8 dtim;
		__le16 aid;
		__le16 beacon_int;
		__le16 atim;
		u8 uapsd;
		u8 delivery_ac;
		u8 trigger_ac;
		u8 rsv;
	} __packed conn = {
		.bss_idx = mvif->mt76.idx,
		.dtim = vif->bss_conf.dtim_period,
		.aid = cpu_to_le16(vif->cfg.aid),
		.beacon_int = cpu_to_le16(vif->bss_conf.beacon_int),
	};
	struct {
		u8 bss_idx;
		u8 rsv[3];
	} __packed abort = {
		.bss_idx = mvif->mt76.idx,
	};
	int ret;

	if (vif->type != NL80211_IFTYPE_STATION)
		return 0;

	if (!vif->cfg.assoc) {
		ret = mt6833_mcu_set_bss(phy, vif, NULL, false);
		if (ret)
			return ret;
		return mt76_mcu_send_msg(mdev, MCU_CE_CMD(SET_BSS_ABORT), &abort, sizeof(abort),
					 false);
	}

	rcu_read_lock();
	sta = vif->bss_conf.bssid ? ieee80211_find_sta(vif, vif->bss_conf.bssid) : NULL;
	ret = sta ? mt6833_mcu_set_sta(phy, vif, sta, true) : -ENOENT;
	if (!ret)
		ret = mt6833_mcu_set_bss(phy, vif, sta, true);
	rcu_read_unlock();
	if (ret)
		return ret;

	return mt76_mcu_send_msg(mdev, MCU_CE_CMD(SET_BSS_CONNECTED), &conn, sizeof(conn),
				 false);
}
EXPORT_SYMBOL_GPL(mt6833_mcu_set_assoc);

static int mt6833_mcu_ba(struct mt7615_dev *dev, struct ieee80211_ampdu_params *params,
			 bool enable)
{
	return enable ? -EOPNOTSUPP : 0;
}

static int mt6833_mcu_add_beacon(struct mt7615_dev *dev, struct ieee80211_hw *hw,
				 struct ieee80211_vif *vif, bool enable)
{
	return enable ? -EOPNOTSUPP : 0;
}

/* power save is the firmware's own (it reports EVENT_ID_SLEEPY_INFO) */
static int mt6833_mcu_set_pm_state(struct mt7615_dev *dev, int band, int state)
{
	return 0;
}

static int mt6833_mcu_decap_offload(struct mt7615_dev *dev, struct ieee80211_vif *vif,
				    struct ieee80211_sta *sta)
{
	return -EOPNOTSUPP;
}

static const struct mt7615_mcu_ops mt6833_update_ops = {
	.add_beacon_offload = mt6833_mcu_add_beacon,
	.set_pm_state = mt6833_mcu_set_pm_state,
	.add_dev_info = mt6833_mcu_bss_activate,
	.add_bss_info = mt6833_mcu_add_bss,
	.add_tx_ba = mt6833_mcu_ba,
	.add_rx_ba = mt6833_mcu_ba,
	.sta_add = mt6833_mcu_add_sta,
	.set_drv_ctrl = mt7615_mcu_lp_drv_pmctrl,
	.set_fw_ctrl = mt7615_mcu_fw_pmctrl,
	.set_sta_decap_offload = mt6833_mcu_decap_offload,
};

static const struct mt7615_mcu_ops uni_update_ops = {
	.add_beacon_offload = mt7615_mcu_uni_add_beacon_offload,
	.set_pm_state = mt7615_mcu_uni_ctrl_pm_state,
	.add_dev_info = mt7615_mcu_uni_add_dev,
	.add_bss_info = mt7615_mcu_uni_add_bss,
	.add_tx_ba = mt7615_mcu_uni_tx_ba,
	.add_rx_ba = mt7615_mcu_uni_rx_ba,
	.sta_add = mt7615_mcu_uni_add_sta,
	.set_drv_ctrl = mt7615_mcu_lp_drv_pmctrl,
	.set_fw_ctrl = mt7615_mcu_fw_pmctrl,
	.set_sta_decap_offload = mt7615_mcu_sta_uni_update_hdr_trans,
};

int mt7615_mcu_restart(struct mt76_dev *dev)
{
	return mt76_mcu_send_msg(dev, MCU_CMD(RESTART_DL_REQ), NULL, 0, true);
}
EXPORT_SYMBOL_GPL(mt7615_mcu_restart);

static int mt7615_load_patch(struct mt7615_dev *dev, u32 addr, const char *name)
{
	const struct mt7615_patch_hdr *hdr;
	const struct firmware *fw = NULL;
	int len, ret, sem;

	ret = firmware_request_nowarn(&fw, name, dev->mt76.dev);
	if (ret)
		return ret;

	if (!fw || !fw->data || fw->size < sizeof(*hdr)) {
		dev_err(dev->mt76.dev, "Invalid firmware\n");
		ret = -EINVAL;
		goto release_fw;
	}

	sem = mt76_connac_mcu_patch_sem_ctrl(&dev->mt76, true);
	switch (sem) {
	case PATCH_IS_DL:
		goto release_fw;
	case PATCH_NOT_DL_SEM_SUCCESS:
		break;
	default:
		dev_err(dev->mt76.dev, "Failed to get patch semaphore\n");
		ret = -EAGAIN;
		goto release_fw;
	}

	hdr = (const struct mt7615_patch_hdr *)(fw->data);

	dev_info(dev->mt76.dev, "HW/SW Version: 0x%x, Build Time: %.16s\n",
		 be32_to_cpu(hdr->hw_sw_ver), hdr->build_date);

	len = fw->size - sizeof(*hdr);

	ret = mt76_connac_mcu_init_download(&dev->mt76, addr, len,
					    DL_MODE_NEED_RSP);
	if (ret) {
		dev_err(dev->mt76.dev, "Download request failed\n");
		goto out;
	}

	ret = mt76_mcu_send_firmware(&dev->mt76, MCU_CMD(FW_SCATTER),
				     fw->data + sizeof(*hdr), len);
	if (ret) {
		dev_err(dev->mt76.dev, "Failed to send firmware to device\n");
		goto out;
	}

	ret = mt76_connac_mcu_start_patch(&dev->mt76);
	if (ret)
		dev_err(dev->mt76.dev, "Failed to start patch\n");

out:
	sem = mt76_connac_mcu_patch_sem_ctrl(&dev->mt76, false);
	switch (sem) {
	case PATCH_REL_SEM_SUCCESS:
		break;
	default:
		ret = -EAGAIN;
		dev_err(dev->mt76.dev, "Failed to release patch semaphore\n");
		break;
	}

release_fw:
	release_firmware(fw);

	return ret;
}

static int
mt7615_mcu_send_ram_firmware(struct mt7615_dev *dev,
			     const struct mt7615_fw_trailer *hdr,
			     const u8 *data, bool is_cr4)
{
	int n_region = is_cr4 ? CR4_REGION_NUM : N9_REGION_NUM;
	int err, i, offset = 0;
	u32 len, addr, mode;

	for (i = 0; i < n_region; i++) {
		mode = mt76_connac_mcu_gen_dl_mode(&dev->mt76,
						   hdr[i].feature_set, is_cr4);
		len = le32_to_cpu(hdr[i].len) + IMG_CRC_LEN;
		addr = le32_to_cpu(hdr[i].addr);

		err = mt76_connac_mcu_init_download(&dev->mt76, addr, len,
						    mode);
		if (err) {
			dev_err(dev->mt76.dev, "Download request failed\n");
			return err;
		}

		err = mt76_mcu_send_firmware(&dev->mt76, MCU_CMD(FW_SCATTER),
					     data + offset, len);
		if (err) {
			dev_err(dev->mt76.dev, "Failed to send firmware to device\n");
			return err;
		}

		offset += len;
	}

	return 0;
}

static int mt7615_load_n9(struct mt7615_dev *dev, const char *name)
{
	const struct mt7615_fw_trailer *hdr;
	const struct firmware *fw;
	int ret;

	ret = request_firmware(&fw, name, dev->mt76.dev);
	if (ret)
		return ret;

	if (!fw || !fw->data || fw->size < N9_REGION_NUM * sizeof(*hdr)) {
		dev_err(dev->mt76.dev, "Invalid firmware\n");
		ret = -EINVAL;
		goto out;
	}

	hdr = (const struct mt7615_fw_trailer *)(fw->data + fw->size -
					N9_REGION_NUM * sizeof(*hdr));

	dev_info(dev->mt76.dev, "N9 Firmware Version: %.10s, Build Time: %.15s\n",
		 hdr->fw_ver, hdr->build_date);

	ret = mt7615_mcu_send_ram_firmware(dev, hdr, fw->data, false);
	if (ret)
		goto out;

	ret = mt76_connac_mcu_start_firmware(&dev->mt76,
					     le32_to_cpu(hdr->addr),
					     FW_START_OVERRIDE);
	if (ret) {
		dev_err(dev->mt76.dev, "Failed to start N9 firmware\n");
		goto out;
	}

	snprintf(dev->mt76.hw->wiphy->fw_version,
		 sizeof(dev->mt76.hw->wiphy->fw_version),
		 "%.10s-%.15s", hdr->fw_ver, hdr->build_date);

	if (!is_mt7615(&dev->mt76)) {
		dev->fw_ver = MT7615_FIRMWARE_V2;
		dev->mcu_ops = &sta_update_ops;
	} else {
		dev->fw_ver = MT7615_FIRMWARE_V1;
		dev->mcu_ops = &wtbl_update_ops;
	}

out:
	release_firmware(fw);
	return ret;
}

static int mt7615_load_cr4(struct mt7615_dev *dev, const char *name)
{
	const struct mt7615_fw_trailer *hdr;
	const struct firmware *fw;
	int ret;

	ret = request_firmware(&fw, name, dev->mt76.dev);
	if (ret)
		return ret;

	if (!fw || !fw->data || fw->size < CR4_REGION_NUM * sizeof(*hdr)) {
		dev_err(dev->mt76.dev, "Invalid firmware\n");
		ret = -EINVAL;
		goto out;
	}

	hdr = (const struct mt7615_fw_trailer *)(fw->data + fw->size -
					CR4_REGION_NUM * sizeof(*hdr));

	dev_info(dev->mt76.dev, "CR4 Firmware Version: %.10s, Build Time: %.15s\n",
		 hdr->fw_ver, hdr->build_date);

	ret = mt7615_mcu_send_ram_firmware(dev, hdr, fw->data, true);
	if (ret)
		goto out;

	ret = mt76_connac_mcu_start_firmware(&dev->mt76, 0,
					     FW_START_WORKING_PDA_CR4);
	if (ret) {
		dev_err(dev->mt76.dev, "Failed to start CR4 firmware\n");
		goto out;
	}

out:
	release_firmware(fw);

	return ret;
}

static int mt7615_load_ram(struct mt7615_dev *dev)
{
	int ret;

	ret = mt7615_load_n9(dev, MT7615_FIRMWARE_N9);
	if (ret)
		return ret;

	return mt7615_load_cr4(dev, MT7615_FIRMWARE_CR4);
}

static int mt7615_load_firmware(struct mt7615_dev *dev)
{
	int ret;
	u32 val;

	val = mt76_get_field(dev, MT_TOP_MISC2, MT_TOP_MISC2_FW_STATE);

	if (val != FW_STATE_FW_DOWNLOAD) {
		dev_err(dev->mt76.dev, "Firmware is not ready for download\n");
		return -EIO;
	}

	ret = mt7615_load_patch(dev, MT7615_PATCH_ADDRESS, MT7615_ROM_PATCH);
	if (ret)
		return ret;

	ret = mt7615_load_ram(dev);
	if (ret)
		return ret;

	if (!mt76_poll_msec(dev, MT_TOP_MISC2, MT_TOP_MISC2_FW_STATE,
			    FIELD_PREP(MT_TOP_MISC2_FW_STATE,
				       FW_STATE_RDY), 500)) {
		dev_err(dev->mt76.dev, "Timeout for initializing firmware\n");
		return -EIO;
	}

	return 0;
}

static int mt7622_load_firmware(struct mt7615_dev *dev)
{
	int ret;
	u32 val;

	mt76_set(dev, MT_WPDMA_GLO_CFG, MT_WPDMA_GLO_CFG_BYPASS_TX_SCH);

	val = mt76_get_field(dev, MT_TOP_OFF_RSV, MT_TOP_OFF_RSV_FW_STATE);
	if (val != FW_STATE_FW_DOWNLOAD) {
		dev_err(dev->mt76.dev, "Firmware is not ready for download\n");
		return -EIO;
	}

	ret = mt7615_load_patch(dev, MT7622_PATCH_ADDRESS, MT7622_ROM_PATCH);
	if (ret)
		return ret;

	ret = mt7615_load_n9(dev, MT7622_FIRMWARE_N9);
	if (ret)
		return ret;

	if (!mt76_poll_msec(dev, MT_TOP_OFF_RSV, MT_TOP_OFF_RSV_FW_STATE,
			    FIELD_PREP(MT_TOP_OFF_RSV_FW_STATE,
				       FW_STATE_NORMAL_TRX), 1500)) {
		dev_err(dev->mt76.dev, "Timeout for initializing firmware\n");
		return -EIO;
	}

	mt76_clear(dev, MT_WPDMA_GLO_CFG, MT_WPDMA_GLO_CFG_BYPASS_TX_SCH);

	return 0;
}

int mt7615_mcu_fw_log_2_host(struct mt7615_dev *dev, u8 ctrl)
{
	struct {
		u8 ctrl_val;
		u8 pad[3];
	} data = {
		.ctrl_val = ctrl
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_EXT_CMD(FW_LOG_2_HOST),
				 &data, sizeof(data), true);
}

static int mt7615_mcu_cal_cache_apply(struct mt7615_dev *dev)
{
	struct {
		bool cache_enable;
		u8 pad[3];
	} data = {
		.cache_enable = true
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_EXT_CMD(CAL_CACHE), &data,
				 sizeof(data), false);
}

static struct mtk_connsys *mt6833_connsys(struct mt7615_dev *dev)
{
	struct device_node *np;
	struct mtk_connsys *cs;

	np = of_parse_phandle(dev->mt76.dev->of_node, "mediatek,connsys", 0);
	if (!np)
		return ERR_PTR(-EINVAL);
	cs = mtk_connsys_get(np);
	of_node_put(np);
	return cs;
}

/* MT6833: a region the firmware runs from connsys' reserved memory, copied there directly */
static int mt6833_load_emi_region(struct mt7615_dev *dev, u32 addr, const u8 *data, u32 len)
{
	struct mtk_connsys *cs = mt6833_connsys(dev);

	if (IS_ERR(cs))
		return PTR_ERR(cs);
	if (addr < MT6833_EMI_BASE)
		return -EINVAL;
	return mtk_connsys_emi_write(cs, addr - MT6833_EMI_BASE, data, len);
}

static int mt7663_load_n9(struct mt7615_dev *dev, const char *name)
{
	u32 offset = 0, override_addr = 0, flag = FW_START_DLYCAL;
	const struct mt76_connac2_fw_trailer *hdr;
	const struct mt7663_fw_buf *buf;
	const struct firmware *fw;
	const u8 *base_addr;
	int i, ret;

	ret = request_firmware(&fw, name, dev->mt76.dev);
	if (ret)
		return ret;

	if (!fw || !fw->data || fw->size < FW_V3_COMMON_TAILER_SIZE) {
		dev_err(dev->mt76.dev, "Invalid firmware\n");
		ret = -EINVAL;
		goto out;
	}

	/*
	 * MT6833 calibrates on its first start: delaying the 5 GHz calibration until EEPROM data
	 * arrives (FW_START_DLYCAL) is for efuse chips
	 */
	if (is_mt6833(&dev->mt76))
		flag = 0;

	hdr = (const void *)(fw->data + fw->size - FW_V3_COMMON_TAILER_SIZE);
	dev_info(dev->mt76.dev, "N9 Firmware Version: %.10s, Build Time: %.15s\n",
		 hdr->fw_ver, hdr->build_date);
	dev_info(dev->mt76.dev, "Region number: 0x%x\n", hdr->n_region);

	base_addr = fw->data + fw->size - FW_V3_COMMON_TAILER_SIZE;
	for (i = 0; i < hdr->n_region; i++) {
		u32 shift = (hdr->n_region - i) * FW_V3_REGION_TAILER_SIZE;
		u32 len, addr, mode;

		dev_info(dev->mt76.dev, "Parsing tailer Region: %d\n", i);

		buf = (const struct mt7663_fw_buf *)(base_addr - shift);
		mode = mt76_connac_mcu_gen_dl_mode(&dev->mt76,
						   buf->feature_set, false);
		addr = le32_to_cpu(buf->img_dest_addr);
		len = le32_to_cpu(buf->img_size);

		if (offset + len > fw->size - FW_V3_COMMON_TAILER_SIZE -
				   hdr->n_region * FW_V3_REGION_TAILER_SIZE) {
			dev_err(dev->mt76.dev, "Invalid firmware region %d\n", i);
			ret = -EINVAL;
			goto out;
		}

		if (is_mt6833(&dev->mt76) &&
		    (buf->feature_set & FW_FEATURE_DL_TO_EMI)) {
			ret = mt6833_load_emi_region(dev, addr, fw->data + offset, len);
			if (ret) {
				dev_err(dev->mt76.dev, "Region %d to EMI: %d\n", i, ret);
				goto out;
			}
			offset += len;
			continue;
		}

		ret = mt76_connac_mcu_init_download(&dev->mt76, addr, len,
						    mode);
		if (ret) {
			dev_err(dev->mt76.dev, "Download request failed\n");
			if (is_mt6833(&dev->mt76))
				mt6833_dump_regs(dev, "after");
			goto out;
		}

		ret = mt76_mcu_send_firmware(&dev->mt76, MCU_CMD(FW_SCATTER),
					     fw->data + offset, len);
		if (ret) {
			dev_err(dev->mt76.dev, "Failed to send firmware\n");
			goto out;
		}

		offset += le32_to_cpu(buf->img_size);
		if (buf->feature_set & DL_MODE_VALID_RAM_ENTRY) {
			override_addr = le32_to_cpu(buf->img_dest_addr);
			dev_info(dev->mt76.dev, "Region %d, override_addr = 0x%08x\n",
				 i, override_addr);
		}
	}

	if (override_addr)
		flag |= FW_START_OVERRIDE;

	dev_info(dev->mt76.dev, "override_addr = 0x%08x, option = %d\n",
		 override_addr, flag);

	ret = mt76_connac_mcu_start_firmware(&dev->mt76, override_addr, flag);
	if (ret) {
		dev_err(dev->mt76.dev, "Failed to start N9 firmware\n");
		goto out;
	}

	snprintf(dev->mt76.hw->wiphy->fw_version,
		 sizeof(dev->mt76.hw->wiphy->fw_version),
		 "%.10s-%.15s", hdr->fw_ver, hdr->build_date);

out:
	release_firmware(fw);

	return ret;
}

static int
mt7663_load_rom_patch(struct mt7615_dev *dev, const char **n9_firmware)
{
	const char *selected_rom, *secondary_rom = MT7663_ROM_PATCH;
	const char *primary_rom = MT7663_OFFLOAD_ROM_PATCH;
	int ret;

	if (!prefer_offload_fw) {
		secondary_rom = MT7663_OFFLOAD_ROM_PATCH;
		primary_rom = MT7663_ROM_PATCH;
	}
	selected_rom = primary_rom;

	ret = mt7615_load_patch(dev, MT7663_PATCH_ADDRESS, primary_rom);
	if (ret) {
		dev_info(dev->mt76.dev, "%s not found, switching to %s",
			 primary_rom, secondary_rom);
		ret = mt7615_load_patch(dev, MT7663_PATCH_ADDRESS,
					secondary_rom);
		if (ret) {
			dev_err(dev->mt76.dev, "failed to load %s",
				secondary_rom);
			return ret;
		}
		selected_rom = secondary_rom;
	}

	if (!strcmp(selected_rom, MT7663_OFFLOAD_ROM_PATCH)) {
		*n9_firmware = MT7663_OFFLOAD_FIRMWARE_N9;
		dev->fw_ver = MT7615_FIRMWARE_V3;
		dev->mcu_ops = &uni_update_ops;
	} else {
		*n9_firmware = MT7663_FIRMWARE_N9;
		dev->fw_ver = MT7615_FIRMWARE_V2;
		dev->mcu_ops = &sta_update_ops;
	}

	return 0;
}

int __mt7663_load_firmware(struct mt7615_dev *dev)
{
	const char *n9_firmware;
	int ret;

	ret = mt76_get_field(dev, MT_CONN_ON_MISC, MT_TOP_MISC2_FW_N9_RDY);
	if (ret) {
		dev_dbg(dev->mt76.dev, "Firmware is already download\n");
		return -EIO;
	}

	ret = mt7663_load_rom_patch(dev, &n9_firmware);
	if (ret)
		return ret;

	ret = mt7663_load_n9(dev, n9_firmware);
	if (ret)
		return ret;

	if (!mt76_poll_msec(dev, MT_CONN_ON_MISC, MT_TOP_MISC2_FW_N9_RDY,
			    MT_TOP_MISC2_FW_N9_RDY, 1500)) {
		ret = mt76_get_field(dev, MT_CONN_ON_MISC,
				     MT7663_TOP_MISC2_FW_STATE);
		dev_err(dev->mt76.dev, "Timeout for initializing firmware\n");
		return -EIO;
	}

#ifdef CONFIG_PM
	if (mt7615_firmware_offload(dev))
		dev->mt76.hw->wiphy->wowlan = &mt76_connac_wowlan_support;
#endif /* CONFIG_PM */

	dev_dbg(dev->mt76.dev, "Firmware init done\n");

	return 0;
}
EXPORT_SYMBOL_GPL(__mt7663_load_firmware);

static int mt7663_load_firmware(struct mt7615_dev *dev)
{
	int ret;

	mt76_set(dev, MT_WPDMA_GLO_CFG, MT_WPDMA_GLO_CFG_BYPASS_TX_SCH);

	ret = __mt7663_load_firmware(dev);
	if (ret)
		return ret;

	mt76_clear(dev, MT_WPDMA_GLO_CFG, MT_WPDMA_GLO_CFG_BYPASS_TX_SCH);

	return 0;
}

/* the ROM patch is in place already (connsys copies it); the client-offload firmware */
static int mt6833_load_firmware(struct mt7615_dev *dev)
{
	int ret;

	if (mt76_get_field(dev, MT_CONN_ON_MISC, MT_TOP_MISC2_FW_N9_RDY)) {
		dev_err(dev->mt76.dev, "Firmware is already running\n");
		return -EIO;
	}

	dev->fw_ver = MT7615_FIRMWARE_V3;
	dev->mcu_ops = &mt6833_update_ops;

	mt76_set(dev, MT_WPDMA_GLO_CFG, MT_WPDMA_GLO_CFG_BYPASS_TX_SCH);
	mt6833_dump_regs(dev, "before");

	ret = mt7663_load_n9(dev, MT6833_FIRMWARE_N9);
	if (ret)
		return ret;

	if (!mt76_poll_msec(dev, MT_CONN_ON_MISC, MT_TOP_MISC2_FW_N9_RDY,
			    MT_TOP_MISC2_FW_N9_RDY, 1500)) {
		dev_err(dev->mt76.dev, "Timeout for initializing firmware (0x%08x)\n",
			mt76_rr(dev, MT_CONN_ON_MISC));
		return -EIO;
	}

	mt76_clear(dev, MT_WPDMA_GLO_CFG, MT_WPDMA_GLO_CFG_BYPASS_TX_SCH);

	return 0;
}

/*
 * MT6833: what the firmware says about itself (MediaTek's wlanQueryNicCapabilityV2(), right after
 * the firmware start). What matters here is TAG_CAP_SINGLE_SKU: without it, the firmware takes the
 * regulatory domain as sub-bands and knows no per-rate power limits.
 */
static int mt6833_mcu_get_nic_capability(struct mt7615_dev *dev)
{
	struct {
		__le16 n_element;
		u8 rsv[2];
	} __packed *hdr;
	DECLARE_BITMAP(tags, 64) = {};
	struct sk_buff *skb;
	int ret, i;

	ret = mt76_mcu_send_and_get_msg(&dev->mt76, MCU_CE_QUERY(GET_NIC_CAPAB), NULL, 0,
					true, &skb);
	if (ret)
		return ret;

	skb_pull(skb, sizeof(struct mt7615_mcu_rxd));
	hdr = (void *)skb->data;
	if (skb->len < sizeof(*hdr)) {
		ret = -EINVAL;
		goto out;
	}
	skb_pull(skb, sizeof(*hdr));

	for (i = 0; i < le16_to_cpu(hdr->n_element); i++) {
		struct {
			__le32 type;
			__le32 len;
		} __packed *tlv = (void *)skb->data;
		u32 type, len;

		if (skb->len < sizeof(*tlv))
			break;
		type = le32_to_cpu(tlv->type);
		len = le32_to_cpu(tlv->len);
		skb_pull(skb, sizeof(*tlv));
		if (skb->len < len)
			break;

		if (type < 64)
			__set_bit(type, tags);
		if (type == MT_NIC_CAP_SINGLE_SKU)
			dev->single_sku = true;
		skb_pull(skb, len);
	}
	dev_info(dev->mt76.dev, "firmware capabilities %*pbl, single SKU %d\n", 64, tags,
		 dev->single_sku);
out:
	dev_kfree_skb(skb);
	return ret;
}

/* MediaTek's regulatory classes for the channel sets of CMD_SET_DOMAIN_INFO */
static u8 mt6833_reg_class(u8 ch)
{
	if (ch <= 13)
		return 81;
	if (ch == 14)
		return 82;
	if (ch <= 48)
		return 115;
	if (ch <= 64)
		return 118;
	if (ch <= 144)
		return 121;
	return 125;
}

/*
 * MT6833 without single SKU: the regulatory domain as runs of channels (MediaTek's
 * CMD_SET_DOMAIN_INFO, rlmDomainSendDomainInfoCmd()). Its firmware asserts in rlm/domain.c on
 * the channel list of mt76_connac_mcu_set_channel_domain() (CMD_SET_DOMAIN_INFO_V2).
 */
static int mt6833_mcu_set_domain_info(struct mt7615_phy *phy)
{
	struct mt7615_dev *dev = phy->dev;
	struct ieee80211_supported_band *sband[] = {
		&phy->mt76->sband_2g.sband, &phy->mt76->sband_5g.sband,
	};
	struct mt6833_subband {
		u8 reg_class;
		u8 band;	/* 1: 2.4 GHz, 2: 5 GHz */
		u8 span;	/* channel number step: 1 (5 MHz) or 4 (20 MHz) */
		u8 first;
		u8 num;
		u8 dfs;
		u8 rsv[2];
	} __packed *s = NULL;
	struct {
		__le16 country;		/* alpha2[0] << 8 | alpha2[1] */
		__le16 passive_scan;
		struct mt6833_subband sub[6];
		u8 bw_2g;	/* 0: 20/40 MHz, as MediaTek's driver */
		u8 bw_5g;
		u8 rsv[2];
	} __packed req = {
		.country = cpu_to_le16(dev->mt76.alpha2[0] << 8 | dev->mt76.alpha2[1]),
	};
	int b, i, n = 0;

	for (b = 0; b < ARRAY_SIZE(sband); b++) {
		for (i = 0; i < sband[b]->n_channels; i++) {
			struct ieee80211_channel *chan = &sband[b]->channels[i];
			u8 ch = chan->hw_value, class = mt6833_reg_class(ch);
			bool dfs = chan->flags & IEEE80211_CHAN_RADAR;

			/*
			 * six sub-bands: 2.4 GHz and the five 5 GHz sets; channel 14 (802.11b
			 * only, Japan) would take the room of one of them
			 */
			if ((chan->flags & IEEE80211_CHAN_DISABLED) || ch == 14)
				continue;
			if (s && s->reg_class == class && s->dfs == dfs &&
			    ch == s->first + s->num * s->span) {
				s->num++;
				continue;
			}
			if (n == ARRAY_SIZE(req.sub)) {
				dev_warn(dev->mt76.dev, "domain: no room for channel %u\n", ch);
				continue;
			}
			s = &req.sub[n++];
			s->reg_class = class;
			s->band = b + 1;
			s->span = b ? 4 : 1;
			s->first = ch;
			s->num = 1;
			s->dfs = dfs;
		}
	}

	return mt76_mcu_send_msg(&dev->mt76, MCU_CE_CMD(SET_CHAN_DOMAIN), &req, sizeof(req),
				 false);
}

/*
 * prototype, for A/B tests: MT6833 commands to leave out or replace (bit mask)
 * bit 0: EXT MAC_INIT_CTRL (0x46) at start and stop
 * bit 1: EXT CHANNEL_SWITCH and SET_RX_PATH (0x08, 0x4e)
 * bit 2: WMM through CMD_ID_UPDATE_WMM_PARMS (CE 0x1d, as gen4m) instead of EXT EDCA_UPDATE (0x27)
 */
static uint mt6833_skip;
module_param(mt6833_skip, uint, 0644);
MODULE_PARM_DESC(mt6833_skip, "MT6833 prototype: 1 no MAC_INIT_CTRL, 2 no channel commands, 4 WMM by CE 0x1d");

bool mt6833_dbg_skip(struct mt7615_dev *dev, unsigned int bit)
{
	return is_mt6833(&dev->mt76) && (mt6833_skip & bit);
}

/* gen4m's CMD_UPDATE_WMM_PARMS: all four ACs at once, in its order BE, BK, VI, VO */
int mt6833_mcu_set_wmm(struct mt7615_dev *dev, struct ieee80211_vif *vif, u16 queue,
		       const struct ieee80211_tx_queue_params *params)
{
	static const u8 to_gen4m[IEEE80211_NUM_ACS] = {
		[IEEE80211_AC_BE] = 0, [IEEE80211_AC_BK] = 1,
		[IEEE80211_AC_VI] = 2, [IEEE80211_AC_VO] = 3,
	};
	struct mt7615_vif *mvif = (struct mt7615_vif *)vif->drv_priv;
	struct {
		struct {
			__le16 cw_min;
			__le16 cw_max;
			__le16 txop;
			__le16 aifsn;
			u8 guard_time;
			u8 acm;
		} __packed ac[IEEE80211_NUM_ACS];
		u8 bss_idx;
		u8 qbss;
		u8 wmm_set;
		u8 rsv;
	} __packed req = {
		.bss_idx = mvif->mt76.idx,
		.qbss = vif->bss_conf.qos,
		.wmm_set = mvif->mt76.wmm_idx,
	};
	int i;

	if (queue >= IEEE80211_NUM_ACS)
		return -EINVAL;
	dev->mt6833_wmm[queue] = *params;
	dev->mt6833_wmm_valid |= BIT(queue);

	for (i = 0; i < IEEE80211_NUM_ACS; i++) {
		/* 802.11 defaults until mac80211 has set an AC */
		static const struct ieee80211_tx_queue_params def[IEEE80211_NUM_ACS] = {
			[IEEE80211_AC_VO] = { .aifs = 2, .cw_min = 3, .cw_max = 7, .txop = 47 },
			[IEEE80211_AC_VI] = { .aifs = 2, .cw_min = 7, .cw_max = 15, .txop = 94 },
			[IEEE80211_AC_BE] = { .aifs = 3, .cw_min = 15, .cw_max = 1023 },
			[IEEE80211_AC_BK] = { .aifs = 7, .cw_min = 15, .cw_max = 1023 },
		};
		const struct ieee80211_tx_queue_params *p =
			dev->mt6833_wmm_valid & BIT(i) ? &dev->mt6833_wmm[i] : &def[i];
		u8 j = to_gen4m[i];

		req.ac[j].cw_min = cpu_to_le16(p->cw_min);
		req.ac[j].cw_max = cpu_to_le16(p->cw_max);
		req.ac[j].txop = cpu_to_le16(p->txop);
		req.ac[j].aifsn = cpu_to_le16(p->aifs);
	}

	return mt76_mcu_send_msg(&dev->mt76, MCU_CE_CMD(SET_EDCA_PARMS), &req, sizeof(req),
				 false);
}

/* prototype, for the A/B test: mt76's channel list, without per-rate power limits */
static bool mt6833_domain_v2;
module_param(mt6833_domain_v2, bool, 0644);
MODULE_PARM_DESC(mt6833_domain_v2, "MT6833 prototype: send CMD_SET_DOMAIN_INFO_V2, no power limits");

int mt7615_mcu_set_channel_domain(struct mt7615_phy *phy)
{
	if (is_mt6833(&phy->dev->mt76) && !phy->dev->single_sku && !mt6833_domain_v2)
		return mt6833_mcu_set_domain_info(phy);

	return mt76_connac_mcu_set_channel_domain(phy->mt76);
}
EXPORT_SYMBOL_GPL(mt7615_mcu_set_channel_domain);

int mt7615_mcu_set_rate_txpower(struct mt7615_phy *phy)
{
	/* without single SKU the firmware keeps the NVRAM's target powers */
	if (is_mt6833(&phy->dev->mt76) && (!phy->dev->single_sku || mt6833_domain_v2))
		return 0;

	return mt76_connac_mcu_set_rate_txpower(phy->mt76);
}
EXPORT_SYMBOL_GPL(mt7615_mcu_set_rate_txpower);

/* DEBUG: MT6833 firmware log to the host: 2 (as mt76 fw_debug) or 0 */
static uint mt6833_fwlog;
module_param(mt6833_fwlog, uint, 0444);
MODULE_PARM_DESC(mt6833_fwlog, "MT6833 debug: firmware log to the host (FW_LOG_2_HOST value, 2 works)");

int mt7615_mcu_init(struct mt7615_dev *dev)
{
	static const struct mt76_mcu_ops mt7615_mcu_ops = {
		.headroom = sizeof(struct mt7615_mcu_txd),
		.mcu_skb_send_msg = mt7615_mcu_send_message,
		.mcu_parse_response = mt7615_mcu_parse_response,
	};
	int ret;

	dev->mt76.mcu_ops = &mt7615_mcu_ops;

	ret = mt7615_mcu_drv_pmctrl(dev);
	if (ret)
		return ret;

	switch (mt76_chip(&dev->mt76)) {
	case 0x7622:
		ret = mt7622_load_firmware(dev);
		break;
	case 0x7663:
		ret = mt7663_load_firmware(dev);
		break;
	case 0x6833:
		ret = mt6833_load_firmware(dev);
		break;
	default:
		ret = mt7615_load_firmware(dev);
		break;
	}
	if (ret)
		return ret;

	mt76_queue_tx_cleanup(dev, dev->mt76.q_mcu[MT_MCUQ_FWDL], false);
	dev_dbg(dev->mt76.dev, "Firmware init done\n");
	set_bit(MT76_STATE_MCU_RUNNING, &dev->mphy.state);

	if (dev->dbdc_support) {
		ret = mt7615_mcu_cal_cache_apply(dev);
		if (ret)
			return ret;
	}

	ret = mt7615_mcu_fw_log_2_host(dev, is_mt6833(&dev->mt76) ? mt6833_fwlog : 0);
	if (ret || !is_mt6833(&dev->mt76))
		return ret;

	return mt6833_mcu_get_nic_capability(dev);
}
EXPORT_SYMBOL_GPL(mt7615_mcu_init);

void mt7615_mcu_exit(struct mt7615_dev *dev)
{
	mt7615_mcu_restart(&dev->mt76);
	mt7615_mcu_set_fw_ctrl(dev);
	skb_queue_purge(&dev->mt76.mcu.res_q);
}
EXPORT_SYMBOL_GPL(mt7615_mcu_exit);

int mt7615_mcu_set_eeprom(struct mt7615_dev *dev)
{
	struct {
		u8 buffer_mode;
		u8 content_format;
		__le16 len;
	} __packed req_hdr = {
		.buffer_mode = 1,
	};
	u8 *eep = (u8 *)dev->mt76.eeprom.data;
	struct sk_buff *skb;
	int eep_len, offset;

	switch (mt76_chip(&dev->mt76)) {
	case 0x6833:
		/* the board's NVRAM file, whole; the firmware sends no answer */
		if (!dev->nvram)
			return 0;
		return mt76_mcu_send_msg(&dev->mt76, MCU_CE_CMD(SET_NVRAM),
					 dev->nvram, MT6833_NVRAM_SIZE, false);
	case 0x7622:
		eep_len = MT7622_EE_MAX - MT_EE_NIC_CONF_0;
		offset = MT_EE_NIC_CONF_0;
		break;
	case 0x7663:
		eep_len = MT7663_EE_MAX - MT_EE_CHIP_ID;
		req_hdr.content_format = 1;
		offset = MT_EE_CHIP_ID;
		break;
	default:
		eep_len = MT7615_EE_MAX - MT_EE_NIC_CONF_0;
		offset = MT_EE_NIC_CONF_0;
		break;
	}

	req_hdr.len = cpu_to_le16(eep_len);

	skb = mt76_mcu_msg_alloc(&dev->mt76, NULL, sizeof(req_hdr) + eep_len);
	if (!skb)
		return -ENOMEM;

	skb_put_data(skb, &req_hdr, sizeof(req_hdr));
	skb_put_data(skb, eep + offset, eep_len);

	return mt76_mcu_skb_send_msg(&dev->mt76, skb,
				     MCU_EXT_CMD(EFUSE_BUFFER_MODE), true);
}

int mt7615_mcu_set_wmm(struct mt7615_dev *dev, u8 queue,
		       const struct ieee80211_tx_queue_params *params)
{
#define WMM_AIFS_SET	BIT(0)
#define WMM_CW_MIN_SET	BIT(1)
#define WMM_CW_MAX_SET	BIT(2)
#define WMM_TXOP_SET	BIT(3)
#define WMM_PARAM_SET	(WMM_AIFS_SET | WMM_CW_MIN_SET | \
			 WMM_CW_MAX_SET | WMM_TXOP_SET)
	struct req_data {
		u8 number;
		u8 rsv[3];
		u8 queue;
		u8 valid;
		u8 aifs;
		u8 cw_min;
		__le16 cw_max;
		__le16 txop;
	} __packed req = {
		.number = 1,
		.queue = queue,
		.valid = WMM_PARAM_SET,
		.aifs = params->aifs,
		.cw_min = 5,
		.cw_max = cpu_to_le16(10),
		.txop = cpu_to_le16(params->txop),
	};

	if (params->cw_min)
		req.cw_min = fls(params->cw_min);
	if (params->cw_max)
		req.cw_max = cpu_to_le16(fls(params->cw_max));

	return mt76_mcu_send_msg(&dev->mt76, MCU_EXT_CMD(EDCA_UPDATE),
				 &req, sizeof(req), true);
}

int mt7615_mcu_set_dbdc(struct mt7615_dev *dev)
{
	struct mt7615_phy *ext_phy = mt7615_ext_phy(dev);
	struct dbdc_entry {
		u8 type;
		u8 index;
		u8 band;
		u8 _rsv;
	};
	struct {
		u8 enable;
		u8 num;
		u8 _rsv[2];
		struct dbdc_entry entry[64];
	} req = {
		.enable = !!ext_phy,
	};
	int i;

	if (!ext_phy)
		goto out;

#define ADD_DBDC_ENTRY(_type, _idx, _band)		\
	do { \
		req.entry[req.num].type = _type;		\
		req.entry[req.num].index = _idx;		\
		req.entry[req.num++].band = _band;		\
	} while (0)

	for (i = 0; i < 4; i++) {
		bool band = !!(ext_phy->omac_mask & BIT_ULL(i));

		ADD_DBDC_ENTRY(DBDC_TYPE_BSS, i, band);
	}

	for (i = 0; i < 14; i++) {
		bool band = !!(ext_phy->omac_mask & BIT_ULL(0x11 + i));

		ADD_DBDC_ENTRY(DBDC_TYPE_MBSS, i, band);
	}

	ADD_DBDC_ENTRY(DBDC_TYPE_MU, 0, 1);

	for (i = 0; i < 3; i++)
		ADD_DBDC_ENTRY(DBDC_TYPE_BF, i, 1);

	ADD_DBDC_ENTRY(DBDC_TYPE_WMM, 0, 0);
	ADD_DBDC_ENTRY(DBDC_TYPE_WMM, 1, 0);
	ADD_DBDC_ENTRY(DBDC_TYPE_WMM, 2, 1);
	ADD_DBDC_ENTRY(DBDC_TYPE_WMM, 3, 1);

	ADD_DBDC_ENTRY(DBDC_TYPE_MGMT, 0, 0);
	ADD_DBDC_ENTRY(DBDC_TYPE_MGMT, 1, 1);

out:
	return mt76_mcu_send_msg(&dev->mt76, MCU_EXT_CMD(DBDC_CTRL), &req,
				 sizeof(req), true);
}

int mt7615_mcu_set_fcc5_lpn(struct mt7615_dev *dev, int val)
{
	struct {
		__le16 tag;
		__le16 min_lpn;
	} req = {
		.tag = cpu_to_le16(0x1),
		.min_lpn = cpu_to_le16(val),
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_EXT_CMD(SET_RADAR_TH),
				 &req, sizeof(req), true);
}

int mt7615_mcu_set_pulse_th(struct mt7615_dev *dev,
			    const struct mt7615_dfs_pulse *pulse)
{
	struct {
		__le16 tag;
		__le32 max_width;	/* us */
		__le32 max_pwr;		/* dbm */
		__le32 min_pwr;		/* dbm */
		__le32 min_stgr_pri;	/* us */
		__le32 max_stgr_pri;	/* us */
		__le32 min_cr_pri;	/* us */
		__le32 max_cr_pri;	/* us */
	} req = {
		.tag = cpu_to_le16(0x3),
#define __req_field(field) .field = cpu_to_le32(pulse->field)
		__req_field(max_width),
		__req_field(max_pwr),
		__req_field(min_pwr),
		__req_field(min_stgr_pri),
		__req_field(max_stgr_pri),
		__req_field(min_cr_pri),
		__req_field(max_cr_pri),
#undef  __req_field
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_EXT_CMD(SET_RADAR_TH),
				 &req, sizeof(req), true);
}

int mt7615_mcu_set_radar_th(struct mt7615_dev *dev, int index,
			    const struct mt7615_dfs_pattern *pattern)
{
	struct {
		__le16 tag;
		__le16 radar_type;
		u8 enb;
		u8 stgr;
		u8 min_crpn;
		u8 max_crpn;
		u8 min_crpr;
		u8 min_pw;
		u8 max_pw;
		__le32 min_pri;
		__le32 max_pri;
		u8 min_crbn;
		u8 max_crbn;
		u8 min_stgpn;
		u8 max_stgpn;
		u8 min_stgpr;
	} req = {
		.tag = cpu_to_le16(0x2),
		.radar_type = cpu_to_le16(index),
#define __req_field_u8(field) .field = pattern->field
#define __req_field_u32(field) .field = cpu_to_le32(pattern->field)
		__req_field_u8(enb),
		__req_field_u8(stgr),
		__req_field_u8(min_crpn),
		__req_field_u8(max_crpn),
		__req_field_u8(min_crpr),
		__req_field_u8(min_pw),
		__req_field_u8(max_pw),
		__req_field_u32(min_pri),
		__req_field_u32(max_pri),
		__req_field_u8(min_crbn),
		__req_field_u8(max_crbn),
		__req_field_u8(min_stgpn),
		__req_field_u8(max_stgpn),
		__req_field_u8(min_stgpr),
#undef __req_field_u8
#undef __req_field_u32
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_EXT_CMD(SET_RADAR_TH),
				 &req, sizeof(req), true);
}

int mt7615_mcu_rdd_send_pattern(struct mt7615_dev *dev)
{
	struct {
		u8 pulse_num;
		u8 rsv[3];
		struct {
			__le32 start_time;
			__le16 width;
			__le16 power;
		} pattern[32];
	} req = {
		.pulse_num = dev->radar_pattern.n_pulses,
	};
	u32 start_time = ktime_to_ms(ktime_get_boottime());
	int i;

	if (dev->radar_pattern.n_pulses > ARRAY_SIZE(req.pattern))
		return -EINVAL;

	/* TODO: add some noise here */
	for (i = 0; i < dev->radar_pattern.n_pulses; i++) {
		u32 ts = start_time + i * dev->radar_pattern.period;

		req.pattern[i].width = cpu_to_le16(dev->radar_pattern.width);
		req.pattern[i].power = cpu_to_le16(dev->radar_pattern.power);
		req.pattern[i].start_time = cpu_to_le32(ts);
	}

	return mt76_mcu_send_msg(&dev->mt76, MCU_EXT_CMD(SET_RDD_PATTERN),
				 &req, sizeof(req), false);
}

static void mt7615_mcu_set_txpower_sku(struct mt7615_phy *phy, u8 *sku)
{
	struct mt76_phy *mphy = phy->mt76;
	struct ieee80211_hw *hw = mphy->hw;
	struct mt76_power_limits limits;
	s8 *limits_array = (s8 *)&limits;
	int n_chains = hweight8(mphy->antenna_mask);
	int tx_power = hw->conf.power_level * 2;
	int i;
	static const u8 sku_mapping[] = {
#define SKU_FIELD(_type, _field) \
		[MT_SKU_##_type] = offsetof(struct mt76_power_limits, _field)
		SKU_FIELD(CCK_1_2, cck[0]),
		SKU_FIELD(CCK_55_11, cck[2]),
		SKU_FIELD(OFDM_6_9, ofdm[0]),
		SKU_FIELD(OFDM_12_18, ofdm[2]),
		SKU_FIELD(OFDM_24_36, ofdm[4]),
		SKU_FIELD(OFDM_48, ofdm[6]),
		SKU_FIELD(OFDM_54, ofdm[7]),
		SKU_FIELD(HT20_0_8, mcs[0][0]),
		SKU_FIELD(HT20_32, ofdm[0]),
		SKU_FIELD(HT20_1_2_9_10, mcs[0][1]),
		SKU_FIELD(HT20_3_4_11_12, mcs[0][3]),
		SKU_FIELD(HT20_5_13, mcs[0][5]),
		SKU_FIELD(HT20_6_14, mcs[0][6]),
		SKU_FIELD(HT20_7_15, mcs[0][7]),
		SKU_FIELD(HT40_0_8, mcs[1][0]),
		SKU_FIELD(HT40_32, ofdm[0]),
		SKU_FIELD(HT40_1_2_9_10, mcs[1][1]),
		SKU_FIELD(HT40_3_4_11_12, mcs[1][3]),
		SKU_FIELD(HT40_5_13, mcs[1][5]),
		SKU_FIELD(HT40_6_14, mcs[1][6]),
		SKU_FIELD(HT40_7_15, mcs[1][7]),
		SKU_FIELD(VHT20_0, mcs[0][0]),
		SKU_FIELD(VHT20_1_2, mcs[0][1]),
		SKU_FIELD(VHT20_3_4, mcs[0][3]),
		SKU_FIELD(VHT20_5_6, mcs[0][5]),
		SKU_FIELD(VHT20_7, mcs[0][7]),
		SKU_FIELD(VHT20_8, mcs[0][8]),
		SKU_FIELD(VHT20_9, mcs[0][9]),
		SKU_FIELD(VHT40_0, mcs[1][0]),
		SKU_FIELD(VHT40_1_2, mcs[1][1]),
		SKU_FIELD(VHT40_3_4, mcs[1][3]),
		SKU_FIELD(VHT40_5_6, mcs[1][5]),
		SKU_FIELD(VHT40_7, mcs[1][7]),
		SKU_FIELD(VHT40_8, mcs[1][8]),
		SKU_FIELD(VHT40_9, mcs[1][9]),
		SKU_FIELD(VHT80_0, mcs[2][0]),
		SKU_FIELD(VHT80_1_2, mcs[2][1]),
		SKU_FIELD(VHT80_3_4, mcs[2][3]),
		SKU_FIELD(VHT80_5_6, mcs[2][5]),
		SKU_FIELD(VHT80_7, mcs[2][7]),
		SKU_FIELD(VHT80_8, mcs[2][8]),
		SKU_FIELD(VHT80_9, mcs[2][9]),
		SKU_FIELD(VHT160_0, mcs[3][0]),
		SKU_FIELD(VHT160_1_2, mcs[3][1]),
		SKU_FIELD(VHT160_3_4, mcs[3][3]),
		SKU_FIELD(VHT160_5_6, mcs[3][5]),
		SKU_FIELD(VHT160_7, mcs[3][7]),
		SKU_FIELD(VHT160_8, mcs[3][8]),
		SKU_FIELD(VHT160_9, mcs[3][9]),
#undef SKU_FIELD
	};

	tx_power = mt76_get_sar_power(mphy, mphy->chandef.chan, tx_power);
	tx_power -= mt76_tx_power_path_delta(n_chains);
	tx_power = mt76_get_rate_power_limits(mphy, mphy->chandef.chan,
					      &limits, tx_power);
	mphy->txpower_cur = tx_power;

	if (is_mt7663(mphy->dev)) {
		memset(sku, tx_power, MT_SKU_4SS_DELTA + 1);
		return;
	}

	for (i = 0; i < MT_SKU_1SS_DELTA; i++)
		sku[i] = limits_array[sku_mapping[i]];

	for (i = 0; i < 4; i++) {
		int delta = 0;

		if (i < n_chains - 1)
			delta = mt76_tx_power_path_delta(n_chains) -
				mt76_tx_power_path_delta(i + 1);
		sku[MT_SKU_1SS_DELTA + i] = delta;
	}
}

static u8 mt7615_mcu_chan_bw(struct cfg80211_chan_def *chandef)
{
	static const u8 width_to_bw[] = {
		[NL80211_CHAN_WIDTH_40] = CMD_CBW_40MHZ,
		[NL80211_CHAN_WIDTH_80] = CMD_CBW_80MHZ,
		[NL80211_CHAN_WIDTH_80P80] = CMD_CBW_8080MHZ,
		[NL80211_CHAN_WIDTH_160] = CMD_CBW_160MHZ,
		[NL80211_CHAN_WIDTH_5] = CMD_CBW_5MHZ,
		[NL80211_CHAN_WIDTH_10] = CMD_CBW_10MHZ,
		[NL80211_CHAN_WIDTH_20] = CMD_CBW_20MHZ,
		[NL80211_CHAN_WIDTH_20_NOHT] = CMD_CBW_20MHZ,
	};

	if (chandef->width >= ARRAY_SIZE(width_to_bw))
		return 0;

	return width_to_bw[chandef->width];
}

int mt7615_mcu_set_chan_info(struct mt7615_phy *phy, int cmd)
{
	struct mt7615_dev *dev = phy->dev;
	struct cfg80211_chan_def *chandef = &phy->mt76->chandef;
	int freq1 = chandef->center_freq1, freq2 = chandef->center_freq2;
	struct {
		u8 control_chan;
		u8 center_chan;
		u8 bw;
		u8 tx_streams;
		u8 rx_streams_mask;
		u8 switch_reason;
		u8 band_idx;
		/* for 80+80 only */
		u8 center_chan2;
		__le16 cac_case;
		u8 channel_band;
		u8 rsv0;
		__le32 outband_freq;
		u8 txpower_drop;
		u8 rsv1[3];
		u8 txpower_sku[53];
		u8 rsv2[3];
	} req = {
		.control_chan = chandef->chan->hw_value,
		.center_chan = ieee80211_frequency_to_channel(freq1),
		.tx_streams = hweight8(phy->mt76->antenna_mask),
		.rx_streams_mask = phy->mt76->chainmask,
		.center_chan2 = ieee80211_frequency_to_channel(freq2),
	};

	if (mt6833_dbg_skip(phy->dev, BIT(1)))
		return 0;

	if (cmd == MCU_EXT_CMD(SET_RX_PATH) ||
	    phy->mt76->hw->conf.flags & IEEE80211_CONF_MONITOR)
		req.switch_reason = CH_SWITCH_NORMAL;
	else if (phy->mt76->offchannel)
		req.switch_reason = CH_SWITCH_SCAN_BYPASS_DPD;
	else if (!cfg80211_reg_can_beacon(phy->mt76->hw->wiphy, chandef,
					  NL80211_IFTYPE_AP))
		req.switch_reason = CH_SWITCH_DFS;
	else
		req.switch_reason = CH_SWITCH_NORMAL;

	req.band_idx = phy != &dev->phy;
	req.bw = mt7615_mcu_chan_bw(chandef);

	if (mt76_testmode_enabled(phy->mt76))
		memset(req.txpower_sku, 0x3f, 49);
	else
		mt7615_mcu_set_txpower_sku(phy, req.txpower_sku);

	return mt76_mcu_send_msg(&dev->mt76, cmd, &req, sizeof(req), true);
}

int mt7615_mcu_get_temperature(struct mt7615_dev *dev)
{
	struct {
		u8 action;
		u8 rsv[3];
	} req = {};

	return mt76_mcu_send_msg(&dev->mt76, MCU_EXT_CMD(THERMAL_CTRL),
				 &req, sizeof(req), true);
}

int mt7615_mcu_set_test_param(struct mt7615_dev *dev, u8 param, bool test_mode,
			      u32 val)
{
	struct {
		u8 test_mode_en;
		u8 param_idx;
		u8 _rsv[2];

		__le32 value;

		u8 pad[8];
	} req = {
		.test_mode_en = test_mode,
		.param_idx = param,
		.value = cpu_to_le32(val),
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_EXT_CMD(ATE_CTRL),
				 &req, sizeof(req), false);
}

int mt7615_mcu_set_sku_en(struct mt7615_phy *phy, bool enable)
{
	struct mt7615_dev *dev = phy->dev;
	struct {
		u8 format_id;
		u8 sku_enable;
		u8 band_idx;
		u8 rsv;
	} req = {
		.format_id = 0,
		.band_idx = phy != &dev->phy,
		.sku_enable = enable,
	};

	return mt76_mcu_send_msg(&dev->mt76,
				 MCU_EXT_CMD(TX_POWER_FEATURE_CTRL),
				 &req, sizeof(req), true);
}

static int mt7615_find_freq_idx(const u16 *freqs, int n_freqs, u16 cur)
{
	int i;

	for (i = 0; i < n_freqs; i++)
		if (cur == freqs[i])
			return i;

	return -1;
}

static int mt7615_dcoc_freq_idx(u16 freq, u8 bw)
{
	static const u16 freq_list[] = {
		4980, 5805, 5905, 5190,
		5230, 5270, 5310, 5350,
		5390, 5430, 5470, 5510,
		5550, 5590, 5630, 5670,
		5710, 5755, 5795, 5835,
		5875, 5210, 5290, 5370,
		5450, 5530, 5610, 5690,
		5775, 5855
	};
	static const u16 freq_bw40[] = {
		5190, 5230, 5270, 5310,
		5350, 5390, 5430, 5470,
		5510, 5550, 5590, 5630,
		5670, 5710, 5755, 5795,
		5835, 5875
	};
	int offset_2g = ARRAY_SIZE(freq_list);
	int idx;

	if (freq < 4000) {
		if (freq < 2427)
			return offset_2g;
		if (freq < 2442)
			return offset_2g + 1;
		if (freq < 2457)
			return offset_2g + 2;

		return offset_2g + 3;
	}

	switch (bw) {
	case NL80211_CHAN_WIDTH_80:
	case NL80211_CHAN_WIDTH_80P80:
	case NL80211_CHAN_WIDTH_160:
		break;
	default:
		idx = mt7615_find_freq_idx(freq_bw40, ARRAY_SIZE(freq_bw40),
					   freq + 10);
		if (idx >= 0) {
			freq = freq_bw40[idx];
			break;
		}

		idx = mt7615_find_freq_idx(freq_bw40, ARRAY_SIZE(freq_bw40),
					   freq - 10);
		if (idx >= 0) {
			freq = freq_bw40[idx];
			break;
		}
		fallthrough;
	case NL80211_CHAN_WIDTH_40:
		idx = mt7615_find_freq_idx(freq_bw40, ARRAY_SIZE(freq_bw40),
					   freq);
		if (idx >= 0)
			break;

		return -1;

	}

	return mt7615_find_freq_idx(freq_list, ARRAY_SIZE(freq_list), freq);
}

int mt7615_mcu_apply_rx_dcoc(struct mt7615_phy *phy)
{
	struct mt7615_dev *dev = phy->dev;
	struct cfg80211_chan_def *chandef = &phy->mt76->chandef;
	int freq2 = chandef->center_freq2;
	int ret;
	struct {
		u8 direction;
		u8 runtime_calibration;
		u8 _rsv[2];

		__le16 center_freq;
		u8 bw;
		u8 band;
		u8 is_freq2;
		u8 success;
		u8 dbdc_en;

		u8 _rsv2;

		struct {
			__le32 sx0_i_lna[4];
			__le32 sx0_q_lna[4];

			__le32 sx2_i_lna[4];
			__le32 sx2_q_lna[4];
		} dcoc_data[4];
	} req = {
		.direction = 1,

		.bw = mt7615_mcu_chan_bw(chandef),
		.band = chandef->center_freq1 > 4000,
		.dbdc_en = !!dev->mt76.phys[MT_BAND1],
	};
	u16 center_freq = chandef->center_freq1;
	int freq_idx;
	u8 *eep = dev->mt76.eeprom.data;

	if (!(eep[MT_EE_CALDATA_FLASH] & MT_EE_CALDATA_FLASH_RX_CAL))
		return 0;

	if (chandef->width == NL80211_CHAN_WIDTH_160) {
		freq2 = center_freq + 40;
		center_freq -= 40;
	}

again:
	req.runtime_calibration = 1;
	freq_idx = mt7615_dcoc_freq_idx(center_freq, chandef->width);
	if (freq_idx < 0)
		goto out;

	memcpy(req.dcoc_data, eep + MT7615_EEPROM_DCOC_OFFSET +
			      freq_idx * MT7615_EEPROM_DCOC_SIZE,
	       sizeof(req.dcoc_data));
	req.runtime_calibration = 0;

out:
	req.center_freq = cpu_to_le16(center_freq);
	ret = mt76_mcu_send_msg(&dev->mt76, MCU_EXT_CMD(RXDCOC_CAL), &req,
				sizeof(req), true);

	if ((chandef->width == NL80211_CHAN_WIDTH_80P80 ||
	     chandef->width == NL80211_CHAN_WIDTH_160) && !req.is_freq2) {
		req.is_freq2 = true;
		center_freq = freq2;
		goto again;
	}

	return ret;
}

static int mt7615_dpd_freq_idx(u16 freq, u8 bw)
{
	static const u16 freq_list[] = {
		4920, 4940, 4960, 4980,
		5040, 5060, 5080, 5180,
		5200, 5220, 5240, 5260,
		5280, 5300, 5320, 5340,
		5360, 5380, 5400, 5420,
		5440, 5460, 5480, 5500,
		5520, 5540, 5560, 5580,
		5600, 5620, 5640, 5660,
		5680, 5700, 5720, 5745,
		5765, 5785, 5805, 5825,
		5845, 5865, 5885, 5905
	};
	int offset_2g = ARRAY_SIZE(freq_list);
	int idx;

	if (freq < 4000) {
		if (freq < 2432)
			return offset_2g;
		if (freq < 2457)
			return offset_2g + 1;

		return offset_2g + 2;
	}

	if (bw != NL80211_CHAN_WIDTH_20) {
		idx = mt7615_find_freq_idx(freq_list, ARRAY_SIZE(freq_list),
					   freq + 10);
		if (idx >= 0)
			return idx;

		idx = mt7615_find_freq_idx(freq_list, ARRAY_SIZE(freq_list),
					   freq - 10);
		if (idx >= 0)
			return idx;
	}

	return mt7615_find_freq_idx(freq_list, ARRAY_SIZE(freq_list), freq);
}


int mt7615_mcu_apply_tx_dpd(struct mt7615_phy *phy)
{
	struct mt7615_dev *dev = phy->dev;
	struct cfg80211_chan_def *chandef = &phy->mt76->chandef;
	int freq2 = chandef->center_freq2;
	int ret;
	struct {
		u8 direction;
		u8 runtime_calibration;
		u8 _rsv[2];

		__le16 center_freq;
		u8 bw;
		u8 band;
		u8 is_freq2;
		u8 success;
		u8 dbdc_en;

		u8 _rsv2;

		struct {
			struct {
				u32 dpd_g0;
				u8 data[32];
			} wf0, wf1;

			struct {
				u32 dpd_g0_prim;
				u32 dpd_g0_sec;
				u8 data_prim[32];
				u8 data_sec[32];
			} wf2, wf3;
		} dpd_data;
	} req = {
		.direction = 1,

		.bw = mt7615_mcu_chan_bw(chandef),
		.band = chandef->center_freq1 > 4000,
		.dbdc_en = !!dev->mt76.phys[MT_BAND1],
	};
	u16 center_freq = chandef->center_freq1;
	int freq_idx;
	u8 *eep = dev->mt76.eeprom.data;

	if (!(eep[MT_EE_CALDATA_FLASH] & MT_EE_CALDATA_FLASH_TX_DPD))
		return 0;

	if (chandef->width == NL80211_CHAN_WIDTH_160) {
		freq2 = center_freq + 40;
		center_freq -= 40;
	}

again:
	req.runtime_calibration = 1;
	freq_idx = mt7615_dpd_freq_idx(center_freq, chandef->width);
	if (freq_idx < 0)
		goto out;

	memcpy(&req.dpd_data, eep + MT7615_EEPROM_TXDPD_OFFSET +
			      freq_idx * MT7615_EEPROM_TXDPD_SIZE,
	       sizeof(req.dpd_data));
	req.runtime_calibration = 0;

out:
	req.center_freq = cpu_to_le16(center_freq);
	ret = mt76_mcu_send_msg(&dev->mt76, MCU_EXT_CMD(TXDPD_CAL),
				&req, sizeof(req), true);

	if ((chandef->width == NL80211_CHAN_WIDTH_80P80 ||
	     chandef->width == NL80211_CHAN_WIDTH_160) && !req.is_freq2) {
		req.is_freq2 = true;
		center_freq = freq2;
		goto again;
	}

	return ret;
}

int mt7615_mcu_set_rx_hdr_trans_blacklist(struct mt7615_dev *dev)
{
	struct {
		u8 operation;
		u8 count;
		u8 _rsv[2];
		u8 index;
		u8 enable;
		__le16 etype;
	} req = {
		.operation = 1,
		.count = 1,
		.enable = 1,
		.etype = cpu_to_le16(ETH_P_PAE),
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_EXT_CMD(RX_HDR_TRANS),
				 &req, sizeof(req), false);
}

int mt7615_mcu_set_bss_pm(struct mt7615_dev *dev, struct ieee80211_vif *vif,
			  bool enable)
{
	struct mt7615_vif *mvif = (struct mt7615_vif *)vif->drv_priv;
	struct {
		u8 bss_idx;
		u8 dtim_period;
		__le16 aid;
		__le16 bcn_interval;
		__le16 atim_window;
		u8 uapsd;
		u8 bmc_delivered_ac;
		u8 bmc_triggered_ac;
		u8 pad;
	} req = {
		.bss_idx = mvif->mt76.idx,
		.aid = cpu_to_le16(vif->cfg.aid),
		.dtim_period = vif->bss_conf.dtim_period,
		.bcn_interval = cpu_to_le16(vif->bss_conf.beacon_int),
	};
	struct {
		u8 bss_idx;
		u8 pad[3];
	} req_hdr = {
		.bss_idx = mvif->mt76.idx,
	};
	int err;

	if (vif->type != NL80211_IFTYPE_STATION)
		return 0;

	err = mt76_mcu_send_msg(&dev->mt76, MCU_CE_CMD(SET_BSS_ABORT),
				&req_hdr, sizeof(req_hdr), false);
	if (err < 0 || !enable)
		return err;

	return mt76_mcu_send_msg(&dev->mt76, MCU_CE_CMD(SET_BSS_CONNECTED),
				 &req, sizeof(req), false);
}

int mt7615_mcu_set_roc(struct mt7615_phy *phy, struct ieee80211_vif *vif,
		       struct ieee80211_channel *chan, int duration)
{
	struct mt7615_vif *mvif = (struct mt7615_vif *)vif->drv_priv;
	struct mt7615_dev *dev = phy->dev;
	struct mt7615_roc_tlv req = {
		.bss_idx = mvif->mt76.idx,
		.active = !chan,
		.max_interval = cpu_to_le32(duration),
		.primary_chan = chan ? chan->hw_value : 0,
		.band = chan ? chan->band : 0,
		.req_type = 2,
	};

	phy->roc_grant = false;

	return mt76_mcu_send_msg(&dev->mt76, MCU_CE_CMD(SET_ROC),
				 &req, sizeof(req), false);
}

int mt7615_mcu_set_protection(struct mt7615_phy *phy, struct ieee80211_vif *vif,
			      u8 ht_mode, bool use_cts_prot)
{
	struct mt7615_dev *dev = phy->dev;
	struct {
		u8 prot_idx;
		u8 band;
		u8 rsv[2];

		bool long_nav;
		bool prot_mm;
		bool prot_gf;
		bool prot_bw40;
		bool prot_rifs;
		bool prot_bw80;
		bool prot_bw160;
		u8 prot_erp_mask;
	} __packed req = {
		.prot_idx = 0x2,
		.band = phy != &dev->phy,
	};

	switch (ht_mode & IEEE80211_HT_OP_MODE_PROTECTION) {
	case IEEE80211_HT_OP_MODE_PROTECTION_NONMEMBER:
	case IEEE80211_HT_OP_MODE_PROTECTION_NONHT_MIXED:
		req.prot_mm = true;
		req.prot_gf = true;
		fallthrough;
	case IEEE80211_HT_OP_MODE_PROTECTION_20MHZ:
		req.prot_bw40 = true;
		break;
	}

	if (ht_mode & IEEE80211_HT_OP_MODE_NON_GF_STA_PRSNT)
		req.prot_gf = true;

	if (use_cts_prot) {
		struct mt7615_vif *mvif = (struct mt7615_vif *)vif->drv_priv;
		u8 i = mvif->mt76.omac_idx > HW_BSSID_MAX ? HW_BSSID_0 : mvif->mt76.omac_idx;

		req.prot_erp_mask = BIT(i);
	}

	return mt76_mcu_send_msg(&dev->mt76, MCU_EXT_CMD(PROTECT_CTRL), &req,
				 sizeof(req), true);
}
