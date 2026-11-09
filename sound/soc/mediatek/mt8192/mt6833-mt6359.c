// SPDX-License-Identifier: GPL-2.0
/*
 * mt6833-mt6359.c  --  MT6833 + MT6359 ALSA SoC machine driver
 *
 * MT6833 has the MT8192 audio front end, so this card is built on the
 * MT8192 AFE driver. Everything goes through the MT6359 codec (ADDA):
 * headset, earpiece, line out to an analog speaker amplifier described
 * as an auxiliary device, and the analog microphones.
 *
 * The MTKAIF calibration follows the MT6833 vendor machine driver: it
 * only calibrates the MISO lines in use. The card has no link for the
 * third ADC, whose data line (MISO2) boards leave unconnected and which
 * then never completes the test.
 */

#include <linux/hex.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <sound/jack.h>
#include <sound/soc.h>

#include "../../codecs/mt6359.h"
#include "../../codecs/mt6359-accdet.h"
#include "../common/mtk-afe-platform-driver.h"
#include "../common/mtk-soc-card.h"
#include "../common/mtk-soundcard-driver.h"
#include "mt8192-afe-clk.h"
#include "mt8192-afe-common.h"
#include "mt8192-afe-gpio.h"

#define MT6833_MTKAIF_MISO_NUM	3

enum {
	MT6833_JACK_HEADSET,
	MT6833_JACK_MAX,
};

static struct snd_soc_jack_pin mt6833_mt6359_jack_pins[] = {
	{
		.pin = "Headphone",
		.mask = SND_JACK_HEADPHONE,
	},
	{
		.pin = "Headset Mic",
		.mask = SND_JACK_MICROPHONE,
	},
};

static void mt6833_mt6359_mtkaif_calibration(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *cmpnt_afe =
		snd_soc_rtdcom_lookup(rtd, AFE_PCM_NAME);
	struct snd_soc_component *cmpnt_codec =
		snd_soc_rtd_to_codec(rtd, 0)->component;
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt_afe);
	struct mt8192_afe_private *afe_priv = afe->platform_priv;
	int test_done[MT6833_MTKAIF_MISO_NUM];
	int cycle[MT6833_MTKAIF_MISO_NUM];
	int prev_cycle[MT6833_MTKAIF_MISO_NUM];
	int phase[MT6833_MTKAIF_MISO_NUM];
	const unsigned int used = BIT(0) | BIT(1);
	bool calib_ok = true;
	unsigned int monitor = 0;
	/* the delay cycle each phase gave on MISO0 and MISO1, one hex digit each */
	char trace[2][44] = {};
	int counter, i, p;

	pm_runtime_get_sync(afe->dev);
	mt8192_afe_gpio_request(afe->dev, true, MT8192_DAI_ADDA, 1);
	mt8192_afe_gpio_request(afe->dev, true, MT8192_DAI_ADDA, 0);

	mt6359_mtkaif_calibration_enable(cmpnt_codec);

	/* set clock protocol 2 */
	regmap_update_bits(afe->regmap, AFE_AUD_PAD_TOP, 0xff, 0x38);
	regmap_update_bits(afe->regmap, AFE_AUD_PAD_TOP, 0xff, 0x39);

	/* set test type to synchronizer pulse */
	regmap_update_bits(afe_priv->topckgen, CKSYS_AUD_TOP_CFG, 0xffff, 0x4);

	afe_priv->mtkaif_calibration_num_phase = 42;	/* mt6359: 0 ~ 42 */
	for (i = 0; i < MT6833_MTKAIF_MISO_NUM; i++) {
		afe_priv->mtkaif_chosen_phase[i] = -1;
		prev_cycle[i] = -1;
	}

	for (p = 0; p <= afe_priv->mtkaif_calibration_num_phase && calib_ok; p++) {
		mt6359_set_mtkaif_calibration_phase(cmpnt_codec, p, p, p);

		regmap_update_bits(afe_priv->topckgen, CKSYS_AUD_TOP_CFG, 0x1, 0x1);

		for (i = 0; i < MT6833_MTKAIF_MISO_NUM; i++) {
			test_done[i] = (used & BIT(i)) ? 0 : -1;
			cycle[i] = -1;
		}

		counter = 0;
		while (test_done[0] == 0 || test_done[1] == 0 || test_done[2] == 0) {
			regmap_read(afe_priv->topckgen, CKSYS_AUD_TOP_MON, &monitor);

			for (i = 0; i < MT6833_MTKAIF_MISO_NUM; i++) {
				if (test_done[i] == 0)
					test_done[i] = (monitor >> (28 + i)) & 0x1;
				if (test_done[i] == 1)
					cycle[i] = (monitor >> (4 * i)) & 0xf;
			}

			if (++counter > 10000) {
				dev_err(afe->dev,
					"MTKAIF calibration timed out, cycles %d/%d/%d, monitor 0x%x\n",
					cycle[0], cycle[1], cycle[2], monitor);
				calib_ok = false;
				break;
			}
		}

		for (i = 0; i < MT6833_MTKAIF_MISO_NUM; i++) {
			if (p == 0)
				prev_cycle[i] = cycle[i];
			if (i < 2)
				trace[i][p] = cycle[i] < 0 ? '-' : hex_asc[cycle[i]];

			if ((used & BIT(i)) && cycle[i] != prev_cycle[i] &&
			    afe_priv->mtkaif_chosen_phase[i] < 0) {
				afe_priv->mtkaif_chosen_phase[i] = p - 1;
				afe_priv->mtkaif_phase_cycle[i] = prev_cycle[i];
			}
		}

		regmap_update_bits(afe_priv->topckgen, CKSYS_AUD_TOP_CFG, 0x1, 0x0);
	}

	for (i = 0; i < MT6833_MTKAIF_MISO_NUM; i++)
		phase[i] = max(afe_priv->mtkaif_chosen_phase[i], 0);

	mt6359_set_mtkaif_calibration_phase(cmpnt_codec, phase[0], phase[1], phase[2]);

	/* disable rx fifo */
	regmap_update_bits(afe->regmap, AFE_AUD_PAD_TOP, 0xff, 0x38);

	mt6359_mtkaif_calibration_disable(cmpnt_codec);

	mt8192_afe_gpio_request(afe->dev, false, MT8192_DAI_ADDA, 1);
	mt8192_afe_gpio_request(afe->dev, false, MT8192_DAI_ADDA, 0);
	pm_runtime_put(afe->dev);

	dev_info(afe->dev, "MTKAIF cycles per phase: MISO0 %s, MISO1 %s\n",
		 trace[0], trace[1]);
	dev_info(afe->dev, "MTKAIF calibration %s, phases %d/%d/%d\n",
		 calib_ok ? "done" : "failed",
		 afe_priv->mtkaif_chosen_phase[0],
		 afe_priv->mtkaif_chosen_phase[1],
		 afe_priv->mtkaif_chosen_phase[2]);
}

static int mt6833_mt6359_accdet_init(struct snd_soc_pcm_runtime *rtd)
{
	struct mtk_soc_card_data *soc_card_data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_jack *jack = &soc_card_data->card_data->jacks[MT6833_JACK_HEADSET];
	int ret;

	if (!soc_card_data->accdet)
		return 0;

	ret = snd_soc_card_jack_new_pins(rtd->card, "Headset Jack",
					 SND_JACK_HEADSET | SND_JACK_BTN_0 |
					 SND_JACK_BTN_1 | SND_JACK_BTN_2 |
					 SND_JACK_BTN_3,
					 jack, mt6833_mt6359_jack_pins,
					 ARRAY_SIZE(mt6833_mt6359_jack_pins));
	if (ret)
		return dev_err_probe(rtd->dev, ret, "Headset Jack create failed\n");

	/*
	 * In three-key mode the accdet driver reports the middle key as BTN_0,
	 * the key between the up and down thresholds (volume down) as BTN_1
	 * and the one between the middle and up thresholds (volume up) as
	 * BTN_2, as the vendor driver decides them; BTN_3 is the fourth key
	 * of four-key headsets.
	 */
	snd_jack_set_key(jack->jack, SND_JACK_BTN_0, KEY_PLAYPAUSE);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_1, KEY_VOLUMEDOWN);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_2, KEY_VOLUMEUP);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_3, KEY_VOICECOMMAND);

	ret = mt6359_accdet_enable_jack_detect(soc_card_data->accdet, jack);
	if (ret)
		return dev_err_probe(rtd->dev, ret, "Headset Jack enable failed\n");

	return 0;
}

static int mt6833_mt6359_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *cmpnt_afe =
		snd_soc_rtdcom_lookup(rtd, AFE_PCM_NAME);
	struct snd_soc_component *cmpnt_codec =
		snd_soc_rtd_to_codec(rtd, 0)->component;
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt_afe);
	struct mt8192_afe_private *afe_priv = afe->platform_priv;

	mt6359_set_mtkaif_protocol(cmpnt_codec, MT6359_MTKAIF_PROTOCOL_2_CLK_P2);
	afe_priv->mtkaif_protocol = MTKAIF_PROTOCOL_2_CLK_P2;

	mt6833_mt6359_mtkaif_calibration(rtd);

	return mt6833_mt6359_accdet_init(rtd);
}

/* FE */
SND_SOC_DAILINK_DEFS(playback1,
		     DAILINK_COMP_ARRAY(COMP_CPU("DL1")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(playback2,
		     DAILINK_COMP_ARRAY(COMP_CPU("DL2")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(playback3,
		     DAILINK_COMP_ARRAY(COMP_CPU("DL3")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(capture1,
		     DAILINK_COMP_ARRAY(COMP_CPU("UL1")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(capture2,
		     DAILINK_COMP_ARRAY(COMP_CPU("UL2")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

/* BE */
SND_SOC_DAILINK_DEFS(primary_codec,
		     DAILINK_COMP_ARRAY(COMP_CPU("ADDA")),
		     DAILINK_COMP_ARRAY(COMP_CODEC("mt6359-sound",
						   "mt6359-snd-codec-aif1")),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

#define MT6833_FE_LINK(_name, _defs, _playback)				\
	{								\
		.name = _name,						\
		.stream_name = _name,					\
		.trigger = { SND_SOC_DPCM_TRIGGER_PRE,			\
			     SND_SOC_DPCM_TRIGGER_PRE },		\
		.dynamic = 1,						\
		.playback_only = _playback,				\
		.capture_only = !(_playback),				\
		SND_SOC_DAILINK_REG(_defs),				\
	}

static struct snd_soc_dai_link mt6833_mt6359_dai_links[] = {
	MT6833_FE_LINK("Playback_1", playback1, 1),
	MT6833_FE_LINK("Playback_2", playback2, 1),
	MT6833_FE_LINK("Playback_3", playback3, 1),
	MT6833_FE_LINK("Capture_1", capture1, 0),
	MT6833_FE_LINK("Capture_2", capture2, 0),
	{
		.name = "Primary Codec",
		.no_pcm = 1,
		.ignore_suspend = 1,
		.init = mt6833_mt6359_init,
		SND_SOC_DAILINK_REG(primary_codec),
	},
};

static const struct snd_soc_dapm_widget mt6833_mt6359_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_SPK("Earpiece", NULL),
};

static const struct snd_kcontrol_new mt6833_mt6359_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Speaker"),
	SOC_DAPM_PIN_SWITCH("Earpiece"),
};

static struct snd_soc_card mt6833_mt6359_card = {
	.driver_name = "mt6833-mt6359",
	.owner = THIS_MODULE,
	.dai_link = mt6833_mt6359_dai_links,
	.num_links = ARRAY_SIZE(mt6833_mt6359_dai_links),
	.dapm_widgets = mt6833_mt6359_widgets,
	.num_dapm_widgets = ARRAY_SIZE(mt6833_mt6359_widgets),
	.controls = mt6833_mt6359_controls,
	.num_controls = ARRAY_SIZE(mt6833_mt6359_controls),
};

static int mt6833_mt6359_soc_card_probe(struct mtk_soc_card_data *soc_card_data,
					bool legacy)
{
	struct snd_soc_card *card = soc_card_data->card_data->card;
	int ret;

	/* the routing between the card widgets and the codec is in DT */
	if (legacy)
		return dev_err_probe(card->dev, -EINVAL, "audio-routing missing\n");

	ret = snd_soc_of_parse_aux_devs(card, "aux-devs");
	if (ret)
		return ret;

	ret = mt8192_afe_gpio_init(card->dev);
	if (ret)
		return dev_err_probe(card->dev, ret, "init gpio error\n");

	return 0;
}

static const struct mtk_soundcard_pdata mt6833_mt6359_pdata = {
	.card_name = "mt6833-mt6359",
	.card_data = &(struct mtk_platform_card_data) {
		.card = &mt6833_mt6359_card,
		.num_jacks = MT6833_JACK_MAX,
	},
	.soc_probe = mt6833_mt6359_soc_card_probe,
};

static const struct of_device_id mt6833_mt6359_dt_match[] = {
	{ .compatible = "mediatek,mt6833-mt6359-sound", .data = &mt6833_mt6359_pdata },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt6833_mt6359_dt_match);

static struct platform_driver mt6833_mt6359_driver = {
	.driver = {
		.name = "mt6833-mt6359",
		.of_match_table = mt6833_mt6359_dt_match,
		.pm = &snd_soc_pm_ops,
	},
	.probe = mtk_soundcard_common_probe,
};
module_platform_driver(mt6833_mt6359_driver);

MODULE_DESCRIPTION("MT6833 MT6359 ALSA SoC machine driver");
MODULE_LICENSE("GPL");
