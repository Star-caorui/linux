// SPDX-License-Identifier: GPL-2.0-only
/*
 * Chipone ICNA35XX Driver IC panels driver
 *
 * Copyright (c) 2025 Teguh Sobirin <teguh@sobir.in>
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct panel_info {
	struct drm_panel panel;
	struct drm_connector *connector;
	struct mipi_dsi_device *dsi;
	struct panel_desc *desc;
	enum drm_panel_orientation orientation;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data *supplies;
	bool supplies_enabled;

	/* the level last sent to the panel, for panels that cannot be read */
	int sent_brightness;
};

/* One step of a reset waveform: drive the line, then wait */
struct panel_reset_step {
	u8 asserted;
	u16 delay_us;
};

/* The command that selects a refresh rate, for panels told it at init */
struct panel_fps_cmd {
	unsigned int vrefresh;
	u8 cmd[2];
};

struct panel_desc {
	unsigned int width_mm;
	unsigned int height_mm;

	unsigned int bpc;
	unsigned int lanes;
	unsigned long mode_flags;
	enum mipi_dsi_pixel_format format;

	const struct drm_display_mode *modes;
	unsigned int num_modes;
	int (*init_sequence)(struct panel_info *pinfo);
	/* the reset waveform, if not icna35xx_reset()'s */
	const struct panel_reset_step *reset_seq;
	unsigned int num_reset_steps;
	/* refresh rate commands, the first one the default */
	const struct panel_fps_cmd *fps_cmds;
	unsigned int num_fps_cmds;
	/* between display off and sleep in, if not 50 ms */
	unsigned int display_off_ms;

	struct drm_dsc_config dsc;
	/* lane rate the panel expects, for a DSI host that cannot derive it */
	unsigned long hs_rate;

	const struct backlight_ops *bl_ops;
	unsigned int max_brightness;
	unsigned int brightness;

	/*
	 * Keep the panel's supplies on from probe when the bootloader left
	 * them on, so that its picture stays until the first modeset.
	 */
	bool boot_on;
};

static const struct regulator_bulk_data panel_supplies[] = {
	{ .supply = "vdd" },
	{ .supply = "vddio" },
	{ .supply = "vci" },
	{ .supply = "disp" },
	{ .supply = "blvdd" },
};

static inline struct panel_info *to_panel_info(struct drm_panel *panel)
{
	return container_of(panel, struct panel_info, panel);
}

static int icna3512_init_sequence(struct panel_info *pinfo)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = pinfo->dsi };
	struct drm_dsc_picture_parameter_set pps;

	pinfo->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9C, 0xA5, 0xA5);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xFD, 0x5A, 0x5A);

	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x53, 0xE0);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x35, 0x00);

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);

	mipi_dsi_msleep(&dsi_ctx, 120);

	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9F, 0x0F);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xCE, 0x22);

	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9F, 0x01);

	/* 165 hz */
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x48, 0x20);

	drm_dsc_pps_payload_pack(&pps, &pinfo->desc->dsc);
	mipi_dsi_picture_parameter_set_multi(&dsi_ctx, &pps);

	mipi_dsi_msleep(&dsi_ctx, 20);

	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

	return dsi_ctx.accum_err;
}

static int icna3520_init_sequence(struct panel_info *pinfo)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = pinfo->dsi };
	struct drm_dsc_picture_parameter_set pps;

	pinfo->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9C, 0xA5, 0xA5);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xFD, 0x5A, 0x5A);

	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x53, 0xE0);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x35, 0x00);

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);

	mipi_dsi_msleep(&dsi_ctx, 120);

	/* 120 hz */
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x48, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9F, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xB3,
		0x00, 0xD8, 0x00, 0x1C, 0x00, 0x4C);

	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9F, 0x01);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xB2, 0x00);

	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9F, 0x0D);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xB2, 0x27);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xB6, 0x03);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xBB, 0x01);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xB2, 0x24);

	drm_dsc_pps_payload_pack(&pps, &pinfo->desc->dsc);
	mipi_dsi_picture_parameter_set_multi(&dsi_ctx, &pps);

	mipi_dsi_msleep(&dsi_ctx, 20);

	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

	return dsi_ctx.accum_err;
}

/*
 * The connector state's mode, for panels that are told their refresh rate
 * in their init sequence
 */
static int icna35xx_vrefresh(struct panel_info *pinfo)
{
	struct drm_connector *connector = pinfo->connector;
	struct drm_crtc_state *crtc_state;

	if (!connector || !connector->state || !connector->state->crtc)
		return 0;
	crtc_state = connector->state->crtc->state;

	return crtc_state ? drm_mode_vrefresh(&crtc_state->mode) : 0;
}

static const struct panel_fps_cmd *icna35xx_fps_cmd(struct panel_info *pinfo)
{
	const struct panel_desc *desc = pinfo->desc;
	int vrefresh = icna35xx_vrefresh(pinfo);
	unsigned int i;

	for (i = 0; i < desc->num_fps_cmds; i++)
		if (desc->fps_cmds[i].vrefresh == vrefresh)
			return &desc->fps_cmds[i];

	return &desc->fps_cmds[0];
}

/*
 * Redmi Note 13 5G (gold) panel n17-36-0f-0c, as Xiaomi's driver
 * panel-n17-36-0f-0c-dsc-vdo.c sends it (panel_init(), :287-380): DCS
 * packets for commands below 0xb0, generic ones above.
 */

static int n17_36_0f_0c_init_sequence(struct panel_info *pinfo)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = pinfo->dsi };
	const struct panel_fps_cmd *rate = icna35xx_fps_cmd(pinfo);

	/* disable() leaves high speed set */
	pinfo->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9c, 0xa5, 0xa5);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xfd, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9f, 0x0f);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xd7, 0x11);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9f, 0x01);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xe4,
		0x00, 0x00, 0x80, 0x0c, 0xff, 0x0f, 0x08, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x90, 0x00);
	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, rate->cmd, sizeof(rate->cmd));

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0xe0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_TEAR_ON);
	/* Xiaomi's driver sends the command after 0x35 twice */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9f, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9f, 0x08);
	/* GIR and LIR off, Xiaomi's default */
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb2, 0x10);
	/* hsync */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9f, 0x01);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb4, 0x10, 0x00, 0x00, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9f, 0x02);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc4,
		0x00, 0x0c, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc6,
		0x00, 0x00, 0x0f, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00,
		0x90, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x02, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x55, 0x01, 0x01, 0x00);
	/* oscillator */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9f, 0x01);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xca,
		0x07, 0x02, 0x00, 0x03, 0x03, 0x01, 0xb8, 0x04, 0x04, 0x01,
		0xb8, 0x04, 0x04, 0x01, 0xb8, 0x04, 0x04, 0x88, 0xaa);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xcb,
		0x04, 0x04, 0x04, 0x01, 0xb8, 0x04, 0x04, 0x01, 0xb8, 0x04,
		0x04, 0x01, 0xb8, 0x04, 0x04);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xd3, 0x80, 0x00);
	/* LVD */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9f, 0x02);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xed, 0x00, 0x01, 0x80);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xee,
		0x04, 0x00, 0x00, 0x00, 0x08, 0x88, 0x08, 0x08, 0x00,
		0xff, 0x88, 0x88, 0x29, 0x00, 0x00, 0x00, 0x00);

	/*
	 * Display on at brightness 0, where the panel comes out of reset: in
	 * front of the MT6382 the video only starts after this, and the panel
	 * lit up shows what its memory holds until then (a white flash). The
	 * level follows when the panel is enabled, the video running by then,
	 * as in stock's screen on (29 at the end of the init, the TX started,
	 * then the backlight a frame later; LK sends 51 before 29 instead).
	 */
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

	return dsi_ctx.accum_err;
}

static const struct drm_display_mode odin2portal_modes[] = {
	{
		/* 165Hz */
		.clock = (1080 + 98 + 1 + 23) * (1920 + 20 + 1 + 15) * 165 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 98,
		.hsync_end = 1080 + 98 + 1,
		.htotal = 1080 + 98 + 1 + 23,
		.vdisplay = 1920,
		.vsync_start = 1920 + 20,
		.vsync_end = 1920 + 20 + 1,
		.vtotal = 1920 + 20 + 1 + 15,
	}
};

static const struct drm_display_mode thor_top_modes[] = {
	{
		/* 120Hz */
		.clock = (1080 + 24 + 1 + 24) * (1920 + 28 + 1 + 28) * 120 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 24,
		.hsync_end = 1080 + 24 + 1,
		.htotal = 1080 + 24 + 1 + 24,
		.vdisplay = 1920,
		.vsync_start = 1920 + 28,
		.vsync_end = 1920 + 28 + 1,
		.vtotal = 1920 + 28 + 1 + 28,
	}
};

/* Modes 60, 90 and 120 Hz, which differ only in VFP (:46-73) */
#define N17_36_0F_0C_MODE(vfp, fps) {						\
	.clock = (1080 + 195 + 4 + 17) * (2400 + (vfp) + 1 + 19) * (fps) / 1000,	\
	.hdisplay = 1080,							\
	.hsync_start = 1080 + 195,						\
	.hsync_end = 1080 + 195 + 4,						\
	.htotal = 1080 + 195 + 4 + 17,						\
	.vdisplay = 2400,							\
	.vsync_start = 2400 + (vfp),						\
	.vsync_end = 2400 + (vfp) + 1,						\
	.vtotal = 2400 + (vfp) + 1 + 19,					\
}

static const struct drm_display_mode n17_36_0f_0c_modes[] = {
	N17_36_0F_0C_MODE(2380, 60),
	N17_36_0F_0C_MODE(812, 90),
	N17_36_0F_0C_MODE(12, 120),
};

static struct panel_desc odin2portal_desc = {
	.modes = odin2portal_modes,
	.num_modes = ARRAY_SIZE(odin2portal_modes),
	.width_mm = 160,
	.height_mm = 89,
	.bpc = 8,
	.lanes = 4,
	.format = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_NO_EOT_PACKET | MIPI_DSI_CLOCK_NON_CONTINUOUS |
			MIPI_DSI_MODE_LPM,
	.init_sequence = icna3512_init_sequence,
	.dsc = {
		.dsc_version_major = 0x1,
		.dsc_version_minor = 0x1,
		.slice_height = 20,
		.slice_width = 540,
		.slice_count = 2,
		.bits_per_component = 8,
		.bits_per_pixel = 8 << 4,
		.block_pred_enable = true,
	},
};

static struct panel_desc thor_top_desc = {
	.modes = thor_top_modes,
	.num_modes = ARRAY_SIZE(thor_top_modes),
	.width_mm = 136,
	.height_mm = 68,
	.bpc = 8,
	.lanes = 4,
	.format = MIPI_DSI_FMT_RGB888,
	.mode_flags =  MIPI_DSI_MODE_NO_EOT_PACKET | MIPI_DSI_CLOCK_NON_CONTINUOUS |
			MIPI_DSI_MODE_LPM,
	.init_sequence = icna3520_init_sequence,
	.dsc = {
		.dsc_version_major = 0x1,
		.dsc_version_minor = 0x1,
		.slice_height = 12,
		.slice_width = 540,
		.slice_count = 2,
		.bits_per_component = 8,
		.bits_per_pixel = 8 << 4,
		.block_pred_enable = true,
	},
};

/* panel_init(), :291-298 */
static const struct panel_reset_step n17_36_0f_0c_reset[] = {
	{ 1, 100 }, { 0, 10000 }, { 1, 5000 }, { 0, 41000 },
};

/* frame rate: dfps_cmd_table, :565-567 */
static const struct panel_fps_cmd n17_36_0f_0c_fps[] = {
	{ 60, { 0x48, 0x23 } },
	{ 90, { 0x48, 0x13 } },
	{ 120, { 0x48, 0x03 } },
};

static void icna35xx_run_reset(struct panel_info *pinfo)
{
	const struct panel_desc *desc = pinfo->desc;
	unsigned int i, us;

	for (i = 0; i < desc->num_reset_steps; i++) {
		gpiod_set_value_cansleep(pinfo->reset_gpio, desc->reset_seq[i].asserted);
		us = desc->reset_seq[i].delay_us;
		usleep_range(us, us + us / 10 + 10);
	}
}

static void icna35xx_reset(struct panel_info *pinfo)
{
	gpiod_set_value_cansleep(pinfo->reset_gpio, 0);
	usleep_range(20000, 21000);
	gpiod_set_value_cansleep(pinfo->reset_gpio, 1);
	usleep_range(20000, 21000);
	gpiod_set_value_cansleep(pinfo->reset_gpio, 0);
	usleep_range(20000, 21000);
}

static int icna35xx_prepare(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int ret;

	if (!pinfo->supplies_enabled) {
		ret = regulator_bulk_enable(ARRAY_SIZE(panel_supplies), pinfo->supplies);
		if (ret < 0) {
			dev_err(panel->dev, "failed to enable regulators: %d\n", ret);
			return ret;
		}
		pinfo->supplies_enabled = true;
	}

	if (pinfo->desc->reset_seq)
		icna35xx_run_reset(pinfo);
	else
		icna35xx_reset(pinfo);

	ret = pinfo->desc->init_sequence(pinfo);
	if (ret < 0) {
		regulator_bulk_disable(ARRAY_SIZE(panel_supplies), pinfo->supplies);
		pinfo->supplies_enabled = false;
		dev_err(panel->dev, "failed to initialize panel: %d\n", ret);
		return ret;
	}

	return 0;
}

static int icna35xx_disable(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = pinfo->dsi };

	pinfo->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, pinfo->desc->display_off_ms ?: 50);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

static int icna35xx_unprepare(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);

	gpiod_set_value_cansleep(pinfo->reset_gpio, 1);
	if (pinfo->supplies_enabled) {
		regulator_bulk_disable(ARRAY_SIZE(panel_supplies), pinfo->supplies);
		pinfo->supplies_enabled = false;
	}
	pinfo->sent_brightness = -1;

	return 0;
}

static int icna35xx_get_modes(struct drm_panel *panel,
			       struct drm_connector *connector)
{
	struct panel_info *pinfo = to_panel_info(panel);
	const struct panel_desc *desc = pinfo->desc;
	struct drm_display_mode *mode;
	unsigned int i;

	pinfo->connector = connector;

	for (i = 0; i < desc->num_modes; i++) {
		mode = drm_mode_duplicate(connector->dev, &desc->modes[i]);
		if (!mode)
			return -ENOMEM;
		drm_mode_set_name(mode);
		mode->type = DRM_MODE_TYPE_DRIVER;
		if (i == 0)
			mode->type |= DRM_MODE_TYPE_PREFERRED;
		drm_mode_probed_add(connector, mode);
	}

	connector->display_info.width_mm = desc->width_mm;
	connector->display_info.height_mm = desc->height_mm;

	return desc->num_modes;
}

static enum drm_panel_orientation icna35xx_get_orientation(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);

	return pinfo->orientation;
}

static const struct drm_panel_funcs icna35xx_panel_funcs = {
	.disable = icna35xx_disable,
	.prepare = icna35xx_prepare,
	.unprepare = icna35xx_unprepare,
	.get_modes = icna35xx_get_modes,
	.get_orientation = icna35xx_get_orientation,
};

static int icna35xx_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return ret;
}

static int icna35xx_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness_large(dsi, &brightness);

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return ret < 0 ? ret : brightness;
}

static const struct backlight_ops icna35xx_bl_ops = {
	.update_status = icna35xx_bl_update_status,
	.get_brightness = icna35xx_bl_get_brightness,
};

/*
 * Brightness (DCS 0x51, 12 bits) and with it the VINT voltages the panel
 * should use at 60, 90 and 120 Hz, from Xiaomi's DBV to VINT table
 * (panel-n17-36-0f-0c-dsc-vdo.c:139-180, get_bl_tb2() and
 * panel_setbacklight_cmdq(), :700-760): per row the DBV range, 12 bits each,
 * and the VINT for 120, 90 and 60 Hz.
 */
static const u8 n17_36_0f_0c_dbv_vint[][6] = {
	{ 0x00, 0x01, 0x84, 0x10, 0x0e, 0x0b },
	{ 0x18, 0x51, 0xfe, 0x11, 0x0f, 0x0c },
	{ 0x1f, 0xf2, 0x78, 0x12, 0x10, 0x0d },
	{ 0x27, 0x92, 0xf2, 0x13, 0x11, 0x0e },
	{ 0x2f, 0x33, 0x99, 0x14, 0x12, 0x0f },
	{ 0x39, 0xa4, 0x66, 0x15, 0x13, 0x10 },
	{ 0x46, 0x75, 0x32, 0x16, 0x14, 0x11 },
	{ 0x53, 0x35, 0xff, 0x17, 0x15, 0x12 },
	{ 0x60, 0x06, 0xcc, 0x18, 0x16, 0x13 },
	{ 0x6c, 0xd7, 0x99, 0x19, 0x17, 0x14 },
	{ 0x79, 0xa8, 0x48, 0x1a, 0x18, 0x15 },
	{ 0x84, 0x98, 0xda, 0x1b, 0x19, 0x16 },
	{ 0x8d, 0xb9, 0x6c, 0x1c, 0x1a, 0x17 },
	{ 0x96, 0xd9, 0xfe, 0x1d, 0x1b, 0x18 },
	{ 0x9f, 0xfa, 0x91, 0x1e, 0x1c, 0x19 },
	{ 0xa9, 0x2b, 0x23, 0x1f, 0x1d, 0x1a },
	{ 0xb2, 0x4b, 0xb5, 0x20, 0x1e, 0x1b },
	{ 0xbb, 0x6c, 0x48, 0x21, 0x1f, 0x1c },
	{ 0xc4, 0x9c, 0xda, 0x22, 0x20, 0x1d },
	{ 0xcd, 0xbd, 0x6c, 0x23, 0x21, 0x1e },
	{ 0xd6, 0xdd, 0xfe, 0x24, 0x22, 0x1f },
	{ 0xdf, 0xfe, 0x91, 0x25, 0x23, 0x20 },
	{ 0xe9, 0x2f, 0x23, 0x26, 0x24, 0x21 },
	{ 0xf2, 0x4f, 0xb5, 0x27, 0x25, 0x22 },
	{ 0xfb, 0x6f, 0xff, 0x28, 0x26, 0x23 },
};

static int n17_36_0f_0c_set_brightness(struct panel_info *pinfo, u16 brightness)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = pinfo->dsi };
	u8 dbv[] = { MIPI_DCS_SET_DISPLAY_BRIGHTNESS, brightness >> 8, brightness & 0xff };
	u8 b6[] = { 0xb6, 0x05, 0xc0, 0x00, 0x11, 0x11, 0x11, 0x1c, 0x08,
		    0x1c, 0x21, 0x0b, 0x21, 0x0e, 0x21, 0x10 };
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(n17_36_0f_0c_dbv_vint); i++) {
		const u8 *row = n17_36_0f_0c_dbv_vint[i];
		u16 start = row[0] << 4 | row[1] >> 4;
		u16 end = (row[1] & 0xf) << 8 | row[2];

		if (brightness >= start && brightness <= end) {
			b6[15] = row[3];
			b6[13] = row[4];
			b6[11] = row[5];
			break;
		}
	}

	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, dbv, sizeof(dbv));
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9f, 0x02);
	mipi_dsi_generic_write_multi(&dsi_ctx, b6, sizeof(b6));

	pinfo->sent_brightness = dsi_ctx.accum_err ? -1 : brightness;
	return dsi_ctx.accum_err;
}

/*
 * Each command sent while video runs costs the bridge in front of the
 * panel a frame, so send only changes, and nothing while the panel is
 * powered off: prepare() sends the level along with the init sequence.
 */
static int n17_36_0f_0c_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct panel_info *pinfo = mipi_dsi_get_drvdata(dsi);
	u16 brightness = backlight_get_brightness(bl);

	if (!pinfo->supplies_enabled || brightness == pinfo->sent_brightness)
		return 0;

	return n17_36_0f_0c_set_brightness(pinfo, brightness);
}

static const struct backlight_ops n17_36_0f_0c_bl_ops = {
	.update_status = n17_36_0f_0c_bl_update_status,
};

static struct panel_desc n17_36_0f_0c_desc = {
	.modes = n17_36_0f_0c_modes,
	.num_modes = ARRAY_SIZE(n17_36_0f_0c_modes),
	.width_mm = 70,
	.height_mm = 155,
	.bpc = 8,
	.lanes = 4,
	.format = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
		      MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM,
	.init_sequence = n17_36_0f_0c_init_sequence,
	.reset_seq = n17_36_0f_0c_reset,
	.num_reset_steps = ARRAY_SIZE(n17_36_0f_0c_reset),
	.fps_cmds = n17_36_0f_0c_fps,
	.num_fps_cmds = ARRAY_SIZE(n17_36_0f_0c_fps),
	/* :413-418 */
	.display_off_ms = 10,
	/* DSC 1.1, as the bridge in front of it is set up (:73-105) */
	.dsc = {
		.dsc_version_major = 0x1,
		.dsc_version_minor = 0x1,
		.slice_height = 12,
		.slice_width = 540,
		.slice_count = 2,
		.bits_per_component = 8,
		.bits_per_pixel = 8 << 4,
		.block_pred_enable = true,
	},
	.hs_rate = 1016000000,
	.bl_ops = &n17_36_0f_0c_bl_ops,
	.max_brightness = 4095,
	/* the bootloader leaves it within 389-510 */
	.brightness = 450,
	.boot_on = true,
};

static struct backlight_device *icna35xx_create_backlight(struct mipi_dsi_device *dsi,
							  const struct panel_desc *desc)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = desc->brightness ?: 4096,
		.max_brightness = desc->max_brightness ?: 4096,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      desc->bl_ops ?: &icna35xx_bl_ops, &props);
}

static void icna35xx_release_power(void *data)
{
	struct panel_info *pinfo = data;

	if (pinfo->supplies_enabled)
		regulator_bulk_disable(ARRAY_SIZE(panel_supplies), pinfo->supplies);
}

/* Hold the supplies the bootloader left on, so that they stay on */
static int icna35xx_hold_boot_power(struct device *dev, struct panel_info *pinfo)
{
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(panel_supplies); i++)
		if (regulator_is_enabled(pinfo->supplies[i].consumer) <= 0)
			return 0;

	ret = regulator_bulk_enable(ARRAY_SIZE(panel_supplies), pinfo->supplies);
	if (ret)
		return ret;
	pinfo->supplies_enabled = true;

	return devm_add_action_or_reset(dev, icna35xx_release_power, pinfo);
}

static int icna35xx_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct panel_info *pinfo;
	int ret;

	pinfo = devm_drm_panel_alloc(dev, __typeof(*pinfo), panel,
				     &icna35xx_panel_funcs,
				     DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(pinfo))
		return PTR_ERR(pinfo);

	ret = devm_regulator_bulk_get_const(dev, ARRAY_SIZE(panel_supplies),
					    panel_supplies, &pinfo->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	pinfo->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(pinfo->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(pinfo->reset_gpio), "failed to get reset gpio\n");

	pinfo->desc = (struct panel_desc *)of_device_get_match_data(dev);
	if (!pinfo->desc)
		return -ENODEV;

	if (pinfo->desc->boot_on) {
		ret = icna35xx_hold_boot_power(dev, pinfo);
		if (ret)
			return dev_err_probe(dev, ret, "failed to keep the supplies on\n");
	}

	pinfo->dsi = dsi;
	pinfo->sent_brightness = -1;
	mipi_dsi_set_drvdata(dsi, pinfo);

	ret = drm_of_get_panel_orientation(dev->of_node, &pinfo->orientation);
	if (ret < 0) {
		dev_err(dev, "%pOF: failed to get orientation %d\n", dev->of_node, ret);
		return ret;
	}

	pinfo->panel.prepare_prev_first = true;

	pinfo->panel.backlight = icna35xx_create_backlight(dsi, pinfo->desc);
	if (IS_ERR(pinfo->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(pinfo->panel.backlight),
				     "Failed to create backlight\n");

	ret = devm_drm_panel_add(dev, &pinfo->panel);
	if (ret)
		return ret;

	pinfo->dsi->lanes = pinfo->desc->lanes;
	pinfo->dsi->format = pinfo->desc->format;
	pinfo->dsi->mode_flags = pinfo->desc->mode_flags;
	pinfo->dsi->dsc = &pinfo->desc->dsc;
	pinfo->dsi->hs_rate = pinfo->desc->hs_rate;

	return devm_mipi_dsi_attach(dev, dsi);
}

static const struct of_device_id icna35xx_of_match[] = {
	{ .compatible = "ayaneo,pocketds-panel-top", .data = &odin2portal_desc },
	{ .compatible = "ayntec,odin2portal-panel", .data = &odin2portal_desc },
	{ .compatible = "ayntec,odin3-panel", .data = &thor_top_desc },
	{ .compatible = "ayntec,thor-panel-top", .data = &thor_top_desc },
	{ .compatible = "xiaomi,gold-panel-n17-36-0f-0c", .data = &n17_36_0f_0c_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, icna35xx_of_match);

static struct mipi_dsi_driver icna35xx_driver = {
	.probe = icna35xx_probe,
	.driver = {
		.name = "panel-chipone-icna35xx",
		.of_match_table = icna35xx_of_match,
	},
};
module_mipi_dsi_driver(icna35xx_driver);

MODULE_AUTHOR("Teguh Sobirin <teguh@sobir.in>");
MODULE_DESCRIPTION("DRM driver for Chipone ICNA35XX based MIPI DSI panels");
MODULE_LICENSE("GPL");
