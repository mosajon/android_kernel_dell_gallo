#ifndef __LINUX_SOUNT_FM34_H__
#define __LINUX_SOUNT_FM34_H__
/*
 * fm34.h  --  FM34  DSP SoC Audio driver
 *
 * Copyright 2011 Pegatron
 *
 * Author: Andrew Hsiao<andrew_hsiao@pegatron.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
enum fm34_mode
{
	FM34_ENABLE_EC,
	FM34_BY_PASS,
};

enum fm34_audio_capture_mode
{
	AUDIO_CAPTURE_MODE_NONE,
	AUDIO_CAPTURE_MODE_INT_MIC_RECORD,
	AUDIO_CAPTURE_MODE_INT_MIC_INCALL,
	AUDIO_CAPTURE_MODE_HS_MIC_RECORD,
	AUDIO_CAPTURE_MODE_HS_MIC_INCALL,
	AUDIO_CAPTURE_MODE_INVALID,
};

struct fm34_platform_data
{
	unsigned int pin_id_bp;		/* hw by pass pin */
	unsigned int pin_id_rst;	/* reset pin */
	unsigned int pin_id_pwdn;	/* power down pin */

	enum fm34_mode mode_int_mic_record;
	enum fm34_mode mode_int_mic_incall;
	enum fm34_mode mode_hs_mic_record;
	enum fm34_mode mode_hs_mic_incall;

	unsigned int cmd_num;
	unsigned char *cmd_buf;
};

void fm34_set_mode(enum fm34_audio_capture_mode);
void fm34_power_up(void);

#endif
