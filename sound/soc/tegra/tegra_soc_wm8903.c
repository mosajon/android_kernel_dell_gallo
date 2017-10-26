/*
 * tegra_soc_wm8903.c  --  SoC audio for tegra
 *
 * (c) 2010-2011 Nvidia Graphics Pvt. Ltd.
 *  http://www.nvidia.com
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 * Author: Graeme Gregory
 *         graeme.gregory@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include "tegra_soc.h"
#include <linux/gpio.h>
#include <sound/soc-dapm.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include "../codecs/wm8903.h"

#include <sound/wm8903.h>
#include <sound/fm34.h>

static enum fm34_audio_capture_mode audio_capture_mode =
	AUDIO_CAPTURE_MODE_INVALID;

static struct platform_device *tegra_snd_device;

static struct regulator *reg_vmic = NULL;
extern int en_dmic;

extern struct snd_soc_dai tegra_i2s_dai[];
extern struct snd_soc_dai tegra_spdif_dai;
extern struct snd_soc_dai tegra_generic_codec_dai[];
extern struct snd_soc_platform tegra_soc_platform;
extern struct wired_jack_conf tegra_wired_jack_conf;

#define R06_MICBIAS_CTRL_0      6
#define B07_MICDET_HYST_ENA     7
#define B04_MICDET_THR          4
#define B02_MICSHORT_THR        2
#define B01_MICDET_ENA          1
#define B00_MICBIAS_ENA         0

#define R0C_INPUT_SEL           12
#define B01_INL_ENA             1
#define B00_INR_ENA             0

#define R0E_HP_PGA              14
#define B01_HPL_PGA_ENA         1
#define B00_HPR_PGA_ENA         0

#define R0F_LINEOUT_PGA         15
#define B01_LINEOUTL_PGA_ENA    1
#define B00_LINEOUTR_PGA_ENA    0

#define R12_ADCDAC_ENA          18
#define B03_DACL_ENA            3
#define B02_DACR_ENA            2
#define B01_ADCL_ENA            1
#define B00_ADCR_ENA            0

#define R18_AUDIO_INTERFACE_0   24
#define B12_DACL_DATINV         12
#define B11_DACR_DATINV         11
#define B09_DAC_BOOST           9
#define B08_LOOPBACK            8
#define B07_AIFADCL_SRC         7
#define B06_AIFADCR_SRC         6
#define B05_AIFDACL_SRC         5
#define B04_AIFDACR_SRC         4
#define B03_ADC_COMP            3
#define B02_ADC_COMPMODE        2
#define B01_DAC_COMP            1
#define B00_DAC_COMPMODE        0

#define R1E_DAC_DIGITAL_VOLUME_LEFT	30
#define R1F_DAC_DIGITAL_VOLUME_RIGHT	31
#define B08_DACVU	8
#define B00_DAC_VOL	0

#define R24_ADC_DIGITAL_VOLUME_LEFT	36
#define R25_ADC_DIGITAL_VOLUME_RIGHT	37
#define B08_ADCVU	8
#define B00_ADC_VOL	0

#define R26_ADC_DIGITAL_0       38
#define B05_ADC_HPF_CUT         5
#define B04_ADC_HPF_ENA         4
#define B01_ADCL_DATINV         1
#define B00_ADCR_DATINV         0

#define R28_DRC_0               40
#define B15_DRC_ENA             15
#define B11_DRC_THRESH_HYST     11
#define B06_DRC_STARTUP_GAIN    6
#define B05_DRC_FF_DELAY        5
#define B03_DRC_SMOOTH_ENA      3
#define B02_DRC_QR_ENA          2
#define B01_DRC_ANTICLIP_ENA    1
#define B00_DRC_HYST_ENA        0

#define R29_DRC_1               41
#define B12_DRC_ATTACK_RATE     12
#define B08_DRC_DECAY_RATE      8
#define B06_DRC_THRESH_QR       6
#define B04_DRC_RATE_QR         4
#define B02_DRC_MINGAIN         2
#define B00_DRC_MAXGAIN         0

#define R2A_DRC_2	42
#define B03_DRC_R0_SLOPE_COMP	3
#define B00_DRC_R1_SLOPE_COMP 0

#define R2B_DRC_3	43
#define B05_DRC_THRESH_COMP 5
#define B00_DRC_AMP_COMP 0

#define R2C_ANALOG_INPUT_L_0    44
#define R2D_ANALOG_INPUT_R_0    45
/* Common for both Left/Right analog input 0 */
#define B07_INEMUTE             7
#define B06_VOL_M3DB            6
#define B00_IN_VOL              0

#define R2E_ANALOG_INPUT_L_1    46
#define R2F_ANALOG_INPUT_R_1    47
/* Common for both Left/Right analog input 1 */
#define B06_IN_CM_ENA           6
#define B04_IP_SEL_N            4
#define B02_IP_SEL_P            2
#define B00_MODE                0

#define R39_HPOUTL_CTRL         57
#define B08_HPL_MUTE            8
#define B07_HPOUTVU             7
#define B06_HPOUTLZC            6
#define B00_HPOUTL_VOL          0

#define R3A_HPOUTR_CTRL         58
#define B08_HPR_MUTE            8
#define B06_HPOUTRZC            6
#define B00_HPOUTR_VOL          0

#define R3B_LINEOUTL_CTRL       59
#define B08_LINEOUTL_MUTE       8
#define B07_LINEOUTVU           7
#define B06_LINEOUTLZC          6
#define B00_LINEOUTL_VOL        0

#define R3C_LINEOUTR_CTRL       60
#define B08_LINEOUTR_MUTE       8
#define B06_LINEOUTRZC          6
#define B00_LINEOUTR_VOL        0


#define R3E_SPKL_CTRL           62
#define B08_SPKL_MUTE           8
#define B07_SPKVU               7
#define B06_SPKLZC              6
#define B00_SPKL_VOL            0

#define R3F_SPKR_CTRL           63
#define B08_SPKR_MUTE           8
#define B06_SPKRZC              6
#define B00_SPKR_VOL            0

#define R74_GPIO_CTRL_1         116
#define R75_GPIO_CTRL_2         117
#define R76_GPIO_CTRL_3         118
#define R77_GPIO_CTRL_4         119     /* Interupt */
#define B08_GPIO_FN             8
#define B07_GPIO_DIR            7
#define B06_GPIO_OP_CFG         6
#define B05_GPIO_IP_CFG         5
#define B04_GPIO_LVL            4
#define B03_GPIO_PD             3
#define B02_GPIO_PU             2
#define B01_GPIO_INTMODE        1
#define B00_GPIO_DB             0

#define R81_TEST_KEY            129
#define B01_USER_KEY            1
#define B00_TEST_KEY            0

#define RA4_ADC_DIG_MIC         164
#define B09_DIGMIC              9


/* codec register values */
#define B00_IN_VOL		0
#define B00_INR_ENA		0
#define B01_INL_ENA		1
#define B01_MICDET_ENA		1
#define B00_MICBIAS_ENA		0
#define B15_DRC_ENA		15
#define B01_ADCL_ENA		1
#define B00_ADCR_ENA		0
#define B06_IN_CM_ENA		6
#define B04_IP_SEL_N		4
#define B02_IP_SEL_P		2
#define B00_MODE 		0
#define B06_AIF_ADCL		7
#define B06_AIF_ADCR		6
#define B04_ADC_HPF_ENA		4
#define R20_SIDETONE_CTRL	32
#define R29_DRC_1		41

#define B08_GPx_FN		8
#define B07_GPx_DIR		7

#define DMIC_CLK_OUT		(0x6 << B08_GPx_FN)
#define DMIC_DAT_DATA_IN	(0x6 << B08_GPx_FN)
#define GPIO_DIR_OUT		(0x0 << B07_GPx_DIR)
#define GPIO_DIR_IN			(0x1 << B07_GPx_DIR)

#define ADC_DIGITAL_VOL_9DB		0x1D8
#define ADC_DIGITAL_VOL_12DB	0x1E0
#define ADC_ANALOG_VOLUME		0x1C
#define DRC_MAX_36DB			0x03

#define SET_REG_VAL(r,m,l,v) (((r)&(~((m)<<(l))))|(((v)&(m))<<(l)))

static ssize_t digital_mic_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%d\n", en_dmic);
}

static ssize_t digital_mic_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	if (count > 3) {
		pr_err("%s: buffer size %d too big\n", __func__, count);
		return -EINVAL;
	}

	if (sscanf(buf, "%d", &en_dmic) != 1) {
		pr_err("%s: invalid input string [%s]\n", __func__, buf);
		return -EINVAL;
	}
	return count;
}

static DEVICE_ATTR(enable_digital_mic, 0644, digital_mic_show, digital_mic_store);

static void configure_dmic(struct snd_soc_codec *codec)
{
	u16 test4, reg;

	if (en_dmic) {
		/* Set GP1_FN as DMIC_LR */
		snd_soc_write(codec, WM8903_GPIO_CONTROL_1,
					DMIC_CLK_OUT | GPIO_DIR_OUT);

		/* Set GP2_FN as DMIC_DAT */
		snd_soc_write(codec, WM8903_GPIO_CONTROL_2,
					DMIC_DAT_DATA_IN | GPIO_DIR_IN);

		/* Enable ADC Digital volumes */
		snd_soc_write(codec, WM8903_ADC_DIGITAL_VOLUME_LEFT,
					ADC_DIGITAL_VOL_9DB);
		snd_soc_write(codec, WM8903_ADC_DIGITAL_VOLUME_RIGHT,
					ADC_DIGITAL_VOL_9DB);

		/* Enable DIG_MIC */
		test4 = WM8903_ADC_DIG_MIC;
	} else {
		/* Disable DIG_MIC */
		test4 = snd_soc_read(codec, WM8903_CLOCK_RATE_TEST_4);
		test4 &= ~WM8903_ADC_DIG_MIC;
	}

	reg = snd_soc_read(codec, WM8903_CONTROL_INTERFACE_TEST_1);
	snd_soc_write(codec, WM8903_CONTROL_INTERFACE_TEST_1,
			 reg | WM8903_TEST_KEY);
	snd_soc_write(codec, WM8903_CLOCK_RATE_TEST_4, test4);
	snd_soc_write(codec, WM8903_CONTROL_INTERFACE_TEST_1, reg);

}


static struct tegra_audio_data audio_data;
static int tegra_jack_func;
static int tegra_spk_func;

int wm8903_connect_dac_to_mix(struct snd_soc_codec *codec, bool connect);
int wm8903_connect_dac_to_mixspk(struct snd_soc_codec *codec, bool connect);
int wm8903_config_digital_audio_channel_source(struct snd_soc_codec *codec,
	bool int_mic, bool connect);
int wm8903_apply_audio_gain_spk(struct snd_soc_codec *codec);
int wm8903_apply_audio_gain_headphone(struct snd_soc_codec *codec);
int wm8903_apply_audio_gain_lineout(struct snd_soc_codec *codec);
int wm8903_apply_audio_gain_mic(struct snd_soc_codec *codec);
int wm8903_apply_audio_gain_headset_mic(struct snd_soc_codec *codec);


static int tegra_hifi_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct tegra_audio_data* audio_data = rtd->socdev->codec_data;
	enum dac_dap_data_format data_fmt;
	int dai_flag = 0, sys_clk;
	int err;
	int SidetoneCtrlReg = 0;

	if (tegra_das_is_port_master(tegra_audio_codec_type_hifi))
		dai_flag |= SND_SOC_DAIFMT_CBM_CFM;
	else
		dai_flag |= SND_SOC_DAIFMT_CBS_CFS;

	data_fmt = tegra_das_get_codec_data_fmt(tegra_audio_codec_type_hifi);

	/* We are supporting DSP and I2s format for now */
	if (data_fmt & dac_dap_data_format_i2s)
		dai_flag |= SND_SOC_DAIFMT_I2S;
	else
		dai_flag |= SND_SOC_DAIFMT_DSP_A;

	err = snd_soc_dai_set_fmt(codec_dai, dai_flag);
	if (err < 0) {
		pr_err("codec_dai fmt not set \n");
		return err;
	}

	err = snd_soc_dai_set_fmt(cpu_dai, dai_flag);
	if (err < 0) {
		pr_err("cpu_dai fmt not set \n");
		return err;
	}

	sys_clk = clk_get_rate(audio_data->dap_mclk);
	err = snd_soc_dai_set_sysclk(codec_dai, 0, sys_clk, SND_SOC_CLOCK_IN);
	if (err < 0) {
		pr_err("codec_dai clock not set\n");
		return err;
	}

	err = snd_soc_dai_set_sysclk(cpu_dai, 0, sys_clk, SND_SOC_CLOCK_IN);
	if (err < 0) {
		pr_err("cpu_dai clock not set\n");
		return err;
	}

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK) {
		int CtrlReg = 0;
		int VolumeCtrlReg = 0;

		snd_soc_write(codec, WM8903_ANALOGUE_LEFT_INPUT_0, 0X7);
		snd_soc_write(codec, WM8903_ANALOGUE_RIGHT_INPUT_0, 0X7);
		/* Mic Bias enable */
		CtrlReg = (0x1<<B00_MICBIAS_ENA) | (0x1<<B01_MICDET_ENA);
		snd_soc_write(codec, WM8903_MIC_BIAS_CONTROL_0, CtrlReg);
		/* Single Ended Mic */
		CtrlReg = (0x0<<B06_IN_CM_ENA) |
			(0x0<<B00_MODE) | (0x0<<B04_IP_SEL_N)
					| (0x1<<B02_IP_SEL_P);
		VolumeCtrlReg = (0x1C << B00_IN_VOL);
		/* Mic Setting */
		snd_soc_write(codec, WM8903_ANALOGUE_LEFT_INPUT_1, CtrlReg);
		snd_soc_write(codec, WM8903_ANALOGUE_RIGHT_INPUT_1, CtrlReg);
		/* voulme for single ended mic */
		snd_soc_write(codec, WM8903_ANALOGUE_LEFT_INPUT_0,
				VolumeCtrlReg);
		snd_soc_write(codec, WM8903_ANALOGUE_RIGHT_INPUT_0,
				VolumeCtrlReg);
		/* Left ADC data on both channels */
		CtrlReg = snd_soc_read(codec, WM8903_AUDIO_INTERFACE_0);
		CtrlReg  = SET_REG_VAL(CtrlReg, 0x1, B06_AIF_ADCR, 0x0);
		CtrlReg  = SET_REG_VAL(CtrlReg, 0x1, B06_AIF_ADCL, 0x0);
		snd_soc_write(codec, WM8903_AUDIO_INTERFACE_0, CtrlReg);
		/* Enable analog inputs */
		CtrlReg = (0x1<<B01_INL_ENA);
		snd_soc_write(codec, WM8903_POWER_MANAGEMENT_0, CtrlReg);
		SidetoneCtrlReg = 0;
		snd_soc_write(codec, R20_SIDETONE_CTRL, SidetoneCtrlReg);
		/* Enable ADC */
		CtrlReg = snd_soc_read(codec, WM8903_POWER_MANAGEMENT_6);
		CtrlReg |= (0x1<<B01_ADCL_ENA);
		snd_soc_write(codec, WM8903_POWER_MANAGEMENT_6, CtrlReg);
	}

	return 0;
}

static int tegra_voice_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct tegra_audio_data* audio_data = rtd->socdev->codec_data;
	enum dac_dap_data_format data_fmt;
	int dai_flag = 0, sys_clk;
	int err;

	if (tegra_das_is_port_master(tegra_audio_codec_type_bluetooth))
		dai_flag |= SND_SOC_DAIFMT_CBM_CFM;
	else
		dai_flag |= SND_SOC_DAIFMT_CBS_CFS;

	data_fmt = tegra_das_get_codec_data_fmt(tegra_audio_codec_type_bluetooth);

	/* We are supporting DSP and I2s format for now */
	if (data_fmt & dac_dap_data_format_dsp)
		dai_flag |= SND_SOC_DAIFMT_DSP_A;
	else
		dai_flag |= SND_SOC_DAIFMT_I2S;

	err = snd_soc_dai_set_fmt(codec_dai, dai_flag);
	if (err < 0) {
		pr_err("codec_dai fmt not set \n");
		return err;
	}

	err = snd_soc_dai_set_fmt(cpu_dai, dai_flag);
	if (err < 0) {
		pr_err("cpu_dai fmt not set \n");
		return err;
	}

	sys_clk = clk_get_rate(audio_data->dap_mclk);
	err = snd_soc_dai_set_sysclk(codec_dai, 0, sys_clk, SND_SOC_CLOCK_IN);
	if (err < 0) {
		pr_err("cpu_dai clock not set\n");
		return err;
	}

	err = snd_soc_dai_set_sysclk(cpu_dai, 0, sys_clk, SND_SOC_CLOCK_IN);
	if (err < 0) {
		pr_err("cpu_dai clock not set\n");
		return err;
	}

	return 0;
}

static int tegra_spdif_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	return 0;
}

int tegra_codec_startup(struct snd_pcm_substream *substream)
{
	tegra_das_power_mode(true);

	if ((SNDRV_PCM_STREAM_CAPTURE == substream->stream) && en_dmic) {
		/* enable d-mic */
		if (reg_vmic) {
			regulator_enable(reg_vmic);
		}
	}

	return 0;
}

void tegra_codec_shutdown(struct snd_pcm_substream *substream)
{
	tegra_das_power_mode(false);

	if ((SNDRV_PCM_STREAM_CAPTURE == substream->stream) && en_dmic) {
		/* disable d-mic */
		if (reg_vmic) {
			regulator_disable(reg_vmic);
		}
	}
}

int tegra_soc_suspend_pre(struct platform_device *pdev, pm_message_t state)
{
	tegra_jack_suspend();
	return 0;
}

int tegra_soc_suspend_post(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct tegra_audio_data* audio_data = socdev->codec_data;

	clk_disable(audio_data->dap_mclk);

	return 0;
}

int tegra_soc_resume_pre(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct tegra_audio_data* audio_data = socdev->codec_data;

	clk_enable(audio_data->dap_mclk);

	return 0;
}

int tegra_soc_resume_post(struct platform_device *pdev)
{
	tegra_jack_resume();
	return 0;
}

static struct snd_soc_ops tegra_hifi_ops = {
	.hw_params = tegra_hifi_hw_params,
	.startup = tegra_codec_startup,
	.shutdown = tegra_codec_shutdown,
};

static struct snd_soc_ops tegra_voice_ops = {
	.hw_params = tegra_voice_hw_params,
	.startup = tegra_codec_startup,
	.shutdown = tegra_codec_shutdown,
};

static struct snd_soc_ops tegra_spdif_ops = {
	.hw_params = tegra_spdif_hw_params,
};

void tegra_ext_control(struct snd_soc_codec *codec, int new_con)
{
	struct tegra_audio_data* audio_data = codec->socdev->codec_data;

	/* Disconnect old codec routes and connect new routes*/
	if (new_con & TEGRA_HEADPHONE)
		snd_soc_dapm_enable_pin(codec, "Headphone");
	else
		snd_soc_dapm_disable_pin(codec, "Headphone");

	if (new_con & TEGRA_LINEOUT)
		snd_soc_dapm_enable_pin(codec, "Lineout");
	else
		snd_soc_dapm_disable_pin(codec, "Lineout");

	if (new_con & (TEGRA_SPK | TEGRA_EAR_SPK))
		snd_soc_dapm_enable_pin(codec, "Int Spk");
	else
		snd_soc_dapm_disable_pin(codec, "Int Spk");

	if (new_con & TEGRA_INT_MIC)
		snd_soc_dapm_enable_pin(codec, "Int Mic");
	else
		snd_soc_dapm_disable_pin(codec, "Int Mic");

	if (new_con & TEGRA_EXT_MIC)
		snd_soc_dapm_enable_pin(codec, "Ext Mic");
	else
		snd_soc_dapm_disable_pin(codec, "Ext Mic");

	if (new_con & TEGRA_LINEIN)
		snd_soc_dapm_enable_pin(codec, "Linein");
	else
		snd_soc_dapm_disable_pin(codec, "Linein");

	if (new_con & TEGRA_HEADSET)
		snd_soc_dapm_enable_pin(codec, "Headset Out");
	else
		snd_soc_dapm_disable_pin(codec, "Headset Out");

	if (new_con & TEGRA_HEADSET_MIC)
		snd_soc_dapm_enable_pin(codec, "Headset In");
	else
		snd_soc_dapm_disable_pin(codec, "Headset In");

	if (new_con & TEGRA_HEADSET_MIC)
		snd_soc_dapm_enable_pin(codec, "Headset Mic");
	else
		snd_soc_dapm_disable_pin(codec, "Headset Mic");

	/* signal a DAPM event */
	snd_soc_dapm_sync(codec);
	audio_data->codec_con = new_con;
}

void tegra_drc_config(struct snd_soc_codec *codec)
{
	struct tegra_audio_data* audio_data = codec->socdev->codec_data;
	struct wm8903_platform_data *pdata =
		dev_get_platdata(audio_data->codec->dev);
	int CtrlReg = 0;

	if (audio_data->is_call_mode) {
		switch (audio_data->capture_device) {
		case TEGRA_AUDIO_DEVICE_IN_BUILTIN_MIC:
		case TEGRA_AUDIO_DEVICE_IN_MIC:
		case TEGRA_AUDIO_DEVICE_IN_BACK_MIC:
			if (pdata->int_mic_incall_drc) {
				/* Enable DRC */
				snd_soc_write(codec, R28_DRC_0, pdata->int_mic_incall_drc0);
				snd_soc_write(codec, R29_DRC_1, pdata->int_mic_incall_drc1);
				snd_soc_write(codec, R2A_DRC_2, pdata->int_mic_incall_drc2);
				snd_soc_write(codec, R2B_DRC_3, pdata->int_mic_incall_drc3);
			} else {
				/* Disable DRC */
				CtrlReg = snd_soc_read(codec, R28_DRC_0);
				CtrlReg &= ~(0x1 << B15_DRC_ENA);
				snd_soc_write(codec, R28_DRC_0, CtrlReg);
			}
			break;
		case TEGRA_AUDIO_DEVICE_IN_HEADSET:
			if (pdata->hs_mic_incall_drc) {
				/* Enable DRC */
				snd_soc_write(codec, R28_DRC_0, pdata->hs_mic_incall_drc0);
				snd_soc_write(codec, R29_DRC_1, pdata->hs_mic_incall_drc1);
				snd_soc_write(codec, R2A_DRC_2, pdata->hs_mic_incall_drc2);
				snd_soc_write(codec, R2B_DRC_3, pdata->hs_mic_incall_drc3);
			} else {
				/* Disable DRC */
				CtrlReg = snd_soc_read(codec, R28_DRC_0);
				CtrlReg &= ~(0x1 << B15_DRC_ENA);
				snd_soc_write(codec, R28_DRC_0, CtrlReg);
			}
			break;
		}
	} else {
		switch (audio_data->capture_device) {
		case TEGRA_AUDIO_DEVICE_IN_BUILTIN_MIC:
		case TEGRA_AUDIO_DEVICE_IN_MIC:
		case TEGRA_AUDIO_DEVICE_IN_BACK_MIC:
			if (pdata->int_mic_record_drc) {
				/* Enable DRC */
				snd_soc_write(codec, R28_DRC_0, pdata->int_mic_record_drc0);
				snd_soc_write(codec, R29_DRC_1, pdata->int_mic_record_drc1);
				snd_soc_write(codec, R2A_DRC_2, pdata->int_mic_record_drc2);
				snd_soc_write(codec, R2B_DRC_3, pdata->int_mic_record_drc3);
			} else {
				/* Disable DRC */
				CtrlReg = snd_soc_read(codec, R28_DRC_0);
				CtrlReg &= ~(0x1 << B15_DRC_ENA);
				snd_soc_write(codec, R28_DRC_0, CtrlReg);
			}
			break;
		case TEGRA_AUDIO_DEVICE_IN_HEADSET:
			if (pdata->hs_mic_record_drc) {
				/* Enable DRC */
				snd_soc_write(codec, R28_DRC_0, pdata->hs_mic_record_drc0);
				snd_soc_write(codec, R29_DRC_1, pdata->hs_mic_record_drc1);
				snd_soc_write(codec, R2A_DRC_2, pdata->hs_mic_record_drc2);
				snd_soc_write(codec, R2B_DRC_3, pdata->hs_mic_record_drc3);
			} else {
				/* Disable DRC */
				CtrlReg = snd_soc_read(codec, R28_DRC_0);
				CtrlReg &= ~(0x1 << B15_DRC_ENA);
				snd_soc_write(codec, R28_DRC_0, CtrlReg);
			}
			break;
		}
	}
}

void tegra_hpf_config(struct snd_soc_codec *codec)
{
	struct tegra_audio_data* audio_data = codec->socdev->codec_data;
	struct wm8903_platform_data *pdata =
		dev_get_platdata(audio_data->codec->dev);
	int CtrlReg = 0;

	if (audio_data->is_call_mode) {
		switch (audio_data->capture_device) {
		case TEGRA_AUDIO_DEVICE_IN_BUILTIN_MIC:
		case TEGRA_AUDIO_DEVICE_IN_MIC:
		case TEGRA_AUDIO_DEVICE_IN_BACK_MIC:
			if (pdata->int_mic_incall_hpf) {
				/* Enable HPF */
				CtrlReg = snd_soc_read(codec, R26_ADC_DIGITAL_0);
				CtrlReg &= ~(0x3 << B05_ADC_HPF_CUT);
				CtrlReg |= ((pdata->int_mic_incall_hpf_cut << B05_ADC_HPF_CUT) |
					(0x1 << B04_ADC_HPF_ENA));
				snd_soc_write(codec, R26_ADC_DIGITAL_0, CtrlReg);
			} else {
				/* Disable HPF */
				CtrlReg = snd_soc_read(codec, R26_ADC_DIGITAL_0);
				CtrlReg &= ~(0x1 << B04_ADC_HPF_ENA);
				snd_soc_write(codec, R26_ADC_DIGITAL_0, CtrlReg);
			}
			break;
		case TEGRA_AUDIO_DEVICE_IN_HEADSET:
			if (pdata->hs_mic_incall_hpf) {
				/* Enable HPF */
				CtrlReg = snd_soc_read(codec, R26_ADC_DIGITAL_0);
				CtrlReg &= ~(0x3 << B05_ADC_HPF_CUT);
				CtrlReg |= ((pdata->hs_mic_incall_hpf_cut << B05_ADC_HPF_CUT) |
					(0x1 << B04_ADC_HPF_ENA));
				snd_soc_write(codec, R26_ADC_DIGITAL_0, CtrlReg);
			} else {
				/* Disable HPF */
				CtrlReg = snd_soc_read(codec, R26_ADC_DIGITAL_0);
				CtrlReg &= ~(0x1 << B04_ADC_HPF_ENA);
				snd_soc_write(codec, R26_ADC_DIGITAL_0, CtrlReg);
			}
			break;
		}
	} else {
		switch (audio_data->capture_device) {
		case TEGRA_AUDIO_DEVICE_IN_BUILTIN_MIC:
		case TEGRA_AUDIO_DEVICE_IN_MIC:
		case TEGRA_AUDIO_DEVICE_IN_BACK_MIC:
			if (pdata->int_mic_record_hpf) {
				/* Enable HPF */
				CtrlReg = snd_soc_read(codec, R26_ADC_DIGITAL_0);
				CtrlReg &= ~(0x3 << B05_ADC_HPF_CUT);
				CtrlReg |= ((pdata->int_mic_record_hpf_cut << B05_ADC_HPF_CUT) |
					(0x1 << B04_ADC_HPF_ENA));
				snd_soc_write(codec, R26_ADC_DIGITAL_0, CtrlReg);
			} else {
				/* Disable HPF */
				CtrlReg = snd_soc_read(codec, R26_ADC_DIGITAL_0);
				CtrlReg &= ~(0x1 << B04_ADC_HPF_ENA);
				snd_soc_write(codec, R26_ADC_DIGITAL_0, CtrlReg);
			}
			break;
		case TEGRA_AUDIO_DEVICE_IN_HEADSET:
			if (pdata->hs_mic_record_hpf) {
				/* Enable HPF */
				CtrlReg = snd_soc_read(codec, R26_ADC_DIGITAL_0);
				CtrlReg &= ~(0x3 << B05_ADC_HPF_CUT);
				CtrlReg |= ((pdata->hs_mic_record_hpf_cut << B05_ADC_HPF_CUT) |
					(0x1 << B04_ADC_HPF_ENA));
				snd_soc_write(codec, R26_ADC_DIGITAL_0, CtrlReg);
			} else {
				/* Disable HPF */
				CtrlReg = snd_soc_read(codec, R26_ADC_DIGITAL_0);
				CtrlReg &= ~(0x1 << B04_ADC_HPF_ENA);
				snd_soc_write(codec, R26_ADC_DIGITAL_0, CtrlReg);
			}
			break;
		}
	}
}

void tegra_fm34_config(struct snd_soc_codec *codec)
{
	struct tegra_audio_data* audio_data = codec->socdev->codec_data;

	enum fm34_audio_capture_mode audio_capture_mode_new;

	audio_capture_mode_new = AUDIO_CAPTURE_MODE_NONE;

	if (audio_data->is_int_mic_capturing || audio_data->is_hs_mic_capturing) {
		if (audio_data->is_call_mode) {
			switch (audio_data->capture_device) {
			case TEGRA_AUDIO_DEVICE_IN_BUILTIN_MIC:
			case TEGRA_AUDIO_DEVICE_IN_MIC:
			case TEGRA_AUDIO_DEVICE_IN_BACK_MIC:
				audio_capture_mode_new = AUDIO_CAPTURE_MODE_INT_MIC_INCALL;
				break;
			case TEGRA_AUDIO_DEVICE_IN_HEADSET:
				audio_capture_mode_new = AUDIO_CAPTURE_MODE_HS_MIC_INCALL;
				break;
			}
		} else {
			switch (audio_data->capture_device) {
			case TEGRA_AUDIO_DEVICE_IN_BUILTIN_MIC:
			case TEGRA_AUDIO_DEVICE_IN_MIC:
			case TEGRA_AUDIO_DEVICE_IN_BACK_MIC:
				audio_capture_mode_new = AUDIO_CAPTURE_MODE_INT_MIC_RECORD;
				break;
			case TEGRA_AUDIO_DEVICE_IN_HEADSET:
				audio_capture_mode_new = AUDIO_CAPTURE_MODE_HS_MIC_RECORD;
				break;
			}
		}
	}

	if (audio_capture_mode_new != audio_capture_mode) {
		audio_capture_mode = audio_capture_mode_new;
		fm34_set_mode(audio_capture_mode_new);
	}
}

static int tegra_int_spk(struct snd_soc_dapm_widget* w,
	struct snd_kcontrol *ctrl, int flag)
{
	printk("<%s>flag:%d,connected:%d\n", __FUNCTION__, flag, w->connected);

	switch(flag) {
	case SND_SOC_DAPM_PRE_PMU:
		break;
	case SND_SOC_DAPM_PRE_PMD:
		{
			int CtrlReg = 0;
			snd_soc_write(w->codec, R76_GPIO_CTRL_3, CtrlReg);

			CtrlReg = snd_soc_read(w->codec, R3E_SPKL_CTRL);
			CtrlReg |= (0x1 << B08_SPKL_MUTE);
			snd_soc_write(w->codec, R3E_SPKL_CTRL, CtrlReg);
			CtrlReg = snd_soc_read(w->codec, R3F_SPKR_CTRL);
			CtrlReg |= (0x1 << B08_SPKR_MUTE);
			snd_soc_write(w->codec, R3F_SPKR_CTRL, CtrlReg);
		}
		break;
	case SND_SOC_DAPM_POST_PMU:
		{
			int CtrlReg = 0;

			wm8903_apply_audio_gain_spk(audio_data.codec);

			CtrlReg = (1<<B05_GPIO_IP_CFG)|
				(1<<B00_GPIO_DB) | (1<<B01_GPIO_INTMODE);
			snd_soc_write(w->codec, R76_GPIO_CTRL_3, CtrlReg);

			msleep(1);

			CtrlReg |= (1<<B04_GPIO_LVL);
			snd_soc_write(w->codec, R76_GPIO_CTRL_3, CtrlReg);

			msleep(20);

			CtrlReg = snd_soc_read(w->codec, R3E_SPKL_CTRL);
			CtrlReg &= ~(0x1 << B08_SPKL_MUTE);
			snd_soc_write(w->codec, R3E_SPKL_CTRL, CtrlReg);
			CtrlReg = snd_soc_read(w->codec, R3F_SPKR_CTRL);
			CtrlReg &= ~(0x1 << B08_SPKR_MUTE);
			snd_soc_write(w->codec, R3F_SPKR_CTRL, CtrlReg);

		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		break;
	}

	tegra_fm34_config(audio_data.codec);

	return 0;
}

static int tegra_headset_mic(struct snd_soc_dapm_widget* w,
	struct snd_kcontrol *ctrl, int flag)
{
	printk("<%s>flag:%d,connected:%d\n", __FUNCTION__, flag, w->connected);

	switch(flag) {
	case SND_SOC_DAPM_PRE_PMU:
		{
			if(audio_data.capture_device | TEGRA_AUDIO_DEVICE_IN_HEADSET) {
				int CtrlReg = 0;
				struct wm8903_platform_data *pdata =
					dev_get_platdata(audio_data.codec->dev);

				// Mic Bias enable
				CtrlReg = (0x1 << B00_MICBIAS_ENA) | (0x1 << B01_MICDET_ENA);
				snd_soc_write(w->codec, R06_MICBIAS_CTRL_0, CtrlReg);

				// voulme for single ended mic
				wm8903_apply_audio_gain_headset_mic(audio_data.codec);
				CtrlReg = snd_soc_read(w->codec, R2C_ANALOG_INPUT_L_0);
				CtrlReg &= ~(0x1 << B07_INEMUTE);
				snd_soc_write(w->codec, R2C_ANALOG_INPUT_L_0, CtrlReg);
				CtrlReg = snd_soc_read(w->codec, R2D_ANALOG_INPUT_R_0);
				CtrlReg &= ~(0x1 << B07_INEMUTE);
				snd_soc_write(w->codec, R2D_ANALOG_INPUT_R_0, CtrlReg);

				// Single Ended Mic
				CtrlReg = (0x0 << B06_IN_CM_ENA) | (0x0 << B00_MODE) |
					(0x0 << B04_IP_SEL_N) | (0x1 << B02_IP_SEL_P);
				snd_soc_write(w->codec, R2E_ANALOG_INPUT_L_1, CtrlReg);
				snd_soc_write(w->codec, R2F_ANALOG_INPUT_R_1, CtrlReg);

				wm8903_config_digital_audio_channel_source(audio_data.codec,
					false, true);

				// Enable analog inputs
				CtrlReg = (0x1 << B01_INL_ENA) | (0x1 << B00_INR_ENA);
				snd_soc_write(w->codec, R0C_INPUT_SEL, CtrlReg);

				/* Enable ADC */
				CtrlReg = snd_soc_read(w->codec, R12_ADCDAC_ENA);
				CtrlReg |= ((0x1 << B00_ADCR_ENA) | (0x1 << B01_ADCL_ENA));
				snd_soc_write(w->codec, R12_ADCDAC_ENA, CtrlReg);

				/* config ADC_DIG_MIC */
				snd_soc_write(w->codec, R81_TEST_KEY, (0x1 << B01_USER_KEY));
				snd_soc_write(w->codec, RA4_ADC_DIG_MIC, (0x0 << B09_DIGMIC));
				snd_soc_write(w->codec, R81_TEST_KEY, (0x0 << B01_USER_KEY));

				if (pdata->pin_id_en_mic_ext_n > 0) {
					gpio_set_value(pdata->pin_id_en_mic_ext_n, 0);
				}

				audio_data.is_hs_mic_capturing = true;
			}
		}
		break;
	case SND_SOC_DAPM_PRE_PMD:
		break;
	case SND_SOC_DAPM_POST_PMU:
		break;
	case SND_SOC_DAPM_POST_PMD:
		{
			if(audio_data.capture_device | TEGRA_AUDIO_DEVICE_IN_HEADSET) {
				int CtrlReg = 0;
				struct wm8903_platform_data *pdata =
					dev_get_platdata(audio_data.codec->dev);

				CtrlReg = snd_soc_read(w->codec, R2C_ANALOG_INPUT_L_0);
				CtrlReg |= (0x1 << B07_INEMUTE);
				snd_soc_write(w->codec, R2C_ANALOG_INPUT_L_0, CtrlReg);
				CtrlReg = snd_soc_read(w->codec, R2D_ANALOG_INPUT_R_0);
				CtrlReg |= (0x1 << B07_INEMUTE);
				snd_soc_write(w->codec, R2D_ANALOG_INPUT_R_0, CtrlReg);


				// Mic Bias disable
				CtrlReg = 0;
				snd_soc_write(w->codec, R06_MICBIAS_CTRL_0, CtrlReg);

				// Disable analog inputs
				CtrlReg = 0;
				snd_soc_write(w->codec, R0C_INPUT_SEL, CtrlReg);

				wm8903_config_digital_audio_channel_source(audio_data.codec,
					false, false);

				/* Disable ADC */
				CtrlReg = snd_soc_read(w->codec, R12_ADCDAC_ENA);
				CtrlReg |= ((0x0 << B00_ADCR_ENA) | (0x0 << B01_ADCL_ENA));
				snd_soc_write(w->codec, R12_ADCDAC_ENA, CtrlReg);

				/* config ADC_DIG_MIC */
				snd_soc_write(w->codec, R81_TEST_KEY, (0x1 << B01_USER_KEY));
				snd_soc_write(w->codec, RA4_ADC_DIG_MIC, (0x1 << B09_DIGMIC));
				snd_soc_write(w->codec, R81_TEST_KEY, (0x0 << B01_USER_KEY));

				if (pdata->pin_id_en_mic_ext_n > 0) {
					gpio_set_value(pdata->pin_id_en_mic_ext_n, 1);
				}

				audio_data.is_hs_mic_capturing = false;
			}
		}
		break;
	}

	tegra_drc_config(audio_data.codec);
	tegra_hpf_config(audio_data.codec);
	tegra_fm34_config(audio_data.codec);

	return 0;
}

static int tegra_mic(struct snd_soc_dapm_widget* w,
	struct snd_kcontrol *ctrl, int flag)
{
	printk("<%s>flag:%d,connected:%d\n", __FUNCTION__, flag, w->connected);

	switch(flag) {
	case SND_SOC_DAPM_PRE_PMU:
		{
			int CtrlReg = 0;

			wm8903_apply_audio_gain_mic(audio_data.codec);

			// mic setting on both channels
			wm8903_config_digital_audio_channel_source(audio_data.codec,
				true, true);

			/* Enable ADC */
			CtrlReg = snd_soc_read(w->codec, R12_ADCDAC_ENA);
			CtrlReg |= ((0x1 << B00_ADCR_ENA) | (0x1 << B01_ADCL_ENA));
			snd_soc_write(w->codec, R12_ADCDAC_ENA, CtrlReg);

			/* config DMIC_LR, DMIC_DAT */
			snd_soc_write(w->codec, R74_GPIO_CTRL_1,
				(0x1 << B00_GPIO_DB) | (0x06 << B08_GPIO_FN) );
			snd_soc_write(w->codec, R75_GPIO_CTRL_2,
				(0x1 << B05_GPIO_IP_CFG) | (0x1 << B07_GPIO_DIR) |
				(0x06 << B08_GPIO_FN));

			/* config ADC_DIG_MIC */
			snd_soc_write(w->codec, R81_TEST_KEY, (0x1 << B01_USER_KEY));
			snd_soc_write(w->codec, RA4_ADC_DIG_MIC, (0x1 << B09_DIGMIC));
			snd_soc_write(w->codec, R81_TEST_KEY, (0x0 << B01_USER_KEY));

			audio_data.is_int_mic_capturing = true;
		}
		break;
	case SND_SOC_DAPM_PRE_PMD:
		break;
	case SND_SOC_DAPM_POST_PMU:
		break;
	case SND_SOC_DAPM_POST_PMD:
		{
			int CtrlReg = 0;
			msleep(10);
			// mic setting on both channels
			wm8903_config_digital_audio_channel_source(audio_data.codec,
				true, false);

			/* Disable ADC */
			CtrlReg = snd_soc_read(w->codec, R12_ADCDAC_ENA);
			CtrlReg &= ~((0x1 << B00_ADCR_ENA) | (0x1 << B01_ADCL_ENA));
			snd_soc_write(w->codec, R12_ADCDAC_ENA, CtrlReg);

			/* config DMIC_LR, DMIC_DAT */
			snd_soc_write(w->codec, R74_GPIO_CTRL_1,(0x1 << B03_GPIO_PD) |
				(0x1 << B05_GPIO_IP_CFG) | (0x1 << B07_GPIO_DIR));
			snd_soc_write(w->codec, R75_GPIO_CTRL_2, (0x1 << B03_GPIO_PD) |
				(0x1 << B05_GPIO_IP_CFG) | (0x1 << B07_GPIO_DIR));

			/* config ADC_DIG_MIC */
			snd_soc_write(w->codec, R81_TEST_KEY, (0x1 << B01_USER_KEY));
			snd_soc_write(w->codec, RA4_ADC_DIG_MIC, (0x1 << B09_DIGMIC));
			snd_soc_write(w->codec, R81_TEST_KEY, (0x0 << B01_USER_KEY));

			audio_data.is_int_mic_capturing = false;
		}
		break;
	}

	tegra_drc_config(audio_data.codec);
	tegra_hpf_config(audio_data.codec);
	tegra_fm34_config(audio_data.codec);

	return 0;
}

static int tegra_headphone(struct snd_soc_dapm_widget* w,
	struct snd_kcontrol *ctrl, int flag)
{
	printk("<%s>flag:%d,connected:%d\n", __FUNCTION__, flag, w->connected);

	switch(flag) {
	case SND_SOC_DAPM_PRE_PMU:
		break;
	case SND_SOC_DAPM_PRE_PMD:
		{
			int CtrlReg = 0;

			CtrlReg = snd_soc_read(w->codec, R0E_HP_PGA);
			CtrlReg &= ~(0x1 << B00_HPR_PGA_ENA) | (0x1 << B01_HPL_PGA_ENA);
			snd_soc_write(w->codec, R0E_HP_PGA, CtrlReg);

			CtrlReg = snd_soc_read(w->codec, R39_HPOUTL_CTRL);
			CtrlReg |= (0x1 << B08_HPL_MUTE);
			snd_soc_write(w->codec, R39_HPOUTL_CTRL, CtrlReg);
			CtrlReg = snd_soc_read(w->codec, R3A_HPOUTR_CTRL);
			CtrlReg |= (0x1 << B08_HPR_MUTE);
			snd_soc_write(w->codec, R3A_HPOUTR_CTRL, CtrlReg);
		}
		break;
	case SND_SOC_DAPM_POST_PMU:
		{
			int CtrlReg = 0;

			wm8903_apply_audio_gain_headphone(audio_data.codec);
			CtrlReg = snd_soc_read(w->codec, R39_HPOUTL_CTRL);
			CtrlReg &= ~(0x1 << B08_HPL_MUTE);
			snd_soc_write(w->codec, R39_HPOUTL_CTRL, CtrlReg);
			CtrlReg = snd_soc_read(w->codec, R3A_HPOUTR_CTRL);
			CtrlReg &= ~(0x1 << B08_HPR_MUTE);
			snd_soc_write(w->codec, R3A_HPOUTR_CTRL, CtrlReg);

			CtrlReg = snd_soc_read(w->codec, R0E_HP_PGA);
			CtrlReg |= (0x1 << B00_HPR_PGA_ENA) | (0x1 << B01_HPL_PGA_ENA);
			snd_soc_write(w->codec, R0E_HP_PGA, CtrlReg);
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		break;
	}

	tegra_fm34_config(audio_data.codec);

	return 0;
}

static int tegra_lineout(struct snd_soc_dapm_widget* w,
	struct snd_kcontrol *ctrl, int flag)
{
	printk("<%s>flag:%d,connected:%d\n", __FUNCTION__, flag, w->connected);

	switch(flag) {
	case SND_SOC_DAPM_PRE_PMU:
		break;
	case SND_SOC_DAPM_PRE_PMD:
		{
			int CtrlReg = 0;

			CtrlReg = snd_soc_read(w->codec, R0F_LINEOUT_PGA);
			CtrlReg &= ~(0x1 << B00_LINEOUTR_PGA_ENA) |
				(0x1 << B01_LINEOUTL_PGA_ENA);
			snd_soc_write(w->codec, R0F_LINEOUT_PGA, CtrlReg);

			CtrlReg = snd_soc_read(w->codec, R3B_LINEOUTL_CTRL);
			CtrlReg |= (0x1 << B08_LINEOUTL_MUTE);
			snd_soc_write(w->codec, R3B_LINEOUTL_CTRL, CtrlReg);
			CtrlReg = snd_soc_read(w->codec, R3C_LINEOUTR_CTRL);
			CtrlReg |= (0x1 << B08_LINEOUTR_MUTE);
			snd_soc_write(w->codec, R3C_LINEOUTR_CTRL, CtrlReg);
		}
		break;
	case SND_SOC_DAPM_POST_PMU:
		{
			int CtrlReg = 0;

			wm8903_apply_audio_gain_lineout(audio_data.codec);
			CtrlReg = snd_soc_read(w->codec, R3B_LINEOUTL_CTRL);
			CtrlReg &= ~(0x1 << B08_LINEOUTL_MUTE);
			snd_soc_write(w->codec, R3B_LINEOUTL_CTRL, CtrlReg);
			CtrlReg = snd_soc_read(w->codec, R3C_LINEOUTR_CTRL);
			CtrlReg &= ~(0x1 << B08_LINEOUTR_MUTE);
			snd_soc_write(w->codec, R3C_LINEOUTR_CTRL, CtrlReg);

			CtrlReg = snd_soc_read(w->codec, R0F_LINEOUT_PGA);
			CtrlReg |= (0x1 << B00_LINEOUTR_PGA_ENA) |
				(0x1 << B01_LINEOUTL_PGA_ENA);
			snd_soc_write(w->codec, R0F_LINEOUT_PGA, CtrlReg);
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		break;
	}

	tegra_fm34_config(audio_data.codec);

	return 0;
}


/*tegra machine dapm widgets */
static const struct snd_soc_dapm_widget tegra_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", tegra_headphone),
	SND_SOC_DAPM_HP("Headset", tegra_headphone),
	SND_SOC_DAPM_LINE("Lineout", tegra_lineout),
	SND_SOC_DAPM_SPK("Int Spk", tegra_int_spk),
	SND_SOC_DAPM_MIC("Ext Mic", NULL),
	SND_SOC_DAPM_MIC("Int Mic", tegra_mic),
	SND_SOC_DAPM_LINE("Linein", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", tegra_headset_mic),
};

/* Tegra machine audio map (connections to the codec pins) */
static const struct snd_soc_dapm_route audio_map[] = {

	/* headphone connected to LHPOUT1, RHPOUT1 */
	{"Headphone", NULL, "HPOUTR"},
	{"Headphone", NULL, "HPOUTL"},

	/* headset Jack  - in = micin, out = HPOUT*/
	{"Headset", NULL, "HPOUTR"},
	{"Headset", NULL, "HPOUTL"},

	/* lineout connected to LINEOUTR and LINEOUTL */
	{"Lineout", NULL, "LINEOUTR"},
	{"Lineout", NULL, "LINEOUTL"},

	/* build-in speaker connected to LON/P RON/P */
	{"Int Spk", NULL, "RON"},
	{"Int Spk", NULL, "ROP"},
	{"Int Spk", NULL, "LON"},
	{"Int Spk", NULL, "LOP"},

	{"ADCL", NULL, "Int Mic"},
	{"ADCR", NULL, "Int Mic"},

	{"IN1R", NULL, "Headset Mic"},
};

static int tegra_codec_init(struct snd_soc_codec *codec)
{
	struct tegra_audio_data* audio_data = codec->socdev->codec_data;
	int err = 0;

	if (!audio_data->init_done) {
		audio_data->dap_mclk = tegra_das_get_dap_mclk();
		if (!audio_data->dap_mclk) {
			pr_err("Failed to get dap mclk \n");
			err = -ENODEV;
			return err;
		}

		/* Add tegra specific widgets */
		snd_soc_dapm_new_controls(codec, tegra_dapm_widgets,
					ARRAY_SIZE(tegra_dapm_widgets));

		/* Set up tegra specific audio path audio_map */
		snd_soc_dapm_add_routes(codec, audio_map,
					ARRAY_SIZE(audio_map));

		/* Add jack detection */
		err = tegra_jack_init(codec);
		if (err < 0) {
			pr_err("Failed in jack init \n");
			return err;
		}

		/* Default to OFF */
		tegra_ext_control(codec, TEGRA_AUDIO_OFF);

		err = tegra_controls_init(codec);
		if (err < 0) {
			pr_err("Failed in controls init \n");
			return err;
		}

		audio_data->codec = codec;
		audio_data->init_done = 1;
	}

	if (!audio_data) {
		audio_data = kzalloc(sizeof(*audio_data), GFP_KERNEL);
		if (!audio_data) {
			pr_err("failed to allocate tegra_audio_data \n");
			return -ENOMEM;
		}

		audio_data->codec = codec;
	}

	return err;
}

static struct snd_soc_dai_link tegra_soc_dai[] = {
	{
		.name = "WM8903",
		.stream_name = "WM8903 HiFi",
		.cpu_dai = &tegra_i2s_dai[0],
		.codec_dai = &wm8903_dai,
		.init = tegra_codec_init,
		.ops = &tegra_hifi_ops,
	},
	{
		.name = "Tegra-generic",
		.stream_name = "Tegra Generic Voice",
		.cpu_dai = &tegra_i2s_dai[1],
		.codec_dai = &tegra_generic_codec_dai[0],
		.init = tegra_codec_init,
		.ops = &tegra_voice_ops,
	},
	{
		.name = "Tegra-spdif",
		.stream_name = "Tegra Spdif",
		.cpu_dai = &tegra_spdif_dai,
		.codec_dai = &tegra_generic_codec_dai[1],
		.init = tegra_codec_init,
		.ops = &tegra_spdif_ops,
	},
};

static struct tegra_audio_data audio_data = {
	.init_done = 0,
	.play_device = TEGRA_AUDIO_DEVICE_NONE,
	.capture_device = TEGRA_AUDIO_DEVICE_NONE,
	.is_call_mode = false,
	.is_int_mic_capturing = false,
	.is_hs_mic_capturing = false,
	.codec_con = TEGRA_AUDIO_OFF,
};

static struct snd_soc_card tegra_snd_soc = {
	.name = "tegra",
	.platform = &tegra_soc_platform,
	.dai_link = tegra_soc_dai,
	.num_links = ARRAY_SIZE(tegra_soc_dai),
	.suspend_pre = tegra_soc_suspend_pre,
	.suspend_post = tegra_soc_suspend_post,
	.resume_pre = tegra_soc_resume_pre,
	.resume_post = tegra_soc_resume_post,
};

static struct snd_soc_device tegra_snd_devdata = {
	.card = &tegra_snd_soc,
	.codec_dev = &soc_codec_dev_wm8903,
	.codec_data = &audio_data,
};

static int __init tegra_init(void)
{
	int ret = 0;

	tegra_snd_device = platform_device_alloc("soc-audio", -1);
	if (!tegra_snd_device) {
		pr_err("failed to allocate soc-audio \n");
		return -ENOMEM;
	}

	platform_set_drvdata(tegra_snd_device, &tegra_snd_devdata);
	tegra_snd_devdata.dev = &tegra_snd_device->dev;

	ret = platform_device_add(tegra_snd_device);
	if (ret) {
		pr_err("audio device could not be added \n");
		goto fail;
	}

	ret = device_create_file(&tegra_snd_device->dev,
							&dev_attr_enable_digital_mic);
	if (ret < 0) {
		dev_err(&tegra_snd_device->dev,
				"%s: could not create sysfs entry %s: %d\n",
				__func__, dev_attr_enable_digital_mic.attr.name, ret);
		goto fail;
	}

	reg_vmic = regulator_get(&tegra_snd_device->dev, "vmic");
	if (IS_ERR_OR_NULL(reg_vmic)) {
		pr_err("Couldn't get vmic regulator\n");
		reg_vmic = NULL;
	}

	return 0;

fail:
	if (tegra_snd_device) {
		platform_device_put(tegra_snd_device);
		tegra_snd_device = 0;
	}

	return ret;
}

static void __exit tegra_exit(void)
{
	tegra_jack_exit();
	if (reg_vmic) {
		regulator_put(reg_vmic);
		reg_vmic = NULL;
	}
	platform_device_unregister(tegra_snd_device);
}

module_init(tegra_init);
module_exit(tegra_exit);

/* Module information */
MODULE_DESCRIPTION("Tegra ALSA SoC");
MODULE_LICENSE("GPL");
