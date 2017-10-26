/*
 * drivers/mmc/host/sdhci-tegra.c
 *
 * Copyright (C) 2009 Palm, Inc.
 * Author: Yvonne Yip <y@palm.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/mmc/card.h>
#include "../core/core.h"

#include <mach/sdhci.h>

#include "sdhci.h"

#define DRIVER_NAME    "sdhci-tegra"

#define SDHCI_VENDOR_CLOCK_CNTRL       0x100

#define SD_TIME_GAP_FOR_SUSPEND 2500
#define SD_POLLING_TIMEOUT 10000

struct tegra_sdhci_host {
	struct sdhci_host *sdhci;
	struct platform_device	*pdev;
	struct clk *clk;
	int clk_enabled;
	bool card_always_on;
	u32 sdhci_ints;
	int wp_gpio;
	int wp_gpio_polarity;

	int card_present;
	bool	card_present_old;
	int cd_gpio;
	int cd_gpio_polarity;
	int irq_cd;
	int power_gpio;
	int power_gpio_polarity;

	int *gpio_pins;
	int nr_gpio_pins;
	unsigned long card_detection_time;
};


static void sdhc_pinmux_config_gpio(struct tegra_sdhci_host *host)
{
	int i = 0;
	int ret = -ENODEV;

	for (i = 0; i < host->nr_gpio_pins; i++) {
		ret = gpio_request(host->gpio_pins[i], "sdhc_gpio");
		tegra_gpio_enable(host->gpio_pins[i]);
		ret = gpio_direction_output(host->gpio_pins[i], 0);

		if (ret < 0) {
			dev_err(&host->pdev->dev, "failed to configure GPIO: %d\n", host->gpio_pins[i]);
			gpio_free(host->gpio_pins[i]);
			break;
		}
	}
}

static void sdhc_pinmux_config_gpio_enable(struct tegra_sdhci_host *host)
{
	int i = 0;
	for (i = 0; i < host->nr_gpio_pins; i++) {
		tegra_gpio_enable(host->gpio_pins[i]);
	}
}

static void sdhc_pinmux_config_gpio_disable(struct tegra_sdhci_host *host)
{
	int i = 0;
	for (i = 0; i < host->nr_gpio_pins; i++) {
		tegra_gpio_disable(host->gpio_pins[i]);
	}
}

static void tegra_do_initial_sequence(struct sdhci_host *sdhost)
{
	struct tegra_sdhci_host *host = sdhci_priv(sdhost);

	dev_info(&host->pdev->dev, "%s, %s: card_present = %d\n", __func__,
				mmc_hostname(sdhost->mmc), host->card_present);
	if (host->card_present) {
		if (host->power_gpio != -1) {
			gpio_set_value(host->power_gpio,  host->power_gpio_polarity);
			mmc_delay(10);
			sdhc_pinmux_config_gpio_disable(host);
			mmc_delay(50);
		}
	} else {
		if (host->power_gpio != -1) {
			sdhc_pinmux_config_gpio_enable(host);
			gpio_set_value(host->power_gpio, !host->power_gpio_polarity);
		}
	}
}

static void tegra_power_off(struct sdhci_host *sdhost)
{
	struct tegra_sdhci_host *host = sdhci_priv(sdhost);
	if (host->power_gpio != -1) {
		sdhc_pinmux_config_gpio_enable(host);
		gpio_set_value(host->power_gpio, !host->power_gpio_polarity);
		mmc_delay(10);
	}
}

static bool tegra_is_polling_timeout(struct sdhci_host *sdhost)
{
	struct tegra_sdhci_host *host = sdhci_priv(sdhost);

	/* If card present but scan fail, continue on polling
	*   until polling time expires
	*/
	if (time_before(jiffies, host->card_detection_time + msecs_to_jiffies(SD_POLLING_TIMEOUT))) {
		printk(KERN_INFO "%s: Before polling time,  continue on polling \n", __func__);
		return false;
	} else {
		printk(KERN_INFO "%s: Polling timeout\n", __func__);
		return true;
	}
}

static irqreturn_t carddetect_irq(int irq, void *data)
{
	struct sdhci_host *sdhost = (struct sdhci_host *)data;
	struct tegra_sdhci_host *host = sdhci_priv(sdhost);

	host->card_present =
		(gpio_get_value(host->cd_gpio) == host->cd_gpio_polarity);
	host->card_detection_time = jiffies;

	if (host->card_present ^ host->card_present_old) {
		dev_info(&host->pdev->dev, "%s: card_present = %d\n",
				mmc_hostname(sdhost->mmc), host->card_present);
		tasklet_schedule(&sdhost->card_tasklet);
	}
	host->card_present_old = host->card_present;
	return IRQ_HANDLED;
};

static void tegra_sdhci_status_notify_cb(int card_present, void *dev_id)
{
	struct sdhci_host *sdhci = (struct sdhci_host *)dev_id;
	pr_debug("%s: card_present %d\n",
		mmc_hostname(sdhci->mmc), card_present);
	sdhci_card_detect_callback(sdhci);
}

static int tegra_sdhci_enable_dma(struct sdhci_host *host)
{
	return 0;
}

static void tegra_sdhci_enable_clock(struct tegra_sdhci_host *host, int enable)
{
	if (enable && !host->clk_enabled) {
		clk_enable(host->clk);
		sdhci_writeb(host->sdhci, 1, SDHCI_VENDOR_CLOCK_CNTRL);
		host->clk_enabled = 1;
	} else if (!enable && host->clk_enabled) {
		sdhci_writeb(host->sdhci, 0, SDHCI_VENDOR_CLOCK_CNTRL);
		clk_disable(host->clk);
		host->clk_enabled = 0;
	}
}

static void tegra_sdhci_set_clock(struct sdhci_host *sdhci, unsigned int clock)
{
	struct tegra_sdhci_host *host = sdhci_priv(sdhci);
	pr_debug("tegra sdhci clock %s %u enabled=%d\n",
		mmc_hostname(sdhci->mmc), clock, host->clk_enabled);

	tegra_sdhci_enable_clock(host, clock);
}

static int tegra_sdhci_card_detect(struct sdhci_host *sdhost)
{
	struct tegra_sdhci_host *host = sdhci_priv(sdhost);

	if (host->cd_gpio != -1) {
		dev_dbg(&host->pdev->dev, "%s, %s: card_present = %d\n", __func__,
				mmc_hostname(sdhost->mmc), host->card_present);
	}
	return host->card_present;
}

static int tegra_sdhci_get_ro(struct sdhci_host *sdhci)
{
	struct tegra_sdhci_host *host;
	host = sdhci_priv(sdhci);
	if (gpio_is_valid(host->wp_gpio))
		return gpio_get_value(host->wp_gpio);
	return 0;
}

static struct sdhci_ops tegra_sdhci_ops = {
	.enable_dma = tegra_sdhci_enable_dma,
	.set_clock = tegra_sdhci_set_clock,
	.get_ro = tegra_sdhci_get_ro,
	.card_detect = tegra_sdhci_card_detect,
	.do_initial_sequence = tegra_do_initial_sequence,
	.power_off = tegra_power_off,
	.is_polling_timeout = tegra_is_polling_timeout,
};

static int __devinit tegra_sdhci_probe(struct platform_device *pdev)
{
	int rc;
	struct tegra_sdhci_platform_data *plat;
	struct sdhci_host *sdhci;
	struct tegra_sdhci_host *host;
	struct resource *res;
	int irq;
	void __iomem *ioaddr;

	plat = pdev->dev.platform_data;
	if (plat == NULL)
		return -ENXIO;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL)
		return -ENODEV;

	irq = res->start;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL)
		return -ENODEV;

	ioaddr = ioremap(res->start, res->end - res->start);

	sdhci = sdhci_alloc_host(&pdev->dev, sizeof(struct tegra_sdhci_host));
	if (IS_ERR(sdhci)) {
		rc = PTR_ERR(sdhci);
		goto err_unmap;
	}

	host = sdhci_priv(sdhci);

	/* default value for gpio polarity */
	host->cd_gpio_polarity = 0;
	host->wp_gpio_polarity = 1;
	host->power_gpio_polarity = 1;

	host->sdhci = sdhci;
	host->card_always_on = (plat->power_gpio == -1) ? 1 : 0;

	host->pdev = pdev;
	host->gpio_pins = plat->gpio_pins;
	host->nr_gpio_pins = plat->nr_gpio_pins;
	host->wp_gpio = plat->wp_gpio;
	host->wp_gpio_polarity = plat->wp_gpio_polarity;
	host->cd_gpio = plat->cd_gpio;
	host->cd_gpio_polarity = plat->cd_gpio_polarity;
	host->power_gpio = plat->power_gpio;
	host->power_gpio_polarity = plat->power_gpio_polarity;
	host->card_detection_time = jiffies;

	dev_info(&pdev->dev, "write protect: %d card detect: %d, power gpio: %d \n",
		host->wp_gpio, host->cd_gpio, host->power_gpio);

	if (host->cd_gpio == -1) {
		host->card_present = host->card_present = true;
	}

	sdhc_pinmux_config_gpio(host);
	host->clk = clk_get(&pdev->dev, plat->clk_id);
	if (IS_ERR(host->clk)) {
		rc = PTR_ERR(host->clk);
		goto err_free_host;
	}

	rc = clk_enable(host->clk);
	if (rc != 0)
		goto err_clkput;

	host->clk_enabled = 1;
	sdhci->hw_name = "tegra";
	sdhci->ops = &tegra_sdhci_ops;
	sdhci->irq = irq;
	sdhci->ioaddr = ioaddr;
	sdhci->version = SDHCI_SPEC_200;
	sdhci->quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
			SDHCI_QUIRK_SINGLE_POWER_WRITE |
			SDHCI_QUIRK_ENABLE_INTERRUPT_AT_BLOCK_GAP |
			SDHCI_QUIRK_BROKEN_WRITE_PROTECT |
			SDHCI_QUIRK_BROKEN_CTRL_HISPD |
			SDHCI_QUIRK_NO_HISPD_BIT |
			SDHCI_QUIRK_8_BIT_DATA |
			SDHCI_QUIRK_NO_VERSION_REG |
			SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC |
			SDHCI_QUIRK_BROKEN_CARD_DETECTION;

	if (host->cd_gpio != -1) {
		sdhci->quirks |= SDHCI_QUIRK_RUNTIME_DISABLE;
	}

	if (plat->force_hs != 0)
		sdhci->quirks |= SDHCI_QUIRK_FORCE_HIGH_SPEED_MODE;
#ifdef CONFIG_MMC_EMBEDDED_SDIO
	mmc_set_embedded_sdio_data(sdhci->mmc,
			&plat->cis,
			&plat->cccr,
			plat->funcs,
			plat->num_funcs);
#endif
	if (host->card_always_on)
		sdhci->mmc->pm_flags |= MMC_PM_IGNORE_PM_NOTIFY;

	rc = sdhci_add_host(sdhci);
	if (rc)
		goto err_clk_disable;

	platform_set_drvdata(pdev, host);

	if (plat->cd_gpio != -1) {
	rc = gpio_request(host->cd_gpio, "card_detect");
		if (rc < 0) {
			dev_err(&pdev->dev, "request cd gpio failed = %d \n", host->cd_gpio);
			host->cd_gpio = -1;
			goto err_remove_host;
		}
		host->irq_cd = gpio_to_irq(host->cd_gpio);
		if (host->irq_cd < 0) {
			dev_err(&pdev->dev, "invalid card detect GPIO\n");
			host->cd_gpio = -1;
			host->irq_cd = -1;
			goto err_remove_host;
		}
		tegra_gpio_enable(host->cd_gpio);
		rc  = gpio_direction_input(host->cd_gpio);
		if (rc < 0) {
			dev_err(&pdev->dev, "failed to configure GPIO\n");
			gpio_free(host->cd_gpio);
			host->cd_gpio = -1;
			goto err_remove_host;
		}
		rc = request_irq(gpio_to_irq(plat->cd_gpio), carddetect_irq,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			mmc_hostname(sdhci->mmc), sdhci);

		if (rc)
			goto err_remove_host;
		host->card_present = host->card_present_old =
			(gpio_get_value(plat->cd_gpio) == host->cd_gpio_polarity);
		dev_info(&pdev->dev, "host->card_present = %d \n", host->card_present);
	} else if (plat->register_status_notify) {
		plat->register_status_notify(
			tegra_sdhci_status_notify_cb, sdhci);
	}
       if (plat->cd_gpio == -1)
		host->card_present = true;

	if (host->wp_gpio != -1) {
		rc  = gpio_request(host->wp_gpio, "write_protect");
		if (rc  < 0) {
			dev_err(&pdev->dev, "request wp gpio failed = %d \n", host->wp_gpio);
			host->wp_gpio = -1;
			goto err_remove_host;
		}
		tegra_gpio_enable(host->wp_gpio);
		rc  = gpio_direction_input(host->wp_gpio);

		if (rc  < 0) {
			dev_err(&pdev->dev, "configure wp gpio failed\n");
			gpio_free(host->wp_gpio);
			host->wp_gpio = -1;
			goto err_remove_host;
		}
	}

	if (host->power_gpio != -1) {
		rc   = gpio_request(host->power_gpio, "power_gpio");
		if (rc   < 0) {
			dev_err(&pdev->dev, "request power gpio failed = %d \n", host->power_gpio);
			host->power_gpio = -1;
			goto err_remove_host;
		}
		tegra_gpio_enable(host->power_gpio);
		rc   = gpio_direction_output(host->power_gpio, !host->power_gpio_polarity);

		if (rc   < 0) {
			dev_err(&pdev->dev, "configure power gpio failed\n");
			gpio_free(host->power_gpio);
			host->power_gpio = -1;
			goto err_remove_host;
		}
		if (host->card_present) {
			tasklet_schedule(&sdhci->card_tasklet);
		}
	}

	if (plat->board_probe)
		plat->board_probe(pdev->id, sdhci->mmc);

	printk(KERN_INFO "sdhci%d: initialized irq %d ioaddr %p\n", pdev->id,
			sdhci->irq, sdhci->ioaddr);

	return 0;

err_remove_host:
	sdhci_remove_host(sdhci, 1);
err_clk_disable:
	clk_disable(host->clk);
err_clkput:
	clk_put(host->clk);
err_free_host:
	if (sdhci)
		sdhci_free_host(sdhci);
err_unmap:
	iounmap(sdhci->ioaddr);

	return rc;
}

static int tegra_sdhci_remove(struct platform_device *pdev)
{
	struct tegra_sdhci_host *host = platform_get_drvdata(pdev);
	if (host) {
		struct tegra_sdhci_platform_data *plat;
		plat = pdev->dev.platform_data;
		if (plat && plat->board_probe)
			plat->board_probe(pdev->id, host->sdhci->mmc);

		if (host->cd_gpio != -1)
			gpio_free(host->cd_gpio);

		if (host->irq_cd != -1)
			free_irq(host->irq_cd, host->sdhci);

		if (host->wp_gpio != -1)
			gpio_free(host->wp_gpio);

		if (host->power_gpio != -1)
			gpio_free(host->power_gpio);

		sdhci_remove_host(host->sdhci, 0);
		sdhci_free_host(host->sdhci);
	}
	return 0;
}


#define is_card_sdio(_card) \
((_card) && ((_card)->type == MMC_TYPE_SDIO))

#ifdef CONFIG_PM


static void tegra_sdhci_restore_interrupts(struct sdhci_host *sdhost)
{
	u32 ierr;
	u32 clear = SDHCI_INT_ALL_MASK;
	struct tegra_sdhci_host *host = sdhci_priv(sdhost);

	/* enable required interrupts */
	ierr = sdhci_readl(sdhost, SDHCI_INT_ENABLE);
	ierr &= ~clear;
	ierr |= host->sdhci_ints;
	sdhci_writel(sdhost, ierr, SDHCI_INT_ENABLE);
	sdhci_writel(sdhost, ierr, SDHCI_SIGNAL_ENABLE);

	if ((host->sdhci_ints & SDHCI_INT_CARD_INT) &&
		(sdhost->quirks & SDHCI_QUIRK_ENABLE_INTERRUPT_AT_BLOCK_GAP)) {
		u8 gap_ctrl = sdhci_readb(sdhost, SDHCI_BLOCK_GAP_CONTROL);
		gap_ctrl |= 0x8;
		sdhci_writeb(sdhost, gap_ctrl, SDHCI_BLOCK_GAP_CONTROL);
	}
}

static int tegra_sdhci_restore(struct sdhci_host *sdhost)
{
	unsigned long timeout;
	u8 mask = SDHCI_RESET_ALL;

	sdhci_writeb(sdhost, mask, SDHCI_SOFTWARE_RESET);

	sdhost->clock = 0;

	/* Wait max 100 ms */
	timeout = 100;

	/* hw clears the bit when it's done */
	while (sdhci_readb(sdhost, SDHCI_SOFTWARE_RESET) & mask) {
		if (timeout == 0) {
			printk(KERN_ERR "%s: Reset 0x%x never completed.\n",
				mmc_hostname(sdhost->mmc), (int)mask);
			return -EIO;
		}
		timeout--;
		mdelay(1);
	}

	tegra_sdhci_restore_interrupts(sdhost);
	return 0;
}

static int tegra_sdhci_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tegra_sdhci_host *host = platform_get_drvdata(pdev);
	int ret = 0;

	printk(KERN_INFO "%s (%s) ++ \n", __func__, mmc_hostname(host->sdhci->mmc));
	if (time_before(jiffies, host->card_detection_time + msecs_to_jiffies(SD_TIME_GAP_FOR_SUSPEND))) {
		printk(KERN_INFO "%s:  Prevent from going to suspend mode too soon \n", __func__);
		return -1;
	}

	if (host->irq_cd != -1)
		disable_irq(host->irq_cd);

	if (host->card_always_on && is_card_sdio(host->sdhci->mmc->card)) {
		int div = 0;
		u16 clk;
		unsigned int clock = 100000;

		if (device_may_wakeup(&pdev->dev)) {
		        enable_irq_wake(host->sdhci->irq);
		}

		/* save interrupt status before suspending */
		host->sdhci_ints = sdhci_readl(host->sdhci, SDHCI_INT_ENABLE);

		/* reduce host controller clk and card clk to 100 KHz */
		tegra_sdhci_set_clock(host->sdhci, clock);
		sdhci_writew(host->sdhci, 0, SDHCI_CLOCK_CONTROL);

		if (host->sdhci->max_clk > clock) {
			div =  1 << (fls(host->sdhci->max_clk / clock) - 2);
			if (div > 128)
				div = 128;
		}

		clk = div << SDHCI_DIVIDER_SHIFT;
		clk |= SDHCI_CLOCK_INT_EN | SDHCI_CLOCK_CARD_EN;
		sdhci_writew(host->sdhci, clk, SDHCI_CLOCK_CONTROL);

		return ret;
	}

	/*
	* Set disable_delay to zero
	* to disable mmc_schedule_delay_work
	* before suspend
	*/
	mmc_set_disable_delay(host->sdhci->mmc, 0);
	ret = sdhci_suspend_host(host->sdhci, state);
	if (ret)
		pr_err("%s: failed, error = %d\n", __func__, ret);

	tegra_sdhci_enable_clock(host, 0);

	printk(KERN_INFO "%s (%s) -- \n", __func__, mmc_hostname(host->sdhci->mmc));
	return ret;
}

static int tegra_sdhci_resume(struct platform_device *pdev)
{
	struct tegra_sdhci_host *host = platform_get_drvdata(pdev);
	int ret;
	u8 pwr;

	if (host->card_always_on && is_card_sdio(host->sdhci->mmc->card)) {
		int ret = 0;

		if (device_may_wakeup(&pdev->dev)) {
		        disable_irq_wake(host->sdhci->irq);
		}

		/* soft reset SD host controller and enable interrupts */
		ret = tegra_sdhci_restore(host->sdhci);
		if (ret) {
			pr_err("%s: failed, error = %d\n", __func__, ret);
			return ret;
		}

		mmiowb();
		host->sdhci->mmc->ops->set_ios(host->sdhci->mmc,
			&host->sdhci->mmc->ios);
		return 0;
	}

	if (host->cd_gpio != -1) {
		bool card_status_before_suspend =  host->card_present;

		if (host->cd_gpio != -1) {
			host->card_present = host->card_present_old = (gpio_get_value(host->cd_gpio) == host->cd_gpio_polarity);
		}
		pr_info("%s: card_status_before_suspend=%d, card_present=%d \n", __func__, card_status_before_suspend, host->card_present);
		if (card_status_before_suspend != host->card_present) {
			if (!host->card_present) {
#ifdef CONFIG_MMC_BLOCK_DEFERRED_RESUME
				mmc_set_bus_resume_policy(host->sdhci->mmc, 0);
#endif
			}
		}
	}

	tegra_sdhci_enable_clock(host, 1);

	pwr = SDHCI_POWER_ON;
	sdhci_writeb(host->sdhci, pwr, SDHCI_POWER_CONTROL);
	host->sdhci->pwr = 0;
	/*
	* Set disable_delay
	* to enable mmc_schedule_delay_work
	*/
	mmc_set_disable_delay(host->sdhci->mmc, 5);
	ret = sdhci_resume_host(host->sdhci);
	if (ret)
		pr_err("%s: failed, error = %d\n", __func__, ret);

	if (host->irq_cd != -1)
		enable_irq(host->irq_cd);

	return ret;
}
#else
#define tegra_sdhci_suspend    NULL
#define tegra_sdhci_resume     NULL
#endif

static struct platform_driver tegra_sdhci_driver = {
	.probe = tegra_sdhci_probe,
	.remove = tegra_sdhci_remove,
	.suspend = tegra_sdhci_suspend,
	.resume = tegra_sdhci_resume,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init tegra_sdhci_init(void)
{
	return platform_driver_register(&tegra_sdhci_driver);
}

static void __exit tegra_sdhci_exit(void)
{
	platform_driver_unregister(&tegra_sdhci_driver);
}

module_init(tegra_sdhci_init);
module_exit(tegra_sdhci_exit);

MODULE_DESCRIPTION("Tegra SDHCI controller driver");
MODULE_LICENSE("GPL");
