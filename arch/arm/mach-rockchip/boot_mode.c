// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2016 Rockchip Electronics Co., Ltd
 */

#include <common.h>
#include <adc.h>
#include <asm/io.h>
#include <asm/arch-rockchip/boot_mode.h>

#if (CONFIG_ROCKCHIP_BOOT_MODE_REG == 0)

int setup_boot_mode(void)
{
	return 0;
}

#else

void set_back_to_bootrom_dnl_flag(void)
{
	writel(BOOT_BROM_DOWNLOAD, CONFIG_ROCKCHIP_BOOT_MODE_REG);
}

/*
 * detect download key status by adc, most rockchip
 * based boards use adc sample the download key status,
 * but there are also some use gpio. So it's better to
 * make this a weak function that can be override by
 * some special boards.
 */
#define KEY_DOWN_MIN_VAL	0
#define KEY_DOWN_MAX_VAL	30

__weak int rockchip_dnl_key_pressed(void)
{
	unsigned int val;

	if (adc_channel_single_shot("saradc", 1, &val)) {
		pr_err("%s: adc_channel_single_shot fail!\n", __func__);
		return false;
	}

	if ((val >= KEY_DOWN_MIN_VAL) && (val <= KEY_DOWN_MAX_VAL))
		return true;
	else
		return false;
}

int rockchip_dnl_mode(int num_modes)
{
	int mode = 0;

	while(mode < num_modes) {
		++mode;

		printf("rockchip_dnl_mode = %d mode\n", mode);

		for (int i = 0; i < mode; ++i) {
			cli_simple_run_command("led power off", 0);
			mdelay(100);
			cli_simple_run_command("led power on", 0);
			mdelay(100);
		}

		// wait 2 seconds
		for (int i = 0; i < 100; ++i) {
			if (!rockchip_dnl_key_pressed()) {
				goto end;
			}
			mdelay(20);
		}
	}

end:
	cli_simple_run_command("led power off", 0);
	cli_simple_run_command("led standby on", 0);
	return mode;
}

void rockchip_dnl_mode_check(void)
{
	if (!rockchip_dnl_key_pressed()) {
		return;
	}

	switch(rockchip_dnl_mode(4)) {
	case 0:
		return;

	case 1:
		printf("entering ums mode...\n");
		cli_simple_run_command("ums 0 mmc 0", 0);
		cli_simple_run_command("ums 0 mmc 1", 0);
		break;

	case 2:
		printf("entering fastboot mode...\n");
		cli_simple_run_command("mmc dev 0; fastboot usb 0", 0);
		cli_simple_run_command("mmc dev 1; fastboot usb 0", 0);
		break;

	case 3:
		printf("entering download mode...\n");
		cli_simple_run_command("rockusb 0 mmc 0", 0);
		cli_simple_run_command("rockusb 0 mmc 1", 0);
		break;

	case 4:
		printf("entering maskrom mode...\n");
		break;
	}

	set_back_to_bootrom_dnl_flag();
	do_reset(NULL, 0, 0, NULL);
}

int setup_boot_mode(void)
{
	void *reg = (void *)CONFIG_ROCKCHIP_BOOT_MODE_REG;
	int boot_mode = readl(reg);

	rockchip_dnl_mode_check();

	boot_mode = readl(reg);
	debug("%s: boot mode 0x%08x\n", __func__, boot_mode);

	/* Clear boot mode */
	writel(BOOT_NORMAL, reg);

	switch (boot_mode) {
	case BOOT_NORMAL:
		printf("normal boot\n");
		env_set("boot_mode", "normal");
		break;
	case BOOT_LOADER:
		printf("enter Rockusb!\n");
		env_set("preboot", "setenv preboot; rockusb 0 mmc 0");
		break;
	case BOOT_RECOVERY:
		printf("enter recovery!\n");
		env_set("boot_mode", "recovery");
		break;
	case BOOT_FASTBOOT:
		printf("enter fastboot!\n");
		env_set("preboot", "setenv preboot; fastboot usb0");
		break;
	case BOOT_CHARGING:
		printf("enter charging!\n");
		env_set("boot_mode", "charging");
		break;
	case BOOT_UMS:
		printf("enter fastboot!\n");
		env_set("preboot", "setenv preboot; if mmc dev 0;"
			"then ums mmc 0; else ums mmc 1;fi");
		break;
	default:
		env_set("boot_mode", "unknown");
		break;
	}

	return 0;
}

#endif
