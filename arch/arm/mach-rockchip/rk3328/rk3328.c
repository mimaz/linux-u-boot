// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2016 Rockchip Electronics Co., Ltd
 */

#include <common.h>
#include <asm/arch-rockchip/bootrom.h>
#include <asm/arch-rockchip/hardware.h>
#include <asm/arch-rockchip/grf_rk3328.h>
#include <asm/arch-rockchip/uart.h>
#include <asm/arch/boot_mode.h>
#include <asm/arch/clock.h>
#include <asm/arch/grf_rk3328.h>
#include <asm/armv8/mmu.h>
#include <asm/io.h>
#include <dm.h>
#include <dwc3-uboot.h>
#include <power/regulator.h>
#include <usb.h>
#include <syscon.h>
#include <misc.h>
#include <u-boot/sha256.h>

DECLARE_GLOBAL_DATA_PTR;

#define CRU_BASE		0xFF440000
#define GRF_BASE		0xFF100000
#define UART2_BASE		0xFF130000
#define FW_DDR_CON_REG		0xFF7C0040

const char * const boot_devices[BROM_LAST_BOOTSOURCE + 1] = {
	[BROM_BOOTSOURCE_EMMC] = "rksdmmc@ff520000",
	[BROM_BOOTSOURCE_SD] = "rksdmmc@ff500000",
};

#define RK3328_CPUID_OFF  0x7
#define RK3328_CPUID_LEN  0x10

static struct mm_region rk3328_mem_map[] = {
	{
		.virt = 0x0UL,
		.phys = 0x0UL,
		.size = 0xff000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			 PTE_BLOCK_INNER_SHARE
	}, {
		.virt = 0xff000000UL,
		.phys = 0xff000000UL,
		.size = 0x1000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
			 PTE_BLOCK_NON_SHARE |
			 PTE_BLOCK_PXN | PTE_BLOCK_UXN
	}, {
		/* List terminator */
		0,
	}
};

struct mm_region *mem_map = rk3328_mem_map;

int arch_cpu_init(void)
{
#ifdef CONFIG_SPL_BUILD
	/* We do some SoC one time setting here. */

	/* Disable the ddr secure region setting to make it non-secure */
	rk_setreg(FW_DDR_CON_REG, 0x200);
#endif
	return 0;
}

void board_debug_uart_init(void)
{
	struct rk3328_grf_regs * const grf = (void *)GRF_BASE;
	struct rk_uart * const uart = (void *)UART2_BASE;
	enum{
		GPIO2A0_SEL_SHIFT       = 0,
		GPIO2A0_SEL_MASK        = 3 << GPIO2A0_SEL_SHIFT,
		GPIO2A0_UART2_TX_M1     = 1,

		GPIO2A1_SEL_SHIFT       = 2,
		GPIO2A1_SEL_MASK        = 3 << GPIO2A1_SEL_SHIFT,
		GPIO2A1_UART2_RX_M1     = 1,
	};
	enum {
		IOMUX_SEL_UART2_SHIFT   = 0,
		IOMUX_SEL_UART2_MASK    = 3 << IOMUX_SEL_UART2_SHIFT,
		IOMUX_SEL_UART2_M0      = 0,
		IOMUX_SEL_UART2_M1,
	};

	/* uart_sel_clk default select 24MHz */
	writel((3 << (8 + 16)) | (2 << 8), CRU_BASE + 0x148);

	/* init uart baud rate 1500000 */
	writel(0x83, &uart->lcr);
	writel(0x1, &uart->rbr);
	writel(0x3, &uart->lcr);

	/* Enable early UART2 */
	rk_clrsetreg(&grf->com_iomux,
		     IOMUX_SEL_UART2_MASK,
		     IOMUX_SEL_UART2_M1 << IOMUX_SEL_UART2_SHIFT);
	rk_clrsetreg(&grf->gpio2a_iomux,
		     GPIO2A0_SEL_MASK,
		     GPIO2A0_UART2_TX_M1 << GPIO2A0_SEL_SHIFT);
	rk_clrsetreg(&grf->gpio2a_iomux,
		     GPIO2A1_SEL_MASK,
		     GPIO2A1_UART2_RX_M1 << GPIO2A1_SEL_SHIFT);

	/* enable FIFO */
	writel(0x1, &uart->sfe);
}

static void boot_mode_set(int boot_mode)
{
	struct rk3328_grf_regs *grf = syscon_get_first_range(ROCKCHIP_SYSCON_GRF);
	writel(boot_mode, &grf->os_reg[0]);
}

#if defined(CONFIG_USB_FUNCTION_FASTBOOT)
int fb_set_reboot_flag(void)
{
	printf("Setting reboot to fastboot flag ...\n");
	boot_mode_set(BOOT_FASTBOOT);
	return 0;
}
#endif

__weak int rk_board_late_init(void)
{
	return 0;
}

int board_late_init(void)
{
	setup_boot_mode();
	return rk_board_late_init();
}

int board_init(void)
{
	int ret;

	ret = regulators_enable_boot_on(false);
	if (ret)
		debug("%s: Cannot enable boot on regulator\n", __func__);

	return ret;
}

#if defined(CONFIG_USB_GADGET) && defined(CONFIG_USB_GADGET_DWC2_OTG)
#include <usb.h>
#include <usb/dwc2_udc.h>

static struct dwc2_plat_otg_data rk3328_otg_data = {
	.rx_fifo_sz	= 512,
	.np_tx_fifo_sz	= 16,
	.tx_fifo_sz	= 128,
};

int board_usb_init(int index, enum usb_init_type init)
{
	int node;
	const char *mode;
	bool matched = false;
	const void *blob = gd->fdt_blob;

	/* find the usb_otg node */
	node = fdt_node_offset_by_compatible(blob, -1,
					"rockchip,rk3328-usb");

	while (node > 0) {
		mode = fdt_getprop(blob, node, "dr_mode", NULL);
		if (mode && strcmp(mode, "otg") == 0) {
			matched = true;
			break;
		}

		node = fdt_node_offset_by_compatible(blob, node,
					"rockchip,rk3328-usb");
	}
	if (!matched) {
		debug("Not found usb_otg device\n");
		return -ENODEV;
	}

	rk3328_otg_data.regs_otg = fdtdec_get_addr(blob, node, "reg");

	return dwc2_udc_probe(&rk3328_otg_data);
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	return 0;
}
#endif

#ifdef CONFIG_MISC_INIT_R
static void setup_macaddr(void)
{
#if CONFIG_IS_ENABLED(CMD_NET)
	int ret;
	const char *cpuid = env_get("cpuid#");
	u8 hash[SHA256_SUM_LEN];
	int size = sizeof(hash);
	u8 mac_addr[6];

	/* Only generate a MAC address, if none is set in the environment */
	if (env_get("ethaddr"))
		return;

	if (!cpuid) {
		debug("%s: could not retrieve 'cpuid#'\n", __func__);
		return;
	}

	ret = hash_block("sha256", (void *)cpuid, strlen(cpuid), hash, &size);
	if (ret) {
		debug("%s: failed to calculate SHA256\n", __func__);
		return;
	}

	/* Copy 6 bytes of the hash to base the MAC address on */
	memcpy(mac_addr, hash, 6);

	/* Make this a valid MAC address and set it */
	mac_addr[0] &= 0xfe;  /* clear multicast bit */
	mac_addr[0] |= 0x02;  /* set local assignment bit (IEEE802) */
	eth_env_set_enetaddr("ethaddr", mac_addr);

	/* Make a valid MAC address for eth1 */
	mac_addr[5] += 0x20;
	mac_addr[5] &= 0xff;
	eth_env_set_enetaddr("eth1addr", mac_addr);
#endif

	return;
}

static void setup_serial(void)
{
#if CONFIG_IS_ENABLED(ROCKCHIP_EFUSE)
	struct udevice *dev;
	int ret, i;
	u8 cpuid[RK3328_CPUID_LEN];
	u8 low[RK3328_CPUID_LEN/2], high[RK3328_CPUID_LEN/2];
	char cpuid_str[RK3328_CPUID_LEN * 2 + 1];
	u64 serialno;
	char serialno_str[16];

	/* retrieve the device */
	ret = uclass_get_device_by_driver(UCLASS_MISC,
					  DM_GET_DRIVER(rockchip_efuse), &dev);
	if (ret) {
		printf("%s: could not find efuse device\n", __func__);
		return;
	}

	/* read the cpu_id range from the efuses */
	ret = misc_read(dev, RK3328_CPUID_OFF, &cpuid, sizeof(cpuid));
	if (ret) {
		printf("%s: reading cpuid from the efuses failed: %d\n",
		      __func__, ret);
		return;
	}

	memset(cpuid_str, 0, sizeof(cpuid_str));
	for (i = 0; i < 16; i++)
		sprintf(&cpuid_str[i * 2], "%02x", cpuid[i]);

	debug("cpuid: %s\n", cpuid_str);

	/*
	 * Mix the cpuid bytes using the same rules as in
	 *   ${linux}/drivers/soc/rockchip/rockchip-cpuinfo.c
	 */
	for (i = 0; i < 8; i++) {
		low[i] = cpuid[1 + (i << 1)];
		high[i] = cpuid[i << 1];
	}

	serialno = crc32_no_comp(0, low, 8);
	serialno |= (u64)crc32_no_comp(serialno, high, 8) << 32;
	snprintf(serialno_str, sizeof(serialno_str), "%llx", serialno);

	env_set("cpuid#", cpuid_str);
	env_set("serial#", serialno_str);
#endif

	return;
}

int misc_init_r(void)
{
	setup_serial();
	setup_macaddr();

	return 0;
}
#endif

#ifdef CONFIG_SERIAL_TAG
void get_board_serial(struct tag_serialnr *serialnr)
{
	char *serial_string;
	u64 serial = 0;

	serial_string = env_get("serial#");

	if (serial_string)
		serial = simple_strtoull(serial_string, NULL, 16);

	serialnr->high = (u32)(serial >> 32);
	serialnr->low = (u32)(serial & 0xffffffff);
}
#endif
