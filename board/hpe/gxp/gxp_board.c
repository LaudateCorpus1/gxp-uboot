// SPDX-License-Identifier: GPL-2.0
/*
 * GXP timer driver
 *
 * (C) Copyright 2019 Hewlett Packard Enterprise Development LP.
 * Author: Gilbert Chen <gilbert.chen@hpe.com>
 */

#include <common.h>
#include <dm.h>
#include <ram.h>
#include <timer.h>
#include <asm/io.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <dm/uclass.h>
#include <console.h>
#ifndef CONFIG_DM_ETH
#include <netdev.h>
#endif
#include <net.h>
#include <i2c_eeprom.h>

#define GXP_USBHC_PHY_PLL_CTRL0 0xc0011000
#define GXP_USBHC_PHY_PLL_CTRL1 0xc0011004
#define GXP_USBHC_PHY_CAL_CTRL 0xc0011008
#define GXP_USBHC_PHY_TX_CHAN_CTRL0 0xc001100C
#define GXP_USBHC_PHY_RX_CHAN_CTRL0 0xc0011014
#define GXP_USBHC_PHY_RX_CHAN_CTRL1 0xc0011018
#define GXP_USBHC_PHY_DIGITAL_CTRL0 0xc001101C
#define GXP_USBHC_PHY_DIGITAL_CTRL1 0xc0011020

DECLARE_GLOBAL_DATA_PTR;

static int usb_phy_init(void)
{
	unsigned int r, w, s, m;
	int i;

	//set PLL lock bypass
	r = readb(GXP_USBHC_PHY_DIGITAL_CTRL0);
	w = r | 0x80;
	writeb((uint8_t)w, GXP_USBHC_PHY_DIGITAL_CTRL0);

	//set PLL setting
	r = readl(GXP_USBHC_PHY_PLL_CTRL0);
	s = 0x00600005;
	m = 0x31ff007f;
	w = (r & ~m) | (m & s);
	writel(w, GXP_USBHC_PHY_PLL_CTRL0);
	r = readl(GXP_USBHC_PHY_PLL_CTRL0);

	//disable FS/LS serial mode low power
	r = readw(GXP_USBHC_PHY_DIGITAL_CTRL1);
	m = 0x1000;
	w = ~m & r;
	writew((uint16_t)w, GXP_USBHC_PHY_DIGITAL_CTRL1);
	r = readw(GXP_USBHC_PHY_DIGITAL_CTRL1);

	//PHY settings for USB 3.0
	r = readw(GXP_USBHC_PHY_CAL_CTRL);
	m = 0x0700;
	s = 0x0600;
	w = (r & ~m) | (m & s);
	writew((uint16_t)w, GXP_USBHC_PHY_CAL_CTRL);
	r = readw(GXP_USBHC_PHY_CAL_CTRL);

	//RX channel control 0
	r = readl(GXP_USBHC_PHY_RX_CHAN_CTRL0);
	m = 0x10008000;
	s = 0x10000000;
	w = (r & ~m) | (m & s);
	writel(w, GXP_USBHC_PHY_RX_CHAN_CTRL0);
	r = readl(GXP_USBHC_PHY_RX_CHAN_CTRL0);

	//RX channel control 1
	r = readb(GXP_USBHC_PHY_RX_CHAN_CTRL1);
	m = 0x0f;
	s = 0x09;
	w = (r & ~m) | (m & s);
	writeb((uint8_t)w, GXP_USBHC_PHY_RX_CHAN_CTRL1);
	r = readb(GXP_USBHC_PHY_RX_CHAN_CTRL1);

	//Power on PLL
	r = readw(GXP_USBHC_PHY_PLL_CTRL1);
	m = 0x100b;
	s = 0x1003;
	w = (r & ~m) | (m & s);
	writew((uint16_t)w, GXP_USBHC_PHY_PLL_CTRL1);
	r = readw(GXP_USBHC_PHY_PLL_CTRL1);

	//TX output driver apmlitude
	r = readb(GXP_USBHC_PHY_TX_CHAN_CTRL0 + 2);
	m = 0x8f;
	s = 0x30;
	w = (r & m) | (~m & s);
	writeb((uint8_t)w, GXP_USBHC_PHY_TX_CHAN_CTRL0 + 2);

	//TX channel control 0, for low speed devices
	r = readl(GXP_USBHC_PHY_TX_CHAN_CTRL0);
	m = 0x000ff000;
	s = 0x00033000;
	w = (r & ~m) | (m & s);
	writel(w, GXP_USBHC_PHY_TX_CHAN_CTRL0);
	r = readl(GXP_USBHC_PHY_TX_CHAN_CTRL0);

	//power on tx/rx channel
	r = readb(GXP_USBHC_PHY_TX_CHAN_CTRL0 + 3);
	m = 0x03;
	w = r | m;
	writeb((uint8_t)w, GXP_USBHC_PHY_TX_CHAN_CTRL0 + 3);

	//check for calibration complete
	r = readl(GXP_USBHC_PHY_RX_CHAN_CTRL1);
	i = 0;
	while (!(r & 0x80000000) && (i < 5))
	{
		i++;
		mdelay(1000);
		r = readl(GXP_USBHC_PHY_RX_CHAN_CTRL1);
	}

	return 0;
}

/*
 * get_eeprom_mac()
 * This function read the MAC from the eeprom, be sure that the
 * chip address is correct.
 */

static int get_eeprom_mac(unsigned char *v_rom_mac, uint8_t nic_index)
{

	struct udevice *dev;
	int ret;
	unsigned offset = 0x84; /* Offset in the eeprom */
	offset = offset + (6 * nic_index);

	ret = uclass_first_device_err(UCLASS_I2C_EEPROM, &dev);
	if (ret)
		return ret;

	ret = i2c_eeprom_read(dev, offset, v_rom_mac, 6);
	if (ret)
	{
		printf("\n%s: ret %d read eeprom failure \n", __func__, ret);
	}

	if (!is_valid_ethaddr(v_rom_mac))
	{
		printf("\nWarning: MAC eeprom %pM is not valid", v_rom_mac);
#ifdef CONFIG_NET_RANDOM_ETHADDR
		net_random_ethaddr(v_rom_mac);
		printf(",using random MAC address - %pM\n", v_rom_mac);
#else
		ret = -1;
#endif
	}

	return ret;
}

int board_init(void)
{

	usb_phy_init();

	return 0;
}

int dram_init(void)
{
	gd->ram_size = CONFIG_SYS_SDRAM_SIZE;

	return 0;
}

int board_eth_init(struct bd_info *bis)
{
	struct eth_device *dev;
	int ret = -EINVAL;

#if defined(CONFIG_GXP_UMAC)
	uint8_t v_mac[6];

	if (!eth_env_get_enetaddr("ethaddr", v_mac))
	{
		/* If the MAC address is not in the environment, get it: */
		if (get_eeprom_mac(v_mac, 0))
		{
			printf("\n*** ERROR: ethaddr is NOT set !!\n");
			return -EINVAL;
		}
		eth_env_set_enetaddr("ethaddr", v_mac);
		printf("MAC Address %pM ", v_mac);
	}

	if (!eth_env_get_enetaddr("eth1addr", v_mac))
	{
		/* If the MAC address is not in the environment, get it: */
		if (get_eeprom_mac(v_mac, 1))
		{
			printf("\n*** ERROR: ethaddr is NOT set !!\n");
			return -EINVAL;
		}
		eth_env_set_enetaddr("eth1addr", v_mac);
	}

	ret = gxp_umac_register(bis);
	if (ret < 1)
	{
		printf("%s: gxp_umac_register() failed. ret = %d\n", __func__, ret);
		return -EINVAL; // failed
	}

	dev = eth_get_dev_by_name("GXP_UMAC0");
	if (!dev)
	{
		printf("%s: Unable to get device entry GXP UMAC0\n", __func__);
		return -EINVAL;
	}
#endif

	return ret;
}
