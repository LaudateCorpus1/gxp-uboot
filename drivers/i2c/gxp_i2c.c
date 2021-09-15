// SPDX-License-Identifier: GPL-2.0
/*
 * GXP I2C driver
 *
 * (C) Copyright 2021 Hewlett Packard Enterprise Development LP.
 * Author: Jorge Cisneros <jorge.cisneros@hpe.com>
 */

#include <common.h>
#include <dm.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <i2c.h>
#include <log.h>
#include <clk.h>

/* GXP I2C registers */
#define GXP_I2CSTAT 0x00
#define GXP_I2CEVTERR 0x01
#define GXP_I2CSNPDAT 0x02
#define GXP_I2CMCMD 0x04
#define GXP_I2CMTXDAT 0x05
#define GXP_I2CSCMD 0x06
#define GXP_I2CSTXDAT 0x07
#define GXP_I2CMANCTRL 0x08
#define GXP_I2CSNPAA 0x09
#define GXP_I2COWNADR 0x0B
#define GXP_I2CFREQDIV 0x0C
#define GXP_I2CFLTFAIR 0x0D
#define GXP_I2CTMOEDG 0x0E
#define GXP_I2CCYCTIM 0x0F

#define POLLTIME_US 100000
enum
{
	GXP_I2C_IDLE = 0,
	GXP_I2C_STARTED,
};

struct gxp_i2c_priv
{
	u32 base;
	uint8_t state;
};

static void gxp_i2c_init_bus(struct udevice *dev)
{
	struct gxp_i2c_priv *priv = dev_get_priv(dev);

	priv->base = (u32)dev_read_addr_ptr(dev);
	priv->state = GXP_I2C_IDLE;

	debug("Base  %x \n", priv->base);

	writeb(0x14, priv->base + GXP_I2CFREQDIV); //clock = 100KHz
	writeb(0x61, priv->base + GXP_I2CFLTFAIR); //filter count = 6, fairness count = 1
	writeb(0x0a, priv->base + GXP_I2CTMOEDG);
	writeb(0x00, priv->base + GXP_I2CCYCTIM); //disable maximum cycle timeout

	writeb(0x80, priv->base + GXP_I2CMCMD);	  //clear event
	writeb(0x30, priv->base + GXP_I2CSCMD);	  //mask slave event
	writeb(0x00, priv->base + GXP_I2CEVTERR); //clear event

	writel(0x000000f0, priv->base + GXP_I2CMANCTRL); // reset the engine
	udelay(10);
	writel(0x00000030, priv->base + GXP_I2CMANCTRL);
}

static void gxp_i2c_stop(struct udevice *dev)
{
	struct gxp_i2c_priv *priv = dev_get_priv(dev);

	writeb(0x82, priv->base + GXP_I2CMCMD); // clear event, send stop
	priv->state = GXP_I2C_IDLE;
}

static int gxp_wait_event(struct udevice *dev)
{
	unsigned int timeout;
	unsigned int value;
	struct gxp_i2c_priv *priv = dev_get_priv(dev);

	/* wait for master event*/
	timeout = 0;
	value = readw(priv->base + GXP_I2CSTAT);
	while (!(value & 0x1000))
	{
		udelay(10);
		timeout += 10;
		if (POLLTIME_US < timeout)
		{
			return -1;
		}
		value = readw(priv->base + GXP_I2CSTAT);
	}
	return 0;
}

static int gxp_i2c_read(struct udevice *dev, u8 chip, u8 *buffer, int len)
{
	unsigned int value;
	int i;
	struct gxp_i2c_priv *priv = dev_get_priv(dev);

	/* start + chip + read bit */
	value = chip << 1;
	value = value << 8;
	value |= 0x05;
	if (priv->state != GXP_I2C_IDLE)
	{
		value |= 0x80; //clear event for restart
	}

	writew(value, priv->base + GXP_I2CMCMD);
	priv->state = GXP_I2C_STARTED;

	if (gxp_wait_event(dev))
	{
		// printf("<%s> base %x chip %d addr phase timeout (I2CSTATE = 0x%04x)\n", __func__, priv->base, chip, value);
		return -1;
	}

	/* check ack */
	value = readw(priv->base + GXP_I2CSTAT);
	if (!(value & 0x0008))
	{
		// printf("<%s> bus-%d No ACK for addr phase (STAT=0x%04x)\n", __func__, chip, value);
		// nack
		return -2;
	}

	/* read data */
	for (i = 0; i < len; i++)
	{
		/* clear event, read, ack*/
		value = 0x8C;
		if (len == (i + 1))
		{
			value &= ~0x08; //lgxp byte, clear ack enable bit
		}
		writeb(value, priv->base + GXP_I2CMCMD);

		if (gxp_wait_event(dev))
		{
			// printf("<%s> bus %d data phase timeout (STAT = 0x%04x)\n", __func__, chip, value);
			return -1;
		}

		/* get the data returned */
		buffer[i] = readb(priv->base + GXP_I2CSNPDAT);
	}
	return 0;
}

static int gxp_i2c_write_addr(struct udevice *dev, uchar chip)
{
	unsigned int value;
	struct gxp_i2c_priv *priv = dev_get_priv(dev);

	/* start + chip(7 bits) + write bit */
	value = chip << 1;
	value = value << 8;
	value |= 0x01;
	if (priv->state != GXP_I2C_IDLE)
	{
		value |= 0x80; //clear event for restart
	}
	writew(value, priv->base + GXP_I2CMCMD);
	priv->state = GXP_I2C_STARTED;

	if (gxp_wait_event(dev))
	{
		// printf("<%s> bus %d base %x addr phase timeout (I2CSTATE = 0x%04x)\n", __func__, chip, priv->base, value);
		return -1;
	}

	/* check ack */
	value = readw(priv->base + GXP_I2CSTAT);
	if (0x0 == (value & 0x0008))
	{
		// printf("<%s> bus %d No ACK for address phase (STAT=0x%04x)\n", __func__, chip, value);
		// nack
		return -2;
	}

	return 0;
}

static int gxp_i2c_write(struct udevice *dev, uchar *buffer, int len)
{
	unsigned int value;
	int i;
	struct gxp_i2c_priv *priv = dev_get_priv(dev);

	/* write data */
	for (i = 0; i < len; i++)
	{
		//printf("<%s> bus %d write byte: B%d=0x%02X\n", __func__, current_bus, i, buffer[i]);
		value = buffer[i];
		value = value << 8;
		value |= 0x80;
		writew(value, priv->base + GXP_I2CMCMD);

		if (gxp_wait_event(dev))
		{
			//printf("<%s> bus %d data phase timeout (I2CEVTERR = 0x%04x)\n", __func__, current_bus, value);
			return -1;
		}

		/* check ack */
		value = readw(priv->base + GXP_I2CSTAT);
		if (0x0 == (value & 0x08))
		{
			//printf("<%s> bus %d No ACK for data phase (STAT=0x%04x)\n", __func__, current_bus, value);
			//nack
			return -2;
		}
	}

	return 0;
}

static int gxp_i2c_read_data(struct udevice *dev, u8 chip_addr, u8 *buffer,
							 size_t len, bool send_stop)
{
	int ret;
	ret = gxp_i2c_read(dev, chip_addr, buffer, len);
	if (send_stop)
		gxp_i2c_stop(dev);
	return ret;
}

static int gxp_i2c_write_data(struct udevice *dev, u8 chip_addr, u8 *buffer, size_t len, bool send_stop)
{
	int ret;

	ret = gxp_i2c_write_addr(dev, chip_addr);
	if (ret < 0)
	{
		gxp_i2c_stop(dev);
		return ret;
	}

	ret = gxp_i2c_write(dev, buffer, len);
	if (send_stop)
		gxp_i2c_stop(dev);
	return ret;
}

static int gxp_i2c_set_speed(struct udevice *dev, unsigned int speed)
{

	return 0;
}

static int gxp_i2c_xfer(struct udevice *dev, struct i2c_msg *msg, int nmsgs)
{
	int ret;
	for (; nmsgs > 0; nmsgs--, msg++)
	{
		if (msg->flags & I2C_M_RD)
		{
			ret = gxp_i2c_read_data(dev, msg->addr, msg->buf,
									msg->len, (nmsgs == 1));
		}
		else
		{
			ret = gxp_i2c_write_data(dev, msg->addr, msg->buf,
									 msg->len, (nmsgs == 1));
		}
		if (ret)
		{
			return -EREMOTEIO;
		}
	}

	return 0;
}

static int gxp_i2c_probe(struct udevice *dev)
{
	debug("Enabling I2C%u\n", dev->seq);
	gxp_i2c_init_bus(dev);
	return 0;
}

static int gxp_i2c_probe_chip(struct udevice *dev, uint chip_addr,
							  uint chip_flags)
{

	u8 dummy;
	debug("Probing chip %d \n", chip_addr);

	return gxp_i2c_read_data(dev, chip_addr, &dummy, 1, true);
}

static const struct dm_i2c_ops gxp_i2c_ops = {
	.xfer = gxp_i2c_xfer,
	.set_bus_speed = gxp_i2c_set_speed,
	.probe_chip = gxp_i2c_probe_chip,
};

static const struct udevice_id gxp_i2c_ids[] = {
	{.compatible = "hpe,gxp-i2c"},
	{.compatible = "hpe,gxp-i2c"},
	{},
};

U_BOOT_DRIVER(gxp_i2c) = {
	.name = "gxp_i2c",
	.id = UCLASS_I2C,
	.of_match = gxp_i2c_ids,
	.probe = gxp_i2c_probe,
	.ops = &gxp_i2c_ops,
	.priv_auto_alloc_size = sizeof(struct gxp_i2c_priv),
};
