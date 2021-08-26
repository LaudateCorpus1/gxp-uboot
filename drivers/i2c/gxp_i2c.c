// SPDX-License-Identifier: GPL-2.0
/*
 * GXP I2C driver
 *
 * (C) Copyright 2021 Hewlett Packard Enterprise Development LP.
 * Author: Gilbert Chen <gilbert.chen@hpe.com>
 */


#include <common.h>
#include <asm/io.h>
#include <linux/delay.h>

#define GXP_IOP_BASE	0xC0000000

#define GXP_I2C0_BASE		(GXP_IOP_BASE + 0x2000)
#define GXP_I2C1_BASE		(GXP_IOP_BASE + 0x2100)
#define GXP_I2C2_BASE		(GXP_IOP_BASE + 0x2200)
#define GXP_I2C3_BASE		(GXP_IOP_BASE + 0x2300)
#define GXP_I2C4_BASE		(GXP_IOP_BASE + 0x2400)
#define GXP_I2C5_BASE		(GXP_IOP_BASE + 0x2500)
#define GXP_I2C6_BASE		(GXP_IOP_BASE + 0x2600)
#define GXP_I2C7_BASE		(GXP_IOP_BASE + 0x2700)
#define GXP_I2C8_BASE		(GXP_IOP_BASE + 0x2800)
#define GXP_I2C9_BASE		(GXP_IOP_BASE + 0x2900)

/* GXP I2C registers */
#define GXP_I2CSTAT	0x00
#define GXP_I2CEVTERR	0x01
#define GXP_I2CSNPDAT	0x02
#define GXP_I2CMCMD	0x04
#define GXP_I2CMTXDAT	0x05
#define GXP_I2CSCMD	0x06
#define GXP_I2CSTXDAT	0x07
#define GXP_I2CMANCTRL	0x08
#define GXP_I2CSNPAA	0x09
#define GXP_I2COWNADR	0x0B
#define GXP_I2CFREQDIV	0x0C
#define GXP_I2CFLTFAIR	0x0D
#define GXP_I2CTMOEDG	0x0E
#define GXP_I2CCYCTIM	0x0F

#define GXP_I2C_BUS_NUM	9
#define POLLTIME_US	100000
enum {
	GXP_I2C_IDLE = 0,
	GXP_I2C_STARTED,
};

static u32 i2c_base[GXP_I2C_BUS_NUM] = {
	GXP_I2C0_BASE, GXP_I2C1_BASE, GXP_I2C2_BASE,
	GXP_I2C3_BASE, GXP_I2C4_BASE, GXP_I2C5_BASE,
	GXP_I2C6_BASE, GXP_I2C7_BASE, GXP_I2C8_BASE};

static unsigned int bus_initialized[GXP_I2C_BUS_NUM];
static unsigned int bus_state[GXP_I2C_BUS_NUM];
static unsigned int current_bus;

static void i2c_bus_init(unsigned int bus)
{
	u32 base = i2c_base[bus];

	bus_state[bus] = GXP_I2C_IDLE;
	bus_initialized[bus] = 1;

	writeb(0x14, base + GXP_I2CFREQDIV);	//clock = 100KHz
	writeb(0x61, base + GXP_I2CFLTFAIR);	//filter count = 6, fairness count = 1
	writeb(0x0a, base + GXP_I2CTMOEDG);
	writeb(0x00, base + GXP_I2CCYCTIM);	//disable maximum cycle timeout

	writeb(0x80, base + GXP_I2CMCMD);	//clear event
	writeb(0x30, base + GXP_I2CSCMD);	//mask slave event
	writeb(0x00, base + GXP_I2CEVTERR);	//clear event

	writel(0x000000f0, base + GXP_I2CMANCTRL); // reset the engine
	udelay(10);
	writel(0x00000030, base + GXP_I2CMANCTRL);
}

static void gxp_i2c_stop(void)
{
	u32 base;

	base = i2c_base[current_bus];
	writeb(0x82, base + GXP_I2CMCMD); // clear event, send stop
	bus_state[current_bus] = GXP_I2C_IDLE;
}

static int gxp_wait_event(void)
{
	unsigned int timeout;
	unsigned int value;
	u32 base;

	base = i2c_base[current_bus];
	/* wait for master event*/
	timeout = 0;
	value = readw(base + GXP_I2CSTAT);
	while(!(value & 0x1000)) {
		udelay(10);
		timeout += 10;
		if(POLLTIME_US < timeout) {
			return -1;
		}
		value = readw(base + GXP_I2CSTAT);
	}
	return 0;
}

static int gxp_i2c_read(uchar chip, uchar *buffer, int len)
{
	unsigned int value;
	int i;
	u32 base;

	base = i2c_base[current_bus];

	/* start + chip + read bit */
	value = chip << 1;
	value = value << 8;
	value |= 0x05;
	if(bus_state[current_bus] != GXP_I2C_IDLE) {
		value |= 0x80;	//clear event for restart
	}
	writew(value, base + GXP_I2CMCMD);
	bus_state[current_bus] = GXP_I2C_STARTED;

	if(gxp_wait_event()) {
		//printf("<%s> bus %d addr phase timeout (I2CSTATE = 0x%04x)\n", __func__, current_bus, value);
		return -1;
	}

	/* check ack */
	value = readw(base + GXP_I2CSTAT);
	if(!(value & 0x0008)) {
		//printf("<%s> bus-%d No ACK for addr phase (STAT=0x%04x)\n", __func__, current_bus, value);
		//nack
		return -2;
	}

	/* read data */
	for (i = 0; i < len; i++) {
		/* clear event, read, ack*/
		value = 0x8C;
		if(len == (i+1)) {
			value &= ~0x08; //last byte, clear ack enable bit
		}
		writeb(value, base + GXP_I2CMCMD);

		if(gxp_wait_event()) {
			//printf("<%s> bus %d data phase timeout (STAT = 0x%04x)\n", __func__, current_bus, value);
			return -1;
		}

		/* get the data returned */
		buffer[i] = readb(base + GXP_I2CSNPDAT);
		
		//printf("<%s> bus %d read byte: B%d=0x%02X\n",	__func__, current_bus, i, buffer[i]);
	}
	return 0;
}

static int gxp_i2c_write_addr(uchar chip)
{
	unsigned int value;
	u32 base;

	base = i2c_base[current_bus];

	/* start + chip(7 bits) + write bit */
	value = chip << 1;
	value = value << 8;
	value |= 0x01;
	if(bus_state[current_bus] != GXP_I2C_IDLE) {
		value |= 0x80;	//clear event for restart
	}
	writew(value, base + GXP_I2CMCMD);
	bus_state[current_bus] = GXP_I2C_STARTED;

	if(gxp_wait_event()) {
		//printf("<%s> bus %d addr phase timeout (I2CSTATE = 0x%04x)\n", __func__, current_bus, value);
		return -1;
	}

	/* check ack */
	value = readw(base + GXP_I2CSTAT);
	if(0x0 == (value & 0x0008)) {
		//printf("<%s> bus %d No ACK for address phase (STAT=0x%04x)\n", __func__, current_bus, value);
		//nack
		return -2;
	}

	return 0;
}

static int gxp_i2c_write_data(uchar *buffer, int len)
{
	unsigned int value;
	int i;
	u32 base;

	base = i2c_base[current_bus];

	/* write data */
	for (i = 0; i < len; i++) {
		//printf("<%s> bus %d write byte: B%d=0x%02X\n", __func__, current_bus, i, buffer[i]);
		value = buffer[i];
		value = value << 8;
		value |= 0x80;
		writew(value, base + GXP_I2CMCMD);

		if(gxp_wait_event()) {
			//printf("<%s> bus %d data phase timeout (I2CEVTERR = 0x%04x)\n", __func__, current_bus, value);
			return -1;
		}

		/* check ack */
		value = readw(base + GXP_I2CSTAT);
		if(0x0 == (value & 0x08)) {
			//printf("<%s> bus %d No ACK for data phase (STAT=0x%04x)\n", __func__, current_bus, value);
			//nack
			return -2;
		}
	}

	return 0;
}

static int gxp_i2c_write(uchar chip, uchar *buffer, int len)
{
	int ret;

	ret = gxp_i2c_write_addr(chip);
	if(ret < 0) {
		return ret;
	}

	return gxp_i2c_write_data(buffer, len);
}

//APIs

int i2c_set_bus_num(unsigned int bus)
{
	if (bus >= GXP_I2C_BUS_NUM) {
		printf("Bad bus: %d\n", bus);
		return -1;
	}

	current_bus = bus;

	if (!bus_initialized[current_bus]) {
		i2c_bus_init(bus);
	}

	return 0;
}

unsigned int i2c_get_bus_num(void)
{
	return current_bus;
}

void i2c_init(int speed, int slaveaddr)
{
	current_bus = 0;
	int i;

	for(i=0;i<GXP_I2C_BUS_NUM;i++) {
		bus_initialized[current_bus] = 0;
	}

	i2c_set_bus_num(current_bus);
}

/*
 * i2c_read: - Read multiple bytes from an i2c device
 *
 * The higher level routines take into account that this function is only
 * called with len < page length of the device (see configuration file)
 *
 * @chip:	address of the chip which is to be read
 * @addr:	i2c data address within the chip
 * @alen:	length of the i2c data address (1..2 bytes)
 * @buffer:	where to write the data
 * @len:	how much byte do we want to read
 * @return:	0 in case of success
 */
int i2c_read(uchar chip, uint addr, int alen, uchar *buffer, int len)
{
	u8 addr_bytes[3];
	int ret;

	if(alen > 0) {
		addr_bytes[0] = (u8)((addr >>  0) & 0x000000FF);
		addr_bytes[1] = (u8)((addr >>  8) & 0x000000FF);
		addr_bytes[2] = (u8)((addr >> 16) & 0x000000FF);
		ret = gxp_i2c_write(chip, addr_bytes, alen);
		if(ret < 0) {
			gxp_i2c_stop();
			return ret;
		}
	}

	ret = gxp_i2c_read(chip, buffer, len);
	gxp_i2c_stop();
	return ret;
}

/*
 * i2c_probe: - Test if a chip answers for a given i2c address
 *
 * @chip:	address of the chip which is searched for
 * @return:	0 if a chip was found, -1 otherwhise
 */
int i2c_probe(uchar chip)
{
	u8 dummy;
	return i2c_read(chip, 0, 0, &dummy, 1);
}

/*
 * i2c_write: -  Write multiple bytes to an i2c device
 *
 * The higher level routines take into account that this function is only
 * called with len < page length of the device (see configuration file)
 *
 * @chip:	address of the chip which is to be written
 * @addr:	i2c data address within the chip
 * @alen:	length of the i2c data address (1..2 bytes)
 * @buffer:	where to find the data to be written
 * @len:	how much byte do we want to read
 * @return:	0 in case of success
 */
int i2c_write(uchar chip, uint addr, int alen, uchar *buffer, int len)
{
	u8 addr_bytes[3];
	int ret;

	ret = gxp_i2c_write_addr(chip);
	if(ret < 0) {
		gxp_i2c_stop();
		return ret;
	}

	if(alen > 0) {
		addr_bytes[0] = (u8)((addr >>  0) & 0x000000FF);
		addr_bytes[1] = (u8)((addr >>  8) & 0x000000FF);
		addr_bytes[2] = (u8)((addr >> 16) & 0x000000FF);
		ret = gxp_i2c_write_data(addr_bytes, alen);
		if(ret < 0) {
			gxp_i2c_stop();
			return ret;
		}
	}

	ret = gxp_i2c_write_data(buffer, len);
	gxp_i2c_stop();
	return ret;
}
