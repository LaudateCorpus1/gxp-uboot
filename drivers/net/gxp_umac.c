// SPDX-License-Identifier: GPL-2.0
/*
 * GXP UMAC driver
 *
 * (C) Copyright 2020 Hewlett Packard Enterprise Development LP.
 *
*/

#include <common.h>
#include <malloc.h>
#include <net.h>
#ifndef CONFIG_DM_ETH
#include <netdev.h>
#endif
#include <miiphy.h>
#include <gxp_umac.h>
#include <gxp_mdio.h>
#include <asm/io.h>
#include <linux/delay.h>

DECLARE_GLOBAL_DATA_PTR;

#define PAGE_SIZE 4096

static void dcache_inv_lines(unsigned long start, unsigned long size)
{
	unsigned long end;

	end = start + size;

	// align start to beginning of a cache line
	start &= ~0x1f;

	// align end to beginning of cache line after the end
	end |= 0x1f;
	end +=1;

	for(; start < end; start += 32) {
		asm volatile("mcr p15, 0, %0, c7, c14, 1" : : "r" (start));
	}
}

static int is_tx_ring_full(struct umac_ring *txring)
{
	// full if next entry is owned by hardware or dma address is not null (waiting for completion)
	umac_ring_entry *next_entry = txring->producer_ptr;
	dcache_inv_lines((unsigned long)(next_entry), (unsigned long)sizeof(*next_entry));

	return ((txring->producer_ptr->common.status & UMAC_RING_ENTRY_HW_OWN) != 0);
}

umac_ring_entry *next_ring_entry(struct umac_ring *ring, umac_ring_entry *current_entry)
{
	current_entry++;
	if ((uint8_t *)current_entry >= (((uint8_t *)ring->ring_base) + ring->ring_size_bytes)) {
		current_entry = ring->ring_base;
	}
	return current_entry;
}

static int gxp_output(struct eth_device *dev, void *eth_data,
	    int data_length)
{
	struct gxp_umac_private *priv;
	struct umac_struct *umac;
	struct gxp_umac_regs *umac_regs;
	umac_ring_entry *next_entry;

	priv = (struct gxp_umac_private *)dev->priv;
	umac = &priv->umac;
	umac_regs = (struct gxp_umac_regs *)dev->iobase;

	if (is_tx_ring_full(&umac->txring)) {
		printf("tx ring is full and this condition isn't handled trying just checking over and over!\n");
		while(1);
	}

	next_entry = umac->txring.producer_ptr;

	umac->txring.producer_ptr = next_ring_entry(&umac->txring, umac->txring.producer_ptr);

	if (data_length < UMAC_MIN_FRAME_SIZE) {
		data_length = UMAC_MIN_FRAME_SIZE;
	}

	next_entry->tx.dma_address = (uint32_t)eth_data;
	next_entry->tx.count = (int16_t)data_length;
	next_entry->tx.cksum_offset = 0;
	next_entry->tx.status = UMAC_RING_ENTRY_HW_OWN;

	dcache_inv_lines((unsigned long)(eth_data), (unsigned long)data_length);
	dcache_inv_lines((unsigned long)(next_entry), (unsigned long)sizeof(*next_entry));

	io_write4(&umac_regs->ring_prompt, 0);	// trigger it
	while(!(io_read4(&umac_regs->interrupt) & 1)) {
		dcache_inv_lines((unsigned long)(next_entry), (unsigned long)sizeof(*next_entry));
	}

	// clear tx interrupt
	io_write4(&umac_regs->interrupt, 1);
	while(next_entry->tx.status & UMAC_RING_ENTRY_HW_OWN) {
		dcache_inv_lines((unsigned long)(next_entry), (unsigned long)sizeof(*next_entry));
	}

	return 0;
}

static void gxp_set_channel_enable(struct eth_device *dev, int enable)
{
	struct gxp_umac_regs *umac_regs;
	uint32_t tmp;

	umac_regs = (struct gxp_umac_regs *)dev->iobase;

	if (enable) {
		tmp = io_read4(&umac_regs->config_status);
		tmp |= UMAC_CFG_STAT_1_REG_TXEN_MASK | UMAC_CFG_STAT_1_REG_RXEN_MASK;
		io_write4(&umac_regs->config_status, tmp);
		io_write4(&umac_regs->ring_prompt, 0);
	}
	else {
		io_write4(&umac_regs->config_status, 0);
	}
}

// malloc on 16 byte boundary
static void *malloc16(size_t len)
{
	uint32_t base;

	// malloc an extra 16 bytes so that the base can be aligned to a 16 byte boundary
	len += 16;

	base = (uint32_t)malloc(len);
	if (NULL == (void *)base) {
		printf("%s: malloc of %d bytes failed\n", __FUNCTION__, len);
		return(0);
	}

	// align base on 16 byte boundary
	if ((base & 0xf) != 0) {
		base = (base | 0xf) + 1;
	}

	return((void *)base);
}

void gxp_umac_reinitialize_data_structures(struct eth_device *dev)
{
	struct gxp_umac_private *priv;
	struct umac_struct *umac;
	uint32_t i;

	priv = (struct gxp_umac_private *)dev->priv;
	umac = &priv->umac;

	memset(umac->txring.ring_base, 0, umac->txring.ring_size_bytes);

 	memset(umac->rxring.ring_base, 0, umac->rxring.ring_size_bytes);

	memset(umac->rx_packet_buf, 0, umac->num_rx_packet_buf_bytes);

	// initialize pointers to base of ring
	umac->txring.producer_ptr = umac->txring.completion_ptr = umac->txring.ring_base;
	umac->rxring.producer_ptr = umac->rxring.completion_ptr = umac->rxring.ring_base;

	// initialize rx ring entries with rx packet buffer addresses and enable hardware ownership
	for(i = 0; i < UMAC_MAX_RING_ENTRIES; i++) {
		umac->rxring.ring_base[i].rx.dma_address = i * sizeof(umac->rx_packet_buf[0]) + (uint32_t)umac->rx_packet_map_base;
		umac->rxring.ring_base[i].rx.count = UMAC_MAX_FRAME_SIZE;
		umac->rxring.ring_base[i].rx.status = UMAC_RING_ENTRY_HW_OWN;
	}

	dcache_inv_lines((unsigned long)umac->rxring.ring_base, (unsigned long)umac->txring.ring_size_bytes);
	dcache_inv_lines((unsigned long)umac->txring.ring_base, (unsigned long)umac->rxring.ring_size_bytes);
	dcache_inv_lines((unsigned long)umac->rx_packet_buf, (unsigned long)umac->num_rx_packet_buf_bytes);
	return;
}

uint32_t gxp_umac_initialize_data_structures(struct eth_device *dev)
{
	struct gxp_umac_private *priv;
	struct umac_struct *umac;

	priv = (struct gxp_umac_private *)dev->priv;
	umac = &priv->umac;

	umac->txring.ring_size_bytes = UMAC_MAX_RING_ENTRIES*sizeof(umac_ring_entry);
	umac->txring.ring_base = (umac_ring_entry *)malloc16(umac->txring.ring_size_bytes);
	if (NULL == umac->txring.ring_base) {
		printf("%s: malloc of txring %d failed\n", __FUNCTION__, umac->txring.ring_size_bytes);
		return(0);
	}

	umac->rxring.ring_size_bytes = umac->txring.ring_size_bytes;
	umac->rxring.ring_base = (umac_ring_entry *)malloc16(umac->rxring.ring_size_bytes);
	if (NULL == umac->rxring.ring_base) {
		printf("%s: malloc of rxring %d bytes failed\n", __FUNCTION__, umac->rxring.ring_size_bytes);
		return(0);
	}

	umac->num_rx_packet_buf_bytes = UMAC_MAX_RING_ENTRIES*sizeof(struct umac_packet);
	umac->rx_packet_buf = (struct umac_packet *)malloc16(umac->num_rx_packet_buf_bytes);
	if (NULL == umac->rx_packet_buf) {
		printf("%s: malloc of rx_packet_buf %d failed\n", __FUNCTION__, umac->num_rx_packet_buf_bytes);
		return(0);
	}
	umac->rx_packet_map_base = umac->rx_packet_buf;
	umac->rx_packet_map_size = UMAC_MAX_RING_ENTRIES * sizeof(umac->rx_packet_buf[0]);

	gxp_umac_reinitialize_data_structures(dev);

	return(1);
}

static uint32_t get_ring_size_value(uint32_t num_entries)
{
	switch(num_entries) {
	case 4:
		return 0;
	case 8:
		return 1;
	case 16:
		return 3;
	case 32:
		return 7;
	case 64:
		return 0xf;
	case 128:
		return 0x1f;
	case 256:
		return 0x3f;
	default:
		printf("get_ring_size_value: unknown value for num_entries\n");
		while(1);
	}
}

void gxp_link_configure(struct eth_device *dev)
{
	struct gxp_umac_regs *umac_regs;
	struct gxp_umac_private *priv;
	struct phy_device *phydev;
	uint32_t tmp;

	umac_regs = (struct gxp_umac_regs *)dev->iobase;
	priv = (struct gxp_umac_private *)dev->priv;
	phydev = priv->phydev;

	// disable the MAC
	gxp_set_channel_enable(dev, 0);

	// Clock enable sequence:
	// - disable both clocks: gigabit and 100/10
	// - wait 2 microseconds
	// - enable appropriate clock
	// - wait 2 microseconds
	tmp = io_read4(&umac_regs->config_status);
	tmp &= 0xfffff9ff;
	io_write4(&umac_regs->config_status, tmp);
	udelay(2);

	tmp &= 0xfffff9fa;
	if (phydev->speed == 1000) {
		tmp |= (1<<10) | (1<<2);
	} else {
		tmp |= (1<<9);
	}
	if (phydev->duplex) {
		tmp |= 1;
	}

	io_write4(&umac_regs->config_status, tmp);
	udelay(2);

	// enable the MAC
	gxp_set_channel_enable(dev, 1);

	tmp = io_read4(&umac_regs->config_status);
}

static void gxp_initialize_hardware(struct eth_device *dev)
{
	uint32_t val;
	uint32_t tx_ring_size;
	uint32_t rx_ring_size;
	struct umac_struct *umac;
	struct gxp_umac_private *priv;
	struct gxp_umac_regs *umac_regs;

	umac_regs = (struct gxp_umac_regs *)dev->iobase;
	priv = (struct gxp_umac_private *)dev->priv;
	umac = &priv->umac;

	// initialize tx and rx rings to first entry
	io_write4(&umac_regs->ring_ptr, 0);

	// clear the missed bit
	io_write4(&umac_regs->clear_status, 0);

	// disable checksum generation
	io_write4(&umac_regs->cksum_config, 0);

	// write ring size register
	tx_ring_size = get_ring_size_value(UMAC_MAX_RING_ENTRIES);
	rx_ring_size = get_ring_size_value(UMAC_MAX_RING_ENTRIES);
	val = (tx_ring_size << UMAC_RING_SIZE_TX_SHIFT) | (rx_ring_size << UMAC_RING_SIZE_RX_SHIFT);
	io_write4(&umac_regs->ring_size, val);

	// write [tr]x ring base address
	io_write4(&umac_regs->rx_ring_addr, (uint32_t)umac->rxring.ring_base);
	io_write4(&umac_regs->tx_ring_addr, (uint32_t)umac->txring.ring_base);

	// write burst size
	io_write4(&umac_regs->dma_config, 0x22);

	// disable clocks and gigabit mode
	io_write4(&umac_regs->config_status, 0);

	gxp_link_configure(dev);
}

static int gxp_umac_write_hwaddr(struct eth_device *dev)
{
	struct gxp_umac_regs *umac_regs;
	uint32_t mac_address_16;

	umac_regs = (struct gxp_umac_regs *)dev->iobase;

	// 15:8 == bits[47:40] of the Ethernet MAC address
	//  7:0 == bits[39:32] of the Ethernet MAC address
	mac_address_16 = (dev->enetaddr[0] << 8) | (dev->enetaddr[1]);
	io_write4(&umac_regs->mac_addr_hi, mac_address_16);

	// 15:8 == bits[31:24] of the Ethernet MAC address
	//  7:0 == bits[23:16] of the Ethernet MAC address
	mac_address_16 = (dev->enetaddr[2] << 8) | (dev->enetaddr[3]);
	io_write4(&umac_regs->mac_addr_mid, mac_address_16);

	// 15:8 == bits[15:8] of the Ethernet MAC address
	//  7:0 == bits[7:0] of the Ethernet MAC address
	mac_address_16 = (dev->enetaddr[4] << 8) | (dev->enetaddr[5]);
	io_write4(&umac_regs->mac_addr_lo, mac_address_16);

	return(0);
}

static int gxp_umac_init(struct eth_device *dev, struct bd_info * bis)
{
	int ret;
	struct gxp_umac_private *priv;
	struct phy_device *phydev;

	priv = (struct gxp_umac_private *)dev->priv;
	phydev = priv->phydev;

	gxp_umac_reinitialize_data_structures(dev); // test

	gxp_initialize_hardware(dev);

	/* Start up the PHY */
	ret = phy_startup(priv->phydev);
	if (ret) {
		printf("Could not initialize PHY %s\n", priv->phydev->dev->name);
		return(0);
	}

	if (!phydev->link) {
		printf("%s: No link.\n", phydev->dev->name);
		return(0);
	}

	// configure the MAC to match the PHY: speed/duplex
	gxp_link_configure(dev);

	return 1;
}

static void gxp_umac_halt(struct eth_device *dev)
{
	struct gxp_umac_private *priv;

	priv = (struct gxp_umac_private *)dev->priv;

	// disable the MAC
	gxp_set_channel_enable(dev, 0);

	/* Shut down the PHY, as needed */
	phy_shutdown(priv->phydev);
}

static int gxp_umac_send(struct eth_device *dev, void *eth_data,
			    int data_length)
{
	int ret;

	ret = gxp_output(dev, eth_data, data_length);
	if (ret != 0) {
		printf("gxp_output returned %d\n", ret);
	}

	return 0;
}

static int gxp_input(struct eth_device *dev)
{
	struct gxp_umac_private *priv;
	struct umac_struct *umac;
	struct gxp_umac_regs *umac_regs;
	umac_ring_entry *next_entry;
	uint32_t val;

	priv = (struct gxp_umac_private *)dev->priv;
	umac = &priv->umac;
	umac_regs = (struct gxp_umac_regs *)dev->iobase;
	next_entry = umac->rxring.producer_ptr;

	val = io_read4(&umac_regs->interrupt);

	// if no rx interrupt, then no packet to receive
	if (!(val & UMAC_MAC_INT_CFG_REG_RX_INT_MASK)) {
		// there is not an interrupt every time we receive a packet
		// so returning here is not quite right
		// return(0);
	}

	if (val & UMAC_MAC_INT_CFG_REG_OVERRUN_INT_MASK) {
		printf("gxp_input - got overrun\n");
	}

	// write to clear interrupt bits that were set
	io_write4(&umac_regs->interrupt, val);

	val = io_read4(&umac_regs->config_status);
	if (val & UMAC_CFG_STAT_1_REG_MISSED_MASK) {
		printf("gxp_input - got missed\n");
		val = 0;
		io_write4(&umac_regs->clear_status, val);
	}

	dcache_inv_lines((unsigned long)(next_entry), (unsigned long)sizeof(*next_entry));

	if (next_entry->rx.status & UMAC_RING_ENTRY_HW_OWN) {
		if (next_entry->rx.status & UMAC_RING_RX_ERR_MASK) {
			printf("HW own but error status is 0x%x - clearing\n", next_entry->rx.status & UMAC_RING_RX_ERR_MASK);
			next_entry->rx.status = next_entry->rx.status & ~UMAC_RING_RX_ERR_MASK;
			dcache_inv_lines((unsigned long)(next_entry), (unsigned long)sizeof(*next_entry));
		}
		return(0);
	}

	net_process_received_packet((uchar *)next_entry->rx.dma_address, next_entry->rx.count);

	umac->rxring.producer_ptr = next_ring_entry(&umac->rxring, umac->rxring.producer_ptr);
	next_entry->rx.count = UMAC_MAX_FRAME_SIZE;
	next_entry->rx.status = UMAC_RING_ENTRY_HW_OWN;

	dcache_inv_lines((unsigned long)(next_entry), (unsigned long)sizeof(*next_entry));

	io_write4(&umac_regs->ring_prompt, 0);	// trigger it

	return next_entry->rx.count;
}

static int gxp_umac_recv(struct eth_device *dev)
{
	/*
	 * This command pulls one frame from the card
	 */
	int frame_length = 0;

	frame_length = gxp_input(dev);

	frame_length = -1;

	return frame_length;
}

static int init_phy(struct eth_device *dev)
{
	struct gxp_umac_private *priv;
	struct mii_dev *bus;
	struct phy_device *phydev;
	int tmp;
	int phy_addr;
	uint32_t supported = (SUPPORTED_10baseT_Half |
			SUPPORTED_10baseT_Full |
			SUPPORTED_100baseT_Half |
			SUPPORTED_100baseT_Full) |
			SUPPORTED_1000baseT_Full;

	priv = (struct gxp_umac_private *)dev->priv;
	bus = priv->external_bus;
	phy_addr = priv->external_dev_num;

	// set phy mode to SGMII to copper
	// set page to 18 by writing 18 to register 22
	gxp_phy_write(bus, phy_addr, 0, 22, 18);

	// read page 18, register 20: General Control Register 1
	tmp = gxp_phy_read(bus, phy_addr, 0, 20);

	// set mode bits to 001 (SGMII to copper aka SGMII system)
	tmp &= 0xfffffff8;
	tmp |= 1;
	gxp_phy_write(bus, phy_addr, 0, 20, tmp);

	// perform mode reset by setting bit 15 in register 20
	gxp_phy_write(bus, phy_addr, 0, 20, tmp | 0x8000);

	// waiting for mode reset to complete
	while(gxp_phy_read(bus, phy_addr, 0, 20) & 0x8000);

	// after setting the mode, must perform a SW reset
	// set page to 18 by writing 0 to register 22
	gxp_phy_write(bus, phy_addr, 0, 22, 0);

	// read register 0
	tmp = gxp_phy_read(bus, phy_addr, 0, 0);
	tmp |= 0x8000;
	gxp_phy_write(bus, phy_addr, 0, 20, tmp);

	// wait for reset bit to automatically clear
	while(gxp_phy_read(bus, phy_addr, 0, 0) & 0x8000);

	priv->interface = PHY_INTERFACE_MODE_SGMII;
	phydev = phy_connect(priv->external_bus, priv->external_dev_num, dev, priv->interface);

	phydev->supported &= supported;
	phydev->advertising = phydev->supported;

	priv->phydev = phydev;

	phy_config(phydev);

	return 1;
}

#ifdef CONFIG_GXP_UMAC_G10P
static int init_phy_gen10p(struct eth_device *dev)
{
	struct gxp_umac_private *priv;
	struct mii_dev *bus;
	struct phy_device *phydev;
	int tmp;
	int phy_addr;
	int r, w, s, m;
	uint32_t supported = SUPPORTED_1000baseT_Full;

	priv = (struct gxp_umac_private *)dev->priv;
	bus = priv->internal_bus;
	phy_addr = priv->internal_dev_num;

	tmp = gxp_phy_read(bus, phy_addr, 0, 0);

	if (tmp & 0x4000) {
		printf("Internal PHY%d loopback is enabled - clearing\n", phy_addr);
	}

	tmp &= ~0x4000;	// clear loopback
	gxp_phy_write(bus, phy_addr, 0, 0, tmp);

	// Enable sideband
	r = readb(0xD1000040);
	m = 0x2; //mask
	s = 0x2;
	w = (r & ~m) | (s & m);
	writeb((uint8_t)w, 0xD1000040);

	// When the reset operation is complete, bit 15 will be cleared
	while(gxp_phy_read(bus, phy_addr, 0, 0) & 0x8000);

	priv->interface = PHY_INTERFACE_MODE_GMII;
	phydev = phy_connect(priv->internal_bus, priv->internal_dev_num, dev, priv->interface);

	phydev->supported &= supported;
	phydev->advertising = phydev->supported;

	phydev->autoneg = !AUTONEG_ENABLE;

	priv->phydev = phydev;

	phy_config(phydev);

	return 1;
}
#endif

void gxp_setup_internal_phy(struct eth_device *dev)
{
	struct gxp_umac_private *priv;
	struct mii_dev *bus;
	int tmp;
	int phy_addr;

	priv = (struct gxp_umac_private *)dev->priv;
	bus = priv->internal_bus;
	phy_addr = priv->internal_dev_num;

	tmp = gxp_phy_read(bus, phy_addr, 0, 0);

	if (tmp & 0x4000) {
		printf("Internal PHY%d loopback is enabled - clearing\n", phy_addr);
	}

	tmp &= ~0x4000;	// clear loopback
	gxp_phy_write(bus, phy_addr, 0, 0, tmp);

	tmp = gxp_phy_read(bus, phy_addr, 0, 0);
	tmp |= 0x1000;	// set aneg enable if not in failover mode

	gxp_phy_write(bus, phy_addr, 0, 0, tmp | 0x8000);

	// When the reset operation is complete, bit 15 will be cleared
	while(gxp_phy_read(bus, phy_addr, 0, 0) & 0x8000);

	return;
}

static int gxp_umac_ll_register(struct bd_info * bis, struct mii_dev *external_mdio_bus, int external_dev_num,
		 struct mii_dev *internal_mdio_bus, int internal_dev_num)
{
	struct eth_device *dev;
	struct gxp_umac_private *priv;
	int rc;

	dev = (struct eth_device *) malloc(sizeof(*dev));
	if (NULL == dev) {
		return(0);
	}

	memset(dev, 0, sizeof *dev);
	priv = (struct gxp_umac_private *)malloc(sizeof(struct gxp_umac_private));
	if (NULL == priv) {
		free(dev);
		return(0);
	}

	memset(priv, 0, sizeof(struct gxp_umac_private));
	priv->external_bus = external_mdio_bus;
	priv->external_dev_num = external_dev_num;
	priv->internal_bus = internal_mdio_bus;
	priv->internal_dev_num = internal_dev_num;

	/* DRIVER: Name of the device */
	sprintf(dev->name, "GXP_UMAC%d", internal_dev_num);
	/* DRIVER: Register base address */
	dev->iobase = UMAC0_BASE_ADDR + (0x1000 * internal_dev_num);
	/* DRIVER: Device initialization function */
	dev->init = gxp_umac_init;
	/* DRIVER: Function for sending packets */
	dev->send = gxp_umac_send;
	/* DRIVER: Function for receiving packets */
	dev->recv = gxp_umac_recv;
	/* DRIVER: Function to cease operation of the device */
	dev->halt = gxp_umac_halt;
	/* DRIVER: Function to change ethernet MAC address */
	dev->write_hwaddr = gxp_umac_write_hwaddr;
	/* DRIVER: Driver's private data */
	dev->priv = (void *)priv;
  
	eth_register(dev);

	rc = gxp_umac_initialize_data_structures(dev);
	if (rc != 1) {
		return(0);
	}

	// disable the MAC
	gxp_set_channel_enable(dev, 0);

#ifdef CONFIG_GXP_UMAC_G10P
	printf("%s: UMAC for Gen10P \n", __FUNCTION__);
	return init_phy_gen10p(dev);
#else
	// init the internal PHYs
	gxp_setup_internal_phy(dev);

	/* Try to initialize the external PHY here, and return */
	return init_phy(dev);
#endif
}

/*
 * Initialize all GXP umac devices
 *
 * Return the number of devices initialized
 */
int gxp_umac_register(struct bd_info * bis)
{
	int rc;
	int init_count = 0;
	struct gxp_mdio_info mdio_info;
	struct mii_dev *external_mdio_bus;
	struct mii_dev *internal_mdio_bus;

#ifndef CONFIG_GXP_UMAC_G10P
	// register the GXP mdio bus handlers
	mdio_info.addr = (struct gxp_mdio_regs *)EXTERNAL_MDIO_BASE_ADDR;
	mdio_info.name = GXP_EXTERNAL_MII_NAME;

	rc = gxp_mdio_init(&mdio_info);
	if (rc == 0) {
		external_mdio_bus = mdio_info.bus;
	}
	else {
		printf("%s: gxp_mdio_init() failed for %s\n", __FUNCTION__, GXP_EXTERNAL_MII_NAME);
		return(-1); 	// can't work without mdio
	}
#endif

	mdio_info.addr = (struct gxp_mdio_regs *)INTERNAL_MDIO_BASE_ADDR;
	mdio_info.name = GXP_INTERNAL_MII_NAME;

	rc = gxp_mdio_init(&mdio_info);
	if (rc == 0) {
		internal_mdio_bus = mdio_info.bus;
	}
	else {
		printf("%s: gxp_mdio_init() failed for %s\n", __FUNCTION__, GXP_INTERNAL_MII_NAME);
		return(-1); 	// can't work without mdio
	}

	rc = gxp_umac_ll_register(bis, external_mdio_bus, 0, internal_mdio_bus, 0);
	if (rc == 1) {
		printf("gxp_umac_register success!!\n");
		init_count++;
	}
	else if (rc < 0) {
		return(rc);
	}

	asm("nop");
	return(init_count);
}
