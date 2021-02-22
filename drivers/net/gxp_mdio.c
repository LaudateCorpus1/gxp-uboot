// SPDX-License-Identifier: GPL-2.0
/*
 * GXP UMAC driver
 *
 * (C) Copyright 2020 Hewlett Packard Enterprise Development LP.
 *
*/

#include <common.h>
#include <miiphy.h>
#include <phy.h>
#include <gxp_mdio.h>

uint32_t io_read4(uint32_t *sys_addr)
{
	return *(volatile uint32_t *)sys_addr;
}

void io_write4(uint32_t *sys_addr, uint32_t value)
{
	*(volatile uint32_t *)sys_addr = value;
}

int gxp_phy_read(struct mii_dev *bus, int phy_addr, int dev_addr, int regnum)
{
	struct gxp_mdio_regs *phyregs = bus->priv;
	uint32_t tmp;

	tmp = io_read4(&phyregs->mmi);
	tmp &= ~(UMAC_MMI_PHY_ADDR_MASK | UMAC_MMI_REG_ADDR);
	tmp |= (phy_addr << UMAC_MMI_PHY_ADDR_SHIFT) & UMAC_MMI_PHY_ADDR_MASK;
	tmp |= regnum & UMAC_MMI_REG_ADDR;

	// this is a read op
	tmp |= UMAC_MMI_MRNW;
	io_write4(&phyregs->mmi, tmp);

	// activate transfer
	tmp |= UMAC_MMI_MOWNER;
	io_write4(&phyregs->mmi, tmp);

	// waiting for mowner to clear...
	while(tmp & UMAC_MMI_MOWNER) {
		tmp = io_read4(&phyregs->mmi);
	}

	tmp = io_read4(&phyregs->mmi_data);

	return (tmp & UMAC_MMI_DATA_MASK);
}

int gxp_phy_write(struct mii_dev *bus, int phy_addr, int dev_addr, int regnum,
			u16 value)
{
	struct gxp_mdio_regs *phyregs = bus->priv;
	uint32_t tmp;

	// store the data to be written to the phy reg
	io_write4(&phyregs->mmi_data, (((uint32_t)(value)) & UMAC_MMI_DATA_MASK));

	tmp = io_read4(&phyregs->mmi);
	tmp &= ~(UMAC_MMI_PHY_ADDR_MASK | UMAC_MMI_REG_ADDR);
	tmp |= (phy_addr << UMAC_MMI_PHY_ADDR_SHIFT) & UMAC_MMI_PHY_ADDR_MASK;
	tmp |= regnum & UMAC_MMI_REG_ADDR;

	// this is a write op
	tmp &= ~UMAC_MMI_MRNW;
	io_write4(&phyregs->mmi, tmp);

	// activate transfer
	tmp |= UMAC_MMI_MOWNER;
	io_write4(&phyregs->mmi, tmp);

	// waiting for mowner to clear...
	while(tmp & UMAC_MMI_MOWNER) {
		tmp = io_read4(&phyregs->mmi);
	}

	return 0;
}

int gxp_mdio_init(struct gxp_mdio_info *info)
{
	struct mii_dev *bus = mdio_alloc();

	if (!bus) {
		printf("Failed to allocate GXP MDIO bus\n");
		return -1;
	}

	bus->read = gxp_phy_read;
	bus->write = gxp_phy_write;

	strcpy(bus->name, info->name);

	bus->priv = (void *)info->addr;

	info->bus = bus;

	return mdio_register(bus);
}
