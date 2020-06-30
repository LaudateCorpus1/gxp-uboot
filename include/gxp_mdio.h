// SPDX-License-Identifier: GPL-2.0
/*
 * GXP UMAC driver
 *
 * (C) Copyright 2020 Hewlett Packard Enterprise Development LP.
 *
*/

#ifndef __GXP_PHY_H__
#define __GXP_PHY_H__

#include <net.h>
#include <miiphy.h>

// mmi register masks
#define UMAC_MAX_PHY            31
#define UMAC_MMI_NMRST          0x00008000
#define UMAC_MMI_PHY_ADDR_MASK  0x001F0000
#define UMAC_MMI_PHY_ADDR_SHIFT 16
#define UMAC_MMI_MOWNER         0x00000200
#define UMAC_MMI_MRNW           0x00000100
#define UMAC_MMI_REG_ADDR       0x0000001F

// mmi data register mask
#define UMAC_MMI_DATA_MASK      0x0000FFFF

struct gxp_mdio_regs {
	uint32_t mmi;              		//  R/W	MMI Register
	uint32_t mmi_data;         		//  R/W	MMI Data Register
};

struct gxp_mdio_info {
	struct gxp_mdio_regs *addr;
	char *name;
	struct mii_dev *bus;
};

uint32_t io_read4(uint32_t *sys_addr);
void io_write4(uint32_t *sys_addr, uint32_t value);

int gxp_phy_read(struct mii_dev *bus, int phy_addr, int dev_addr, int regnum);
int gxp_phy_write(struct mii_dev *bus, int phy_addr, int dev_addr, int regnum, u16 value);
int gxp_mdio_init(struct gxp_mdio_info *);

#endif /* __GXP_PHY_H__ */
